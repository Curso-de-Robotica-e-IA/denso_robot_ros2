import sys
import csv
from pathlib import Path
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt
from skimage.exposure import match_histograms
from datetime import datetime

# Adiciona o módulo boruvka-superpixel ao path
sys.path.insert(0, "boruvka-superpixel/pybuild")
import boruvka_superpixel

from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd
from lightglue import viz2d

# --- Configurações ---
torch.set_grad_enabled(False)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Rodando em: {device}")

extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(feature="superpoint", flash=True).eval().to(device)

# --- Funções Modulares Existentes ---

def match_histogram_tensor(source_tensor, template_tensor):
    src_np = source_tensor.permute(1, 2, 0).cpu().numpy()
    tmpl_np = template_tensor.permute(1, 2, 0).cpu().numpy()
    matched_np = match_histograms(src_np, tmpl_np, channel_axis=2)
    matched_np = np.clip(matched_np, 0.0, 1.0)
    return torch.from_numpy(matched_np).permute(2, 0, 1).float()

def apply_bilateral_filter(image_tensor):
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    img_np = np.ascontiguousarray(img_np)
    img_filtered = cv2.bilateralFilter(img_np, d=9, sigmaColor=75, sigmaSpace=75)
    return torch.from_numpy(img_filtered).permute(2, 0, 1).float() / 255.0

def apply_canny_edge(image_tensor):
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    img_np = np.ascontiguousarray(img_np)
    return cv2.Canny(img_np, 50, 150)

def run_boruvka(image_tensor, edge_map, n_supix):
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    img_np = np.ascontiguousarray(img_np)
    if edge_map is None:
        edge_map = np.zeros(img_np.shape[:2], dtype=np.uint8)
    bosupix = boruvka_superpixel.BoruvkaSuperpixel()
    bosupix.build_2d(img_np, edge_map)
    out = bosupix.average(n_supix, 3, img_np)
    return torch.from_numpy(out).permute(2, 0, 1).float() / 255.0

def crop_center_tensor(image_tensor, pct_w=0.3, pct_h=0.3):
    C, H, W = image_tensor.shape
    crop_w, crop_h = int(W * pct_w), int(H * pct_h)
    x_start, y_start = (W - crop_w) // 2, (H - crop_h) // 2
    cropped_tensor = image_tensor[:, y_start:y_start+crop_h, x_start:x_start+crop_w]
    return cropped_tensor, (x_start, y_start)

def estimate_homography(pts0, pts1):
    if len(pts0) < 4:
        print("Aviso: Menos de 4 pontos para homografia.")
        return None, None, 0
    H, mask = cv2.findHomography(pts1, pts0, cv2.RANSAC, 3.0)
    inliers_count = int(np.sum(mask)) if mask is not None else 0
    return H, mask, inliers_count

# --- PIPELINE ATUALIZADO (Plota Duas Formas Simultaneamente) ---

def run_pipeline_with_homography(img0_proc, img1_proc, img0_raw, img1_raw, title, use_roi=True, pct_w=0.3, pct_h=0.3):
    """
    Roda o pipeline do LightGlue e Homografia uma única vez, mas gera e retorna 
    duas figuras distintas: uma sobre o background de superpixels e outra sobre o background cru.
    """
    # 1. Definição da Região de Busca baseada nos Tensores Processados (Boruvka)
    if use_roi:
        img0_search, offset0 = crop_center_tensor(img0_proc, pct_w, pct_h)
        img1_search, offset1 = crop_center_tensor(img1_proc, pct_w, pct_h)
    else:
        img0_search, offset0 = img0_proc, (0, 0)
        img1_search, offset1 = img1_proc, (0, 0)
    
    # 2. Extração de Features na Região dos Superpixels
    feats0 = extractor.extract(img0_search.to(device))
    feats1 = extractor.extract(img1_search.to(device))

    # 3. Matching Adaptativo LightGlue
    matches0 = matcher({"image0": feats0, "image1": feats1, "filter_threshold": 0.1})
    
    if "stop" in matches0:
        stop_val = matches0["stop"]
        stop_layer = stop_val.item() if hasattr(stop_val, "item") else int(stop_val)
    else:
        stop_layer = -1

    feats0, feats1, matches0 = [rbd(x) for x in [feats0, feats1, matches0]]

    kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
    matches = matches0["matches"]
    
    m_kpts0_crop = kpts0[matches[..., 0]]
    m_kpts1_crop = kpts1[matches[..., 1]]

    # 4. Translação de Coordenadas para o Espaço Original (Tamanho total da Imagem)
    offset0_tensor = torch.tensor([offset0[0], offset0[1]], device=m_kpts0_crop.device)
    offset1_tensor = torch.tensor([offset1[0], offset1[1]], device=m_kpts1_crop.device)
    
    m_kpts0_orig = (m_kpts0_crop + offset0_tensor).cpu().numpy()
    m_kpts1_orig = (m_kpts1_crop + offset1_tensor).cpu().numpy()

    # 5. Cálculo Único da Homografia com RANSAC
    H, mask, inliers = estimate_homography(m_kpts0_orig, m_kpts1_orig)
    
    W0, H0 = img0_raw.shape[2], img0_raw.shape[1]
    roi_status = "ROI Ativa (30%)" if use_roi else "Imagem Completa"
    texto_plot = f'{title} [{roi_status}]\nMatches Totais: {len(matches)} | Inliers RANSAC: {inliers}\nStop Layer: {stop_layer}'

    # -------------------------------------------------------------------------
    # FUNÇÃO INTERNA AUXILIAR PARA EVITAR DUPLICAÇÃO DE DESENHO
    # -------------------------------------------------------------------------
    def desenhar_grafico(background_images, sufixo_titulo):
        plt.figure() # Abre uma nova janela/figura limpa
        axes = viz2d.plot_images(background_images)
        
        if mask is not None and len(mask) > 0:
            inliers_mask = mask.ravel() == 1
            viz2d.plot_matches(
                torch.tensor(m_kpts0_orig[inliers_mask]), 
                torch.tensor(m_kpts1_orig[inliers_mask]), 
                color="lime", lw=0.3
            )
        
        if use_roi:
            rect_roi = plt.Rectangle((offset0[0], offset0[1]), W0*pct_w, H0*pct_h, 
                                     edgecolor="red", facecolor="none", linestyle="--", linewidth=1.5)
            plt.gca().add_patch(rect_roi)
        
        if H is not None:
            if use_roi:
                cantos_base = np.array([
                    [offset0[0], offset0[1]], [offset0[0] + W0*pct_w, offset0[1]],
                    [offset0[0] + W0*pct_w, offset0[1] + H0*pct_h], [offset0[0], offset0[1] + H0*pct_h]
                ], dtype=np.float32).reshape(-1, 1, 2)
            else:
                cantos_base = np.array([[0, 0], [W0, 0], [W0, H0], [0, H0]], dtype=np.float32).reshape(-1, 1, 2)
            
            cantos_projetados = cv2.perspectiveTransform(cantos_base, np.linalg.inv(H))
            cantos_plot = cantos_projetados.squeeze() + np.array([W0, 0])
            
            polygon = plt.Polygon(cantos_plot, edgecolor="cyan", facecolor="none", linewidth=2.5, linestyle="-")
            plt.gca().add_patch(polygon)

        viz2d.add_text(0, f"{texto_plot}\nFundo: {sufixo_titulo}", fs=10)
        return plt.gcf()

    # 6. Gera as duas figuras com backgrounds diferentes usando o mesmo cálculo matemático
    plt.close('all') 
    fig_proc = desenhar_grafico([img0_proc, img1_proc], "Superpixels (Boruvka)")
    fig_raw = desenhar_grafico([img0_raw, img1_raw], "Imagens Cruas (Originais)")
    
    return fig_proc, fig_raw, len(matches), inliers, stop_layer

def extrair_erro_homografia_nao_calibrada(H):
    """
    Extrai o vetor de características visuais e o vetor de erro 
    para sistemas de Visual Servoing Não-Calibrado.
    
    Parâmetros:
    ----------
    H : np.ndarray (3x3)
        Matriz de homografia calculada pelo RANSAC.
        
    Retorna:
    -------
    s : np.ndarray (8x1)
        Vetor com os 8 parâmetros independentes da homografia atual.
    e : np.ndarray (8x1)
        Vetor de erro (s - s*) que alimenta a malha fechada do robô.
    """
    if H is None:
        return None, None
        
    # 1. Garante a normalização dividindo toda a matriz pelo último elemento (H[2,2] = 1)
    H_norm = H / H[2, 2]
    
    # 2. Extrai as 8 componentes independentes (linhas 0, 1 e os dois primeiros elementos da linha 2)
    s = np.array([
        H_norm[0, 0], H_norm[0, 1], H_norm[0, 2],
        H_norm[1, 0], H_norm[1, 1], H_norm[1, 2],
        H_norm[2, 0], H_norm[2, 1]
    ], dtype=np.float32).reshape(8, 1)
    
    # 3. Define o vetor de destino s* (representando a Matriz Identidade)
    s_estrela = np.array([1, 0, 0, 0, 1, 0, 0, 0], dtype=np.float32).reshape(8, 1)
    
    # 4. Calcula o vetor de erro geométrico puro
    e = s - s_estrela
    
    return s, e

if __name__ == "__main__":
    # 1. Carrega as imagens originais do disco
    path_ref = Path("src/assets/ref_img.jpeg")
    path_cur = Path("src/assets/current_img.jpeg")
    
    if not path_ref.exists() or not path_cur.exists():
        print(f"Erro: Certifique-se de que as imagens existem em 'src/assets/'.")
        sys.exit(1)

    ref_tensor = load_image(path_ref)
    cur_tensor = load_image(path_cur)

    # 2. Aplica as etapas preliminares nas imagens
    cur_hm = match_histogram_tensor(cur_tensor, ref_tensor)

    ref_edge = apply_canny_edge(ref_tensor)
    cur_edge = apply_canny_edge(cur_hm)

    ref_tensor_bilateral = apply_bilateral_filter(ref_tensor)
    cur_tensor_bilateral = apply_bilateral_filter(cur_hm)

    # 3. Executa a redução por Superpixels do Borůvka (SH)
    print("Processando segmentação por Boruvka...")
    ref_processada = run_boruvka(ref_tensor_bilateral, edge_map=ref_edge, n_supix=100)
    cur_processada = run_boruvka(cur_tensor_bilateral, edge_map=cur_edge, n_supix=100)

    # 4. Alimenta o pipeline para gerar as duas formas de plotagem
    print("Calculando casamento com LightGlue e gerando visualizações...")
    fig_proc, fig_raw, total, inliers, stop = run_pipeline_with_homography(
        img0_proc=ref_processada,       # Para os cálculos matemáticos
        img1_proc=cur_processada,       # Para os cálculos matemáticos
        img0_raw=ref_tensor,            # Para o background original limpo
        img1_raw=cur_hm,                # Para o background original com HM
        title="Teste Homografia",
        use_roi=False                   # Defina como True se quiser ativar a ROI central de 30%
    )

    # 5. CONFIGURAÇÃO, SALVAMENTO E EXIBIÇÃO DAS JANELAS
    fig_proc.canvas.manager.set_window_title("Forma 1: Fundo Reconstruído por Superpixels")
    fig_raw.canvas.manager.set_window_title("Forma 2: Fundo de Imagens Cruas (Originais)")
    
    # Salva ambas as imagens de forma independente
    fig_proc.savefig("resultado_1_superpixels.png", bbox_inches="tight", dpi=150)
    fig_raw.savefig("resultado_2_imagens_cruas.png", bbox_inches="tight", dpi=150)
    print("\n=> Sucesso! Foram salvos dois ficheiros no disco:")
    print("   - 'resultado_1_superpixels.png'")
    print("   - 'resultado_2_imagens_cruas.png'")
    
    # Exibe as duas janelas interativas na tela ao mesmo tempo
    print("\nAbrindo ambas as formas gráficas no ecrã...")
    plt.show()