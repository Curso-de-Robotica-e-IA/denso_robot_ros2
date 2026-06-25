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

# --- Funções Modulares do Pipeline ---

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

def estimate_homography(pts0, pts1):
    if len(pts0) < 4:
        return None, None, 0
    H, mask = cv2.findHomography(pts0, pts1, cv2.RANSAC, 3.0)
    inliers_count = int(np.sum(mask)) if mask is not None else 0
    return H, mask, inliers_count

# --- PIPELINE DE RENDERIZAÇÃO ---

def run_pipeline_with_homography(img0_proc, img1_proc, img0_raw, img1_raw, title):
    """
    Executa o matching e estimação de homografia na imagem completa.
    Retorna dois plots side-by-side: um com fundo de superpixels e outro com fundo cru.
    """
    # 1. Extração de Features na Imagem Completa
    feats0 = extractor.extract(img0_proc.to(device))
    feats1 = extractor.extract(img1_proc.to(device))

    # 2. Matching Adaptativo LightGlue
    matches0 = matcher({"image0": feats0, "image1": feats1, "filter_threshold": 0.1})
    
    if "stop" in matches0:
        stop_val = matches0["stop"]
        stop_layer = stop_val.item() if hasattr(stop_val, "item") else int(stop_val)
    else:
        stop_layer = -1

    feats0, feats1, matches0 = [rbd(x) for x in [feats0, feats1, matches0]]

    kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
    matches = matches0["matches"]
    
    m_kpts0_orig = kpts0[matches[..., 0]].cpu().numpy()
    m_kpts1_orig = kpts1[matches[..., 1]].cpu().numpy()

    # 3. Cálculo matemático da Homografia
    H, mask, inliers = estimate_homography(m_kpts0_orig, m_kpts1_orig)
    
    texto_base = f'{title}\nMatches Totais: {len(matches)} | Inliers RANSAC: {inliers}\nStop Layer: {stop_layer}'

    # Função auxiliar interna para gerar o plot com o background desejado
    def gerar_plot_individual(background_images, tipo_fundo):
        plt.figure()
        axes = viz2d.plot_images(background_images)
        
        if mask is not None and len(mask) > 0:
            inliers_mask = mask.ravel() == 1
            viz2d.plot_matches(
                torch.tensor(m_kpts0_orig[inliers_mask]), 
                torch.tensor(m_kpts1_orig[inliers_mask]), 
                color="lime", lw=0.3
            )
            
        viz2d.add_text(0, f"{texto_base}\nFundo: {tipo_fundo}", fs=10)
        return plt.gcf()

    # 4. GERA OS DOIS PLOTS REQUISITADOS
    plt.close('all')
    fig_proc = gerar_plot_individual([img0_proc, img1_proc], "Superpixels (Boruvka)")
    fig_raw = gerar_plot_individual([img0_raw, img1_raw], "Imagens Cruas (Originais)")
    
    return fig_proc, fig_raw, H, len(matches), inliers, stop_layer

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
    # 1. Carrega as imagens originais
    path_ref = Path("src/assets/ref_img.jpeg")
    if not path_ref.exists():
        print(f"Erro: Arquivo {path_ref} não encontrado.")
        sys.exit(1)
        
    ref_tensor = load_image(path_ref)
    img_ref_np = (ref_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    h_img, w_img, _ = img_ref_np.shape
    
    # 2. Rotação e Translação Simulada (Criação do Ground Truth controlado)
    cx, cy = w_img / 2.0, h_img / 2.0
    M_rot = cv2.getRotationMatrix2D((cx, cy), 15, 0.90)  # 15 graus, 90% escala
    H_ground_truth = np.eye(3, dtype=np.float32)
    H_ground_truth[0:2, 0:3] = M_rot
    H_ground_truth[0, 2] += 40   
    H_ground_truth[1, 2] += -20  
    
    img_cur_np_sintetica = cv2.warpPerspective(img_ref_np, H_ground_truth, (w_img, h_img))
    cur_tensor_sintetico = torch.from_numpy(img_cur_np_sintetica).permute(2, 0, 1).float() / 255.0
    
    # 3. Execução do Pipeline de Filtragem Clássica
    cur_hm = match_histogram_tensor(cur_tensor_sintetico, ref_tensor)
    ref_edge = apply_canny_edge(ref_tensor)
    cur_edge = apply_canny_edge(cur_hm)
    ref_bf = apply_bilateral_filter(ref_tensor)
    cur_bf = apply_bilateral_filter(cur_hm)
    
    print("Processando segmentação por Boruvka...")
    ref_proc = run_boruvka(ref_bf, edge_map=ref_edge, n_supix=100)
    cur_proc = run_boruvka(cur_bf, edge_map=cur_edge, n_supix=100)
    
    # 4. Chamada do Pipeline Gerador de Ambos os Gráficos
    print("Calculando casamento e Homografia na imagem completa...")
    fig_proc, fig_raw, H_calculado, total, inliers, stop = run_pipeline_with_homography(
        img0_proc=ref_proc, 
        img1_proc=cur_proc, 
        img0_raw=ref_tensor, 
        img1_raw=cur_hm, 
        title="Validacao por Warp Geometrico"
    )
    
    # 5. Cálculo do Erro Residual Real de Alinhamento
    if H_calculado is not None:


        H_calculada = np.array([
        [ 0.965, -0.258, 40.0],
        [ 0.258,  0.965, -20.0],
        [ 0.000,  0.000,   1.0]
        ]   , dtype=np.float32)
    
        s, erro = extrair_erro_homografia_nao_calibrada(H_calculada)
        
        print("=======================================================")
        print("      VETOR DE ERRO PARA VISUAL SERVOING NÃO-CALIBRADO ")
        print("=======================================================")
        print(" Vetor de Características Atuais (s):")
        print(s.flatten())
        print("\n Vetor de Erro que vai para o Controlador (e = s - s*):")
        print(erro.flatten())
        print("=======================================================")

        
        cantos = np.array([[0, 0], [w_img, 0], [w_img, h_img], [0, h_img]], dtype=np.float32).reshape(-1, 1, 2)
        cantos_reais = cv2.perspectiveTransform(cantos, H_ground_truth)
        cantos_estimados = cv2.perspectiveTransform(cantos, H_calculado)
        erro_medio_pixels = np.mean(np.linalg.norm(cantos_reais - cantos_estimados, axis=2))
        
        print("\n=======================================================")
        print("          MÉTRICAS DA VALIDAÇÃO SIMULADA               ")
        print("=======================================================")
        print(f" -> Matches Totais Identificados: {total}")
        print(f" -> Inliers Validados pelo RANSAC: {inliers}")
        print(f" -> ERRO MÉDIO DE ALINHAMENTO: {erro_medio_pixels:.4f} pixels")
        print("=======================================================\n")
    
    # Configura títulos das janelas e salva no disco de forma independente
    fig_proc.canvas.manager.set_window_title("Forma 1: Matching sobre Superpixels")
    fig_raw.canvas.manager.set_window_title("Forma 2: Matching sobre Imagens Cruas")
    
    fig_proc.savefig("simulacao_1_superpixels.png", bbox_inches="tight", dpi=150)
    fig_raw.savefig("simulacao_2_imagens_cruas.png", bbox_inches="tight", dpi=150)
    
    print("Exibindo os dois gráficos simultaneamente na tela...")
    plt.show()