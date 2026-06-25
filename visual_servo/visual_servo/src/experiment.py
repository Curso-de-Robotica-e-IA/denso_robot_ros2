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

# Instancia os modelos com suporte adaptativo de velocidade do artigo
matcher = LightGlue(
    feature="superpoint",
    flash=True,               # Aceleração de hardware se disponível
    depth_confidence=0.95,    # Adaptive Depth ativado
    width_confidence=0.99     # Adaptive Width ativado
).eval().to(device)

extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)

# --- Funções Modulares do Pipeline ---

def match_histogram_tensor(source_tensor, template_tensor):
    """Fase: Histogram Matching (HM)"""
    src_np = source_tensor.permute(1, 2, 0).cpu().numpy()
    tmpl_np = template_tensor.permute(1, 2, 0).cpu().numpy()
    matched_np = match_histograms(src_np, tmpl_np, channel_axis=2)
    matched_np = np.clip(matched_np, 0.0, 1.0)
    return torch.from_numpy(matched_np).permute(2, 0, 1).float()

def apply_bilateral_filter(image_tensor):
    """Fase: Bilateral Filter (BF)"""
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    img_np = np.ascontiguousarray(img_np)
    img_filtered = cv2.bilateralFilter(img_np, d=9, sigmaColor=75, sigmaSpace=75)
    return torch.from_numpy(img_filtered).permute(2, 0, 1).float() / 255.0

def apply_canny_edge(image_tensor):
    """Fase: Canny Edge Detection (CE)"""
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    img_np = np.ascontiguousarray(img_np)
    edge_map = cv2.Canny(img_np, 50, 150)
    return edge_map

def run_boruvka(image_tensor, edge_map, n_supix):
    """Fase: Boruvka Superpixel (SH)"""
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    img_np = np.ascontiguousarray(img_np)
    
    if edge_map is None:
        edge_map = np.zeros(img_np.shape[:2], dtype=np.uint8)
        
    bosupix = boruvka_superpixel.BoruvkaSuperpixel()
    bosupix.build_2d(img_np, edge_map)
    out = bosupix.average(n_supix, 3, img_np)
    
    return torch.from_numpy(out).permute(2, 0, 1).float() / 255.0

def crop_center_tensor(image_tensor, pct_w=0.3, pct_h=0.5):
    """Fase: Focalização de Atenção (ROI) - Recorta o centro da imagem"""
    C, H, W = image_tensor.shape
    crop_w = int(W * pct_w)
    crop_h = int(H * pct_h)
    
    x_start = (W - crop_w) // 2
    y_start = (H - crop_h) // 2
    x_end = x_start + crop_w
    y_end = y_start + crop_h
    
    cropped_tensor = image_tensor[:, y_start:y_end, x_start:x_end]
    return cropped_tensor, (x_start, y_start)

def run_lightglue_pipeline_with_roi(img0_orig, img1_orig, title, pct_w=0.3, pct_h=0.5):
    """Módulo de Extração e Matching com ROI e Correção para CPU/GPU"""
    # 1. Aplica o Crop Central em ambas as imagens (Reduz região de busca)
    img0_crop, offset0 = crop_center_tensor(img0_orig, pct_w, pct_h)
    img1_crop, offset1 = crop_center_tensor(img1_orig, pct_w, pct_h)
    
    img0_dev = img0_crop.to(device)
    img1_dev = img1_crop.to(device)

    # 2. Extração de Features restrita à ROI
    feats0 = extractor.extract(img0_dev)
    feats1 = extractor.extract(img1_dev)

    # 3. Matching Adaptativo
    matches0 = matcher({
        "image0": feats0, 
        "image1": feats1,
        "filter_threshold": 0.1  # Poda de pontos irrelevantes
    })
    
    # SOLUÇÃO DO ERRO CRÍTICO: Tratamento seguro para inteiros em CPU e tensores em GPU
    if "stop" in matches0:
        stop_val = matches0["stop"]
        stop_layer = stop_val.item() if hasattr(stop_val, "item") else int(stop_val)
    else:
        stop_layer = -1

    # Remove batch dimension
    feats0, feats1, matches0 = [rbd(x) for x in [feats0, feats1, matches0]]

    kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
    matches = matches0["matches"]
    
    m_kpts0_crop = kpts0[matches[..., 0]]
    m_kpts1_crop = kpts1[matches[..., 1]]

    count = len(matches)
    
    # 4. Translação de Coordenadas de volta para o espaço da Imagem Original
    offset0_tensor = torch.tensor([offset0[0], offset0[1]], device=m_kpts0_crop.device)
    offset1_tensor = torch.tensor([offset1[0], offset1[1]], device=m_kpts1_crop.device)
    
    m_kpts0_orig = m_kpts0_crop + offset0_tensor
    m_kpts1_orig = m_kpts1_crop + offset1_tensor

    # 5. Renderização do gráfico na Imagem Original
    plt.close('all') 
    axes = viz2d.plot_images([img0_orig, img1_orig])
    viz2d.plot_matches(m_kpts0_orig, m_kpts1_orig, color="lime", lw=0.2)
    
    # Desenha o quadrado tracejado vermelho da ROI na imagem de referência
    W0, H0 = img0_orig.shape[2], img0_orig.shape[1]
    rect = plt.Rectangle((offset0[0], offset0[1]), W0 * pct_w, H0 * pct_h, 
                         edgecolor="red", facecolor="none", linestyle="--", linewidth=1.5)
    plt.gca().add_patch(rect)
    
    viz2d.add_text(0, f'{title}\nMatches na ROI: {count} | Stop Layer: {stop_layer}', fs=12)
    fig = plt.gcf()
    
    return fig, count, stop_layer

# --- EXECUÇÃO EXPERIMENTAL ---

path_ref = Path("src/assets/vaso_1.jpeg")
path_cur = Path("src/assets/vaso_2.jpeg")

try:
    ref_orig = load_image(path_ref)
    cur_orig = load_image(path_cur)

    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base_dir = Path(f"runs/run_{run_id}")
    base_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n=> Iniciando Experimento Seguro. Diretorio central: {base_dir}")

    # Valores de granularidade validados pelo survey e artigo de hierarquia
    n_values = [50, 75, 100, 150, 200, 300, 400]

    fluxos = [
        {"id": "1_Boruvka_Puro", "desc": "Boruvka Puro + LightGlue"},
        {"id": "2_HM_Boruvka", "desc": "Histogram Matching + Boruvka + LightGlue"},
        {"id": "3_HM_BF_Boruvka", "desc": "Histogram Matching + Bilateral Filter + Boruvka + LightGlue"},
        {"id": "4_HM_BF_CE_Boruvka", "desc": "Histogram Matching + Bilateral Filter + Canny + Boruvka + LightGlue"}
    ]

    for fluxo in fluxos:
        fluxo_id = fluxo["id"]
        fluxo_desc = fluxo["desc"]
        print(f"\n=== Executando Fluxo: {fluxo_desc} ===")
        
        fluxo_dir = base_dir / fluxo_id
        fluxo_dir.mkdir(parents=True, exist_ok=True)
        csv_data = []
        
        for n in n_values:
            title = f"N={n}"
            
            if fluxo_id == "1_Boruvka_Puro":
                ref_proc = run_boruvka(ref_orig, edge_map=None, n_supix=n)
                cur_proc = run_boruvka(cur_orig, edge_map=None, n_supix=n)
                
            elif fluxo_id == "2_HM_Boruvka" or fluxo_id == "2_HM_Boruvka":
                cur_hm = match_histogram_tensor(cur_orig, ref_orig)
                ref_proc = run_boruvka(ref_orig, edge_map=None, n_supix=n)
                cur_proc = run_boruvka(cur_hm, edge_map=None, n_supix=n)
                
            elif fluxo_id == "3_HM_BF_Boruvka":
                cur_hm = match_histogram_tensor(cur_orig, ref_orig)
                ref_bf = apply_bilateral_filter(ref_orig)
                cur_bf = apply_bilateral_filter(cur_hm)
                ref_proc = run_boruvka(ref_bf, edge_map=None, n_supix=n)
                cur_proc = run_boruvka(cur_bf, edge_map=None, n_supix=n)
                
            elif fluxo_id == "4_HM_BF_CE_Boruvka":
                cur_hm = match_histogram_tensor(cur_orig, ref_orig)
                ref_bf = apply_bilateral_filter(ref_orig)
                cur_bf = apply_bilateral_filter(cur_hm)
                
                ref_edge = apply_canny_edge(ref_bf)
                cur_edge = apply_canny_edge(cur_bf)
                
                ref_proc = run_boruvka(ref_bf, edge_map=ref_edge, n_supix=n)
                cur_proc = run_boruvka(cur_bf, edge_map=cur_edge, n_supix=n)

            # Executa com restrição de foco central de 30% na largura e altura
            fig, num_matches, stop_layer = run_lightglue_pipeline_with_roi(
                ref_proc, cur_proc, f"{fluxo_id} - {title}", pct_w=0.3, pct_h=0.3
            )
            print(f" -> {title}: {num_matches} matches encontrados (Stop Layer: {stop_layer}).")
            
            img_filename = f"matches_n_{n}.png"
            fig.savefig(fluxo_dir / img_filename, bbox_inches="tight", dpi=150)
            plt.close(fig)
            
            csv_data.append({"n_superpixels": n, "matches_count": num_matches, "stop_layer": stop_layer})

        csv_filepath = fluxo_dir / "metrics.csv"
        with open(csv_filepath, mode="w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=["n_superpixels", "matches_count", "stop_layer"])
            writer.writeheader()
            writer.writerows(csv_data)
        print(f"Relatório CSV salvo em: {csv_filepath}")

    print("\n=> Experimento Concluído! Imagens reais salvas com as linhas de matches mapeadas de volta.")

except FileNotFoundError as e:
    print(f"Erro de arquivo: {e}")
except Exception as e:
    print(f"Erro crítico no pipeline: {e}")