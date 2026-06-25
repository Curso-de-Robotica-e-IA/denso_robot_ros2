import sys
import torch
import numpy as np
import cv2
from pathlib import Path
from dataclasses import dataclass
import matplotlib.pyplot as plt
from lightglue.utils import load_image, rbd
from lightglue import LightGlue, SuperPoint

# Garanta que o path do Boruvka esteja correto
sys.path.insert(0, "boruvka-superpixel/pybuild")
import boruvka_superpixel

# Reimportar funções do seu pipeline estruturado
from validation_pipeline import (
    match_histogram_tensor,
    apply_bilateral_filter,
    apply_canny_edge,
    run_boruvka,
    estimate_homography
)

# --- Configurações Locais ---
torch.set_grad_enabled(False)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(feature="superpoint", flash=True).eval().to(device)

@dataclass
class MetricasConfig:
    nome: str
    erros: list
    matches_totais: list
    inliers_ransac: list

# --- Gerador Aleatório de Ground Truth (Monte Carlo) ---
def gerar_transformacao_aleatoria(w, h):
    """Gera movimentos aleatórios realistas dentro dos limites operacionais do robô"""
    angulo = np.random.uniform(-20.0, 20.0)      # Rotação de até 20 graus
    escala = np.random.uniform(0.85, 1.15)       # Zoom/Afastamento de até 15%
    tx = np.random.uniform(-50.0, 50.0)          # Translação horizontal em pixels
    ty = np.random.uniform(-40.0, 40.0)          # Translação vertical em pixels
    
    cx, cy = w / 2.0, h / 2.0
    M_rot = cv2.getRotationMatrix2D((cx, cy), angulo, escala)
    H = np.eye(3, dtype=np.float32)
    H[0:2, 0:3] = M_rot
    H[0, 2] += tx
    H[1, 2] += ty
    return H

# --- Core do Pipeline para Testes ---
def processar_teste_matematico(img0_proc, img1_proc):
    """Executa apenas a extração e o casamento matemático rápido para o loop de testes"""
    feats0 = extractor.extract(img0_proc.to(device))
    feats1 = extractor.extract(img1_proc.to(device))
    matches0 = matcher({"image0": feats0, "image1": feats1, "filter_threshold": 0.1})
    
    feats0, feats1, matches0 = [rbd(x) for x in [feats0, feats1, matches0]]
    kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
    matches = matches0["matches"]
    
    m_kpts0 = kpts0[matches[..., 0]].cpu().numpy()
    m_kpts1 = kpts1[matches[..., 1]].cpu().numpy()
    
    H, mask, inliers = estimate_homography(m_kpts0, m_kpts1)
    return H, len(matches), inliers

# --- SCRIPT PRINCIPAL DA ROTINA ---
if __name__ == "__main__":
    path_ref = Path("src/assets/vaso_1.jpeg")
    if not path_ref.exists():
        print(f"Erro: Arquivo {path_ref} não encontrado.")
        sys.exit(1)
        
    ref_tensor = load_image(path_ref)
    img_ref_np = (ref_tensor.permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
    h_img, w_img, _ = img_ref_np.shape
    cantos_base = np.array([[0, 0], [w_img, 0], [w_img, h_img], [0, h_img]], dtype=np.float32).reshape(-1, 1, 2)

    # CORREÇÃO: Inicialização das configurações conforme os requisitos fornecidos
    configs = {
        1: MetricasConfig("1. Borůvka Puro", [], [], []),
        2: MetricasConfig("2. HM + Borůvka", [], [], []),
        3: MetricasConfig("3. HM + BF + Borůvka", [], [], []),
        4: MetricasConfig("4. HM + BF + CE + Borůvka (Completo)", [], [], [])
    }

    NUM_SIMULACOES = 30  # Sinta-se livre para subir para 50 se necessário
    print(f"Iniciando rotina estatística com {NUM_SIMULACOES} simulações de homografia aleatórias...")

    for i in range(NUM_SIMULACOES):
        print(f" -> Processando Simulação {i+1}/{NUM_SIMULACOES}...")
        
        # 1. Gera o cenário distorcido aleatório (Monte Carlo)
        H_gt = gerar_transformacao_aleatoria(w_img, h_img)
        img_cur_np = cv2.warpPerspective(img_ref_np, H_gt, (w_img, h_img))
        cur_tensor = torch.from_numpy(img_cur_np).permute(2, 0, 1).float() / 255.0
        
        cantos_reais = cv2.perspectiveTransform(cantos_base, H_gt)

        # -----------------------------------------------------------------
        # GERAÇÃO DAS ENTRADAS DINÂMICAS PARA CADA CONFIGURAÇÃO INCREMENTAL
        # -----------------------------------------------------------------
        
        # Config 1: Borůvka Puro (Roda direto nos tensores originais, sem Canny guiando)
        ref_puro = run_boruvka(ref_tensor, edge_map=None, n_supix=100)
        cur_puro = run_boruvka(cur_tensor, edge_map=None, n_supix=100)
        
        # Config 2: HM + Borůvka (Aplica Histogram Matching antes do superpixel)
        cur_hm = match_histogram_tensor(cur_tensor, ref_tensor)
        ref_hm_bor = run_boruvka(ref_tensor, edge_map=None, n_supix=100)
        cur_hm_bor = run_boruvka(cur_hm, edge_map=None, n_supix=100)
        
        # Config 3: HM + BF + Borůvka (Adiciona Filtro Bilateral antes do superpixel)
        ref_bf = apply_bilateral_filter(ref_tensor)
        cur_bf = apply_bilateral_filter(cur_hm)
        ref_bf_bor = run_boruvka(ref_bf, edge_map=None, n_supix=100)
        cur_bf_bor = run_boruvka(cur_bf, edge_map=None, n_supix=100)
        
        # Config 4: HM + BF + CE + Borůvka (Completo - Adiciona mapa de bordas Canny guiando o grafo)
        ref_edge = apply_canny_edge(ref_tensor)
        cur_edge = apply_canny_edge(cur_hm)
        ref_completo = run_boruvka(ref_bf, edge_map=ref_edge, n_supix=100)
        cur_completo = run_boruvka(cur_bf, edge_map=cur_edge, n_supix=100)

        # 2. Executa o casamento estatístico para as 4 variantes reconstruídas
        for ID, cfg in configs.items():
            if ID == 1:
                H_est, m_tot, inl = processar_teste_matematico(ref_puro, cur_puro)
            elif ID == 2:
                H_est, m_tot, inl = processar_teste_matematico(ref_hm_bor, cur_hm_bor)
            elif ID == 3:
                H_est, m_tot, inl = processar_teste_matematico(ref_bf_bor, cur_bf_bor)
            elif ID == 4:
                H_est, m_tot, inl = processar_teste_matematico(ref_completo, cur_completo)
            
            # Cálculo de erro geométrico residual
            if H_est is not None:
                cantos_est = cv2.perspectiveTransform(cantos_base, H_est)
                erro = np.mean(np.linalg.norm(cantos_reais - cantos_est, axis=2))
            else:
                erro = 999.0  # Penalidade matemática se a homografia divergir
                
            cfg.erros.append(erro)
            cfg.matches_totais.append(m_tot)
            cfg.inliers_ransac.append(inl)

    # --- EXIBIÇÃO DO RELATÓRIO CIENTÍFICO ATUALIZADO ---
    print("\n" + "="*70)
    print("      RELATÓRIO DE ROBUSTEZ GEOMÉTRICA (ESTUDO DE ABLAÇÃO)   ")
    print("="*70)
    print(f"Resultados médios obtidos após {NUM_SIMULACOES} testes aleatórios de Monte Carlo:")
    print("-"*70)
    
    for ID, cfg in configs.items():
        media_erro = np.mean(cfg.erros)
        media_matches = np.mean(cfg.matches_totais)
        media_inliers = np.mean(cfg.inliers_ransac)
        taxa_inliers = (media_inliers / media_matches * 100) if media_matches > 0 else 0
        
        print(f"{cfg.nome}")
        print(f"  -> Erro Médio de Alinhamento: {media_erro:.4f} pixels")
        print(f"  -> Densidade de Matches:     {media_matches:.1f} pontos")
        print(f"  -> Filtro RANSAC (Inliers):  {media_inliers:.1f} ({taxa_inliers:.1f}%)")
        print("-"*70)