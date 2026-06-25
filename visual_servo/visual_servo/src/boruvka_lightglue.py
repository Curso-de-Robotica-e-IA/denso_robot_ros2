import sys
from pathlib import Path
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt
from skimage.exposure import match_histograms
from datetime import datetime  # <-- Adicionado para gerir a data/hora

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

# Instancia os modelos
extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(feature="superpoint").eval().to(device)


def match_histogram_tensor(source_tensor, template_tensor):
    """
    Força o source_tensor (imagem atual) a ter o mesmo histograma 
    do template_tensor (imagem de referência).
    Ambos os tensores devem estar no formato (C, H, W) e no intervalo [0, 1].
    """
    # 1. Converte Tensors para matrizes NumPy no formato (H, W, C)
    src_np = source_tensor.permute(1, 2, 0).cpu().numpy()
    tmpl_np = template_tensor.permute(1, 2, 0).cpu().numpy()

    # 2. Aplica a especificação de histograma multicanal (RGB)
    matched_np = match_histograms(src_np, tmpl_np, channel_axis=2)

    # Garantir que os valores fiquem estritamente cravados entre [0.0, 1.0]
    matched_np = np.clip(matched_np, 0.0, 1.0)

    # 3. Converte de volta para Tensor (C, H, W)
    matched_tensor = torch.from_numpy(matched_np).permute(2, 0, 1).float()

    return matched_tensor


def apply_boruvka_to_tensor(image_tensor, n_supix=200):
    # Tensor → numpy
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255.0)
    img_np = np.ascontiguousarray(img_np.astype(np.uint8))

    # =========================================================================
    # PULO DO GATO: Filtro Bilateral preserva as bordas fortes, mas elimina 
    # as microfrequências que fazem o Borůvka divergir.
    img_filtered = cv2.bilateralFilter(img_np, d=9, sigmaColor=75, sigmaSpace=75)
    # =========================================================================

    img_edge = np.zeros((img_filtered.shape[:2]), dtype=np.uint8)
    img_edge = cv2.Canny(img_np, 50, 150) 
    
    bosupix = boruvka_superpixel.BoruvkaSuperpixel()
    bosupix.build_2d(img_np, img_edge) # O algoritmo agora respeita o Canny
    # bosupix = boruvka_superpixel.BoruvkaSuperpixel()
    
    # # Constrói o grafo na imagem filtrada
    # bosupix.build_2d(img_filtered, img_edge)
    
    # Calcula a média usando a imagem original (ou a filtrada, teste ambos)
    out = bosupix.average(n_supix, 3, img_np)

    # numpy → tensor
    output_tensor = torch.from_numpy(out).permute(2, 0, 1).float() / 255.0
    return output_tensor


def apply_meanshift_to_tensor(image_tensor, sp=20, sr=40):
    """
    Recebe um Tensor (C, H, W) [0,1], converte para OpenCV,
    aplica Mean Shift e retorna Tensor novamente.
    """
    img_np = (img_tensor.permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
    filtered_np = cv2.pyrMeanShiftFiltering(img_np, sp, sr)
    filtered_tensor = torch.from_numpy(filtered_np).permute(2, 0, 1).float() / 255.0
    return filtered_tensor


def run_pipeline(title, img0_tensor, img1_tensor):
    """
    Roda extração, matching e plotagem para um par de imagens.
    """
    print(f"--- Processando: {title} ---")

    img0 = img0_tensor.to(device)
    img1 = img1_tensor.to(device)

    # Extração
    feats0 = extractor.extract(img0)
    feats1 = extractor.extract(img1)

    # Matching
    matches0 = matcher({"image0": feats0, "image1": feats1})

    # Remove batch dimension
    feats0, feats1, matches0 = [rbd(x) for x in [feats0, feats1, matches0]]

    # Filtra keypoints
    kpts0, kpts1, matches = feats0["keypoints"], feats1["keypoints"], matches0["matches"]
    m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]

    count = len(matches)
    print(f"Matches encontrados: {count}")

    # Plotagem
    axes = viz2d.plot_images([img0_tensor, img1_tensor])
    viz2d.plot_matches(m_kpts0, m_kpts1, color="lime", lw=0.2)
    viz2d.add_text(0, f'{title}\nMatches: {count}', fs=15)

    return plt.gcf()


# --- EXECUÇÃO ---

# path_ref = Path("src/assets/ref_img.jpeg")
# path_cur = Path("src/assets/current_img.jpeg")
# #
path_ref = Path("src/assets/vaso_1.jpeg")
path_cur = Path("src/assets/vaso_2.jpeg")

try:
    ref_tensor_orig = load_image(path_ref)
    cur_tensor_orig = load_image(path_cur)

    # =========================================================================
    # APLICA A ESPECIFICAÇÃO DE HISTOGRAMA AQUI
    print("Aplicando especificação de histograma para pareamento de cores...")
    cur_tensor_orig = match_histogram_tensor(source_tensor=cur_tensor_orig, 
                                             template_tensor=ref_tensor_orig)
    # =========================================================================

    # Criação do diretório de saída com a data e hora atual do run
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = Path(f"runs/run_{run_id}")
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Diretório de saída criado: {output_dir}")

    scenarios = [
        ("Original (Sem Superpixel)", None),
        ("Boruvka (n=10)", 2),
        ("Boruvka (n=25)", 25),
        ("Boruvka (n=30)", 30),
        ("Boruvka (n=50)", 50),
        ("Boruvka (n=70)", 70),
        ("Boruvka (n=100)", 100)
    ]

    # Itera sobre os cenários
    for title, n_supix in scenarios:

        if n_supix is None:
            ref_input = ref_tensor_orig
            cur_input = cur_tensor_orig
        else:
            ref_input = apply_boruvka_to_tensor(ref_tensor_orig, n_supix=n_supix)
            cur_input = apply_boruvka_to_tensor(cur_tensor_orig, n_supix=n_supix)

        # Roda o pipeline
        fig = run_pipeline(title, ref_input, cur_input)

        # Define título da janela para identificação
        fig.canvas.manager.set_window_title(f"Teste: {title}")

        # Limpa o título para criar um nome de arquivo válido (remove parênteses e espaços)
        filename = title.replace(" ", "_").replace("(", "").replace(")", "") + ".png"
        filepath = output_dir / filename
        
        # Guarda a imagem na pasta da run atual
        fig.savefig(filepath, bbox_inches="tight", dpi=300)
        print(f"Imagem guardada em: {filepath}")

    # Exibe todas as janelas ao final
    print("Gerando visualizações no ecrã...")
    plt.show()

except FileNotFoundError as e:
    print(f"Erro: Verifique se os arquivos existem. {e}")
except Exception as e:
    print(f"Erro inesperado: {e}")