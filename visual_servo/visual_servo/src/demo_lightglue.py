from pathlib import Path
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt

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


def apply_meanshift_to_tensor(image_tensor, sp=20, sr=40):
    """
    Recebe um Tensor (C, H, W) [0,1], converte para OpenCV,
    aplica Mean Shift e retorna Tensor novamente.
    """
    img_np = (image_tensor.permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
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

    scenarios = [
        ("Original (Sem Mean Shift)", None),
        ("MeanShift (sp=20, sr=40)", (20, 40)),
        ("MeanShift (sp=40, sr=60)", (40, 60)),
        ("MeanShift (sp=60, sr=80)", (60, 80))
    ]

    # 3. Itera sobre os cenários
    for title, params in scenarios:

        # Se params for None, usa a imagem original.
        # Se tiver params, aplica o filtro.
        if params is None:
            ref_input = ref_tensor_orig
            cur_input = cur_tensor_orig
        else:
            sp, sr = params
            ref_input = apply_meanshift_to_tensor(ref_tensor_orig, sp=sp, sr=sr)
            cur_input = apply_meanshift_to_tensor(cur_tensor_orig, sp=sp, sr=sr)

        # Roda o pipeline
        fig = run_pipeline(title, ref_input, cur_input)

        # Define título da janela (Window Title) para facilitar identificação na barra de tarefas
        fig.canvas.manager.set_window_title(f"Teste: {title}")

    # Exibe todas as janelas ao final
    print("Gerando visualizações...")
    plt.show()

except FileNotFoundError as e:
    print(f"Erro: Verifique se os arquivos existem. {e}")
except Exception as e:
    print(f"Erro inesperado: {e}")