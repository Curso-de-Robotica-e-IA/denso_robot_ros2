from skimage.segmentation import slic
from skimage.color import rgb2lab
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

def apply_slic_to_tensor(image_tensor, n_segments=200, compactness=10):
    """
    Aplica SLIC e reconstrói a imagem usando média por superpixel.

    image_tensor: (C, H, W) em [0,1]
    """

    # Tensor → numpy (H, W, C)
    img_np = image_tensor.permute(1, 2, 0).cpu().numpy()

    # SLIC funciona melhor em LAB
    img_lab = rgb2lab(img_np)

    # Segmentação
    segments = slic(
        img_lab,
        n_segments=n_segments,
        compactness=compactness,
        start_label=0
    )

    # Reconstrução da imagem (média por superpixel)
    output = np.zeros_like(img_np)

    for seg_val in np.unique(segments):
        mask = segments == seg_val
        output[mask] = img_np[mask].mean(axis=0)

    # numpy → tensor
    output_tensor = torch.from_numpy(output).permute(2, 0, 1).float()

    return output_tensor


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

path_ref = Path("src/assets/ref_img.jpeg")
path_cur = Path("src/assets/current_img.jpeg")
# #
# path_ref = Path("src/assets/vaso_1.jpeg")
# path_cur = Path("src/assets/vaso_2.jpeg")
try:
    ref_tensor_orig = load_image(path_ref)
    cur_tensor_orig = load_image(path_cur)

    scenarios = [
        ("Original (Sem SLIC)", None),
        # ("SLIC (n=200, c=10)", (200, 10)),
        # ("SLIC (n=400, c=15)", (300, 15)),
        ("SLIC (n=600, c=20)", (200, 20))
    ]

    # 3. Itera sobre os cenários
    for title, params in scenarios:

        # Se params for None, usa a imagem original.
        # Se tiver params, aplica o filtro.
        if params is None:
            ref_input = ref_tensor_orig
            cur_input = cur_tensor_orig
        else:
            n_segments, compactness = params
            ref_input = apply_slic_to_tensor(ref_tensor_orig, n_segments=n_segments, compactness=compactness)
            cur_input = apply_slic_to_tensor(cur_tensor_orig, n_segments=n_segments, compactness=compactness)

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