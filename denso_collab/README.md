# DENSO cellphone collaboration

Dois comandos separados: **bringup abre a célula**; depois
**`screen_approach.launch.py` executa o teste**. Encerrar o teste não encerra o bringup.

- [Testes reais: D405, Hall e app](REAL_TESTS.md)
- [Testes em simulação: Gazebo e celular virtual](SIM_TESTS.md)

Ambos os guias incluem RGB, CAPTCHA e celular horizontal. Os parâmetros do
teste ficam no launch do toque (`target`, `phone_orientation`, `test_runs`,
`use_random_holder_pose`, `touch_enabled`, `approach_plan_only`).

Distâncias, velocidades, retração, dwell e limites:
[cellphone_collab.yaml](config/cellphone_collab.yaml). Âncoras aleatórias:
[right_holder_poses.yaml](config/right_holder_poses.yaml). Configuração de visão:
[cellphone_holder.yaml](../denso_vision/config/cellphone_holder.yaml).
Reinicie o teste após alterar YAML; alterações em C++ exigem rebuild.

A calibração do TCP esquerdo mantém `X=+2 mm`, `Y=+1 mm` locais. Os argumentos
`left_hall_touch_calib_offset_x_mm` e `left_hall_touch_calib_offset_y_mm` do
bringup permitem ajustar esses valores; mudar a geometria exige reiniciar o bringup.
