# Testes reais — DENSO, D405, Hall e app

Em cada terminal ROS, com o workspace já compilado:

```bash
source /opt/ros/humble/setup.bash
source /root/denso_ws/install/setup.bash
```

## 1. Bringup — terminal A

Abre os robôs, MoveIt, RViz e Servo; não inicia o teste.
IPs padrão: esquerdo `192.168.160.228`, direito `192.168.160.227`.

```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=false rviz:=true use_servo:=true \
  left_basic_camera:=false left_hall_touch_camera:=true \
  right_cellphone_holder:=true right_virtual_phone:=false
```

Mantenha esse terminal aberto. Confirme ambos os controladores de trajetória
e o joint-state broadcaster `active` com `ros2 control list_controllers`.
Não desative a verificação de colisões do Servo para contornar uma falha no real.

## 2. Sensor e app

Em outro terminal, inicie o bridge do repositório `finger-sensor`
(exemplo quando montado em `/finger-sensor`):

```bash
/usr/bin/python3 /finger-sensor/touch_ros2.py --ros-args -p port:=/dev/ttyUSB0
```

Confira `ros2 topic echo /touch_detected`: publicação contínua, `false` em
repouso e `true` ao pressionar. Ajuste a porta USB se necessário.
No app, escolha RGB ou CAPTCHA e gere os alvos; mantenha os quatro tags visíveis.
CAPTCHA usa o checkbox de teste do Google, requer internet e começa com um alvo.
Não abra outro processo da D405 ou da serial durante o teste.

## 3. Teste — terminal B

Primeiro valide a pose atual sem movimento, com os alvos já na tela:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  sim:=false image_source:=realsense image_topic:=/left_basic_camera \
  target:=rgb use_random_holder_pose:=false \
  approach_plan_only:=true touch_enabled:=false
```

Depois, para tocar na pose atual:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  sim:=false image_source:=realsense image_topic:=/left_basic_camera \
  target:=rgb use_random_holder_pose:=false touch_enabled:=true test_runs:=1
```

Para CAPTCHA, troque por `target:=captcha` e mostre esse modo no app.
Se já houver um driver ROS da câmera, use `image_source:=topic` e o tópico dele.

## Celular horizontal / poses aleatórias

```bash
ros2 launch denso_collab screen_approach.launch.py \
  sim:=false image_source:=realsense image_topic:=/left_basic_camera \
  target:=rgb use_random_holder_pose:=true phone_orientation:=horizontal \
  touch_enabled:=true test_runs:=1
```

O estágio aleatório gira o holder +90° anti-horário e mantém a câmera na
orientação vertical em relação ao mundo. Use `phone_orientation:=vertical`
para o modo normal. Com `use_random_holder_pose:=false`, o holder precisa
estar previamente na orientação escolhida; esse modo não move o braço direito.
Cheque a pose horizontal hipotética sem executar com:

```bash
ros2 run denso_collab random_collab_pose --ros-args \
  -p sim:=false -p plan_only:=true -p phone_orientation:=horizontal
```

`test_runs:=N` executa 1–100 lotes; com pose aleatória ligada, há uma nova pose
por lote. Confira velocidades e limites em [cellphone_collab.yaml](config/cellphone_collab.yaml)
antes de mover. Falha de sensor, planejamento ou toque interrompe a sequência:
inspecione antes de reiniciar. Exporte os resultados pelo app.

A detecção CAPTCHA ainda precisa de validação com a câmera real.
