# Testes em simulação — Gazebo e celular virtual

No contêiner `denso_gpu`, em cada terminal, com o workspace já compilado:

```bash
source /opt/ros/humble/setup.bash
source /root/denso_ws/install/setup.bash
```

## 1. Bringup — terminal A

Abre os robôs, Gazebo, MoveIt, RViz e Servo; não inicia o teste.

```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=true rviz:=true use_servo:=true \
  left_basic_camera:=false left_hall_touch_camera:=true \
  right_cellphone_holder:=true right_virtual_phone:=true target:=rgb \
  left_servo_check_collisions:=false
```

Mantenha esse terminal aberto. Espere ambos os controladores de trajetória
e o joint-state broadcaster ficarem `active` em `ros2 control list_controllers`.
Somente na simulação, a checagem do Servo é desativada para permitir contato
intencional com a tela virtual; o planejamento MoveIt mantém checagem de colisões.

## 2. Sensor e tela

O celular virtual mostra uma imagem estática de RGB ou CAPTCHA.
`sim:=true touch_enabled:=true` no launch abaixo inicia automaticamente o
sensor geométrico, publicando `/touch_detected` (`std_msgs/Bool`).
Não é necessário iniciar o bridge ESP32 nem o app.

## 3. Teste — terminal B

```bash
ros2 launch denso_collab screen_approach.launch.py \
  sim:=true image_source:=topic image_topic:=/left_basic_camera \
  target:=rgb use_random_holder_pose:=true touch_enabled:=true test_runs:=1
```

Executa pose aleatória validada, detecção, aproximação, toque e retração.
Para usar a pose atual, troque por `use_random_holder_pose:=false`.
Para apenas planejar, use `approach_plan_only:=true touch_enabled:=false`.

## Celular horizontal / poses aleatórias

```bash
ros2 launch denso_collab screen_approach.launch.py \
  sim:=true image_source:=topic image_topic:=/left_basic_camera \
  target:=rgb use_random_holder_pose:=true phone_orientation:=horizontal \
  touch_enabled:=true test_runs:=1
```

O estágio aleatório gira o holder +90° anti-horário e mantém a câmera na
orientação vertical em relação ao mundo. Use `phone_orientation:=vertical`
para o modo normal. Com `use_random_holder_pose:=false`, o holder precisa
estar previamente na orientação escolhida; esse modo não move o braço direito.

## CAPTCHA

Reinicie o bringup do terminal A com `target:=captcha` para trocar a imagem
do celular. Depois execute no terminal B:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  sim:=true image_source:=topic image_topic:=/left_basic_camera \
  target:=captcha use_random_holder_pose:=true touch_enabled:=true test_runs:=1
```

Também aceita `phone_orientation:=horizontal`. O `target` do bringup escolhe
somente a imagem; o `target` do teste escolhe o detector. Ambos precisam coincidir.
O toque simulado não marca o checkbox, não resolve desafios de imagens e
não gera logs do app: confirma apenas contato geométrico.

`test_runs:=N` executa 1–100 lotes. Distâncias, velocidades, retração e limites
ficam em [cellphone_collab.yaml](config/cellphone_collab.yaml).
Falhas interrompem o teste; o bringup continua aberto para inspeção.
