# Teste real: DENSO, D405, Hall e celular

Use quatro terminais. Não execute outro `touch_ros2.py`, monitor serial ou
`realsense-viewer` enquanto este teste estiver ativo.

## 1. Docker

No host:

```bash
cd /home/pcgr/code/denso/denso_docker
docker start denso_container
docker exec -it denso_container bash
```

Se `denso_container` ainda não existir, crie-o uma vez com
`docker compose -f docker-compose.yaml -f docker-compose.gpu.yaml up -d`.
Use `--build` somente quando o Dockerfile ou as dependências da imagem forem
alterados.

Em cada terminal dentro do contêiner:

```bash
source /opt/ros/humble/setup.bash
source /root/denso_ws/install/setup.bash
```

## 2. ESP32 Hall — terminal A

```bash
/usr/bin/python3 /finger-sensor/touch_ros2.py \
  --ros-args -p port:=/dev/ttyUSB0
```

Em outro terminal, confirme que o sinal está fresco antes de mover o robô:

```bash
ros2 daemon stop
ros2 topic echo /touch_detected
```

O estado normal é `data: false`. Ao pressionar o sensor, deve aparecer
`data: true`. Se a ESP32 estiver em outra porta, troque `/dev/ttyUSB0` por
`/dev/ttyACM0`.

## 3. Robôs reais — terminal B

Os IPs padrão são esquerdo `192.168.160.228` e direito `192.168.160.227`.
O `left_calib_link` usa por padrão os offsets locais `X=-2 mm` e `Y=-1 mm`.
Para voltar à posição antiga, acrescente
`left_hall_touch_calib_offset_x_mm:=0.0
left_hall_touch_calib_offset_y_mm:=0.0` ao comando.

```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=false rviz:=true use_servo:=true \
  left_basic_camera:=false left_hall_touch_camera:=true \
  right_cellphone_holder:=true right_virtual_phone:=false \
  left_servo_check_collisions:=false
```

Espere RViz, `move_group` e os dois controladores ficarem ativos. Mantenha
este terminal aberto durante todos os lotes.

## 4. Aplicativo do celular

No host, instale a versão atual e abra o app:

```bash
cd /home/pcgr/code/denso/robot-touch-test-app
JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64 \
PATH=/usr/lib/jvm/java-21-openjdk-amd64/bin:$PATH \
./gradlew installDebug --no-daemon
```

No app, em **Settings**:

- `RGB trios per spawn`: comece com `1`.
- `Fixed target positions`: ligado.
- `Random target positions`: desligado.
- `Random target sizes`: desligado.
- Escolha um raio fixo e mantenha-o igual em todo o lote.

Os três círculos devem permanecer visíveis simultaneamente. O app não possui
timeout. Um toque fora do alvo registra um miss sem bloquear os próximos
toques. O lote termina automaticamente depois que todos os alvos distintos
forem tocados, mesmo que existam tentativas extras ou toques repetidos. Use
**Continue after miss** uma vez para encerrar antecipadamente; os alvos ainda
pendentes serão registrados como `no_touch`.

## 5. Validação sem movimento — terminal C

Com o celular já mostrando o trio RGB:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  image_source:=realsense image_topic:=/left_basic_camera \
  use_random_holder_pose:=false approach_plan_only:=true \
  touch_enabled:=false sim:=false
```

O resultado esperado contém três alvos detectados e termina com
`Standoff sequence complete`. Este comando não move os robôs.

## 6. Lote real — terminal C

Depois da validação, execute um lote com o mesmo trio visível:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  image_source:=realsense image_topic:=/left_basic_camera \
  use_random_holder_pose:=false approach_plan_only:=false \
  touch_enabled:=true sim:=false
```

O braço esquerdo toca os alvos em ordem espacial. O braço direito fica fixo.
Se o Hall não confirmar contato, a sequência para; inspecione o robô antes de
iniciar outro lote.

## Próximo lote

Mantenha os robôs fixos (`use_random_holder_pose:=false`) e altere apenas uma
variável por lote, por exemplo o raio no app ou as velocidades em
`config/cellphone_collab.yaml`. Exporte o CSV do app ao final de cada grupo de
lotes.
