# Future works

Что не удалось восстановить по [manipulator_manual.md](manipulator_manual.md) и что надо
узнать у разработчиков BOXes. Пакеты `move_to_pose_interface`, `manipulator_msgs` и `box_msgs`
в `workspace/src/boxes_interfaces/` написаны по описаниям из §9.4 мануала. Типы ниже в мануале
не описаны, поэтому в код пока не добавлены: здесь только предлагаемые описания.

## TODO: сверить все типы с оригиналами из `box_ws`

- [ ] **TODO: узнать** исходники `.msg` / `.srv` / `.action` пакетов `move_to_pose_interface`,
  `manipulator_msgs`, `box_msgs`, `tf_stuff` (или вывод `ros2 interface show` для каждого типа).

Программа, написанная под симулятор, будет работать с реальной системой, только если типы
совпадают в точности: имена пакетов и типов, имена, типы и **порядок** полей. В Jazzy по этому
описанию считается хэш типа. Порядок полей взят из мануала, но мануал мог его не сохранить.

## TODO: `tf_stuff` — сервисы TF-сервера (§6)

- [ ] **TODO: узнать** поля `tf_stuff/srv/CreateTF`, `SaveTF`, `SavePose`, `MoveTF`,
  `RemoveTF`. В мануале только имена сервисов и отсылка к
  `ros2 interface show tf_stuff/srv/CreateTF`.
- [ ] **TODO: узнать** формат `tfs.json` (где лежит родительский кадр, кватернион или RPY).
- [ ] **TODO: узнать**, `MoveTF` задаёт новую позу кадра или смещение относительно текущей.

Предлагаемые описания (черновик, все кадры относительно `boxes_prefix/base_link`):

```
# tf_stuff/srv/CreateTF — создать кадр (в памяти, без записи в tfs.json)
string name                   # имя кадра точки
string parent_frame           # boxes_prefix/base_link
geometry_msgs/Pose pose       # поза кадра в parent_frame
---
bool success
string message
```

```
# tf_stuff/srv/SaveTF — записать кадр в tfs.json
string name                   # пусто = все кадры
---
bool success
string message
```

```
# tf_stuff/srv/SavePose — создать кадр из текущей позы ee_link и записать его
string name
---
bool success
string message
```

```
# tf_stuff/srv/MoveTF — сдвинуть кадр
string name
geometry_msgs/Pose pose       # новая поза в parent_frame
---
bool success
string message
```

```
# tf_stuff/srv/RemoveTF — удалить кадр (и из tfs.json)
string name
---
bool success
string message
```

## TODO: `manipulator_msgs/JointModeCommand` — jog по осям (§7)

- [ ] **TODO: узнать** типы полей. Из примера в мануале известны только имена:
  `{index_joint: 0, command: 0.1}`.
- [ ] **TODO: узнать**, с 0 или с 1 считается `index_joint` (мануал сам предлагает проверить
  на имитаторах).

Предлагаемое описание:

```
# manipulator_msgs/msg/JointModeCommand — команда jog одной оси
int32 index_joint     # индекс оси
float64 command       # позиция, рад (position-режим) или скорость, рад/с (velocity-режим)
```

## TODO: константы `manipulator_msgs/RobotMode`

- [ ] **TODO: узнать** константы в `RobotMode.msg`. Мануал (§9.4) пишет, что в `mode` сейчас
  передаётся режим приводов CiA-402 (1, 3, 6, 7, 10), а не константы сообщения, но самих
  констант не приводит. В нашем `RobotMode.msg` констант нет, только `int8 mode`.

## TODO: интерфейс сменщика инструмента

- [ ] **TODO: узнать** интерфейс сменщика (будет отдельным). Пока в симуляции — сервис
  `std_srvs/SetBool`, зажать / разжать кулачок.
