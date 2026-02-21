# vel_control — Agent guide

## Назначение пакета

ROS 2 пакет для управления роботом ABB по **скоростям** через протокол EGM (External Guidance Motion). Подписка на `cmd_vel` (Twist); команды передаются на контроллер по UDP в виде protobuf (EgmSensor).

- **Один источник команды:** только таймер `update()` вызывает `controller_->write()`; callback `cmd_vel` лишь обновляет кэш.
- Фидбек от робота **не используется** в логике управления: пакет принимается для протокола, данные не парсятся в ноде для позы/маркеров.
- В EGM всегда отправляются: поза (сейчас нули) + скорости (из `cmd_vel` или нули).

## Структура пакета

```
vel_control/
├── AGENTS.md
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params.yaml          # port, gains, cmd_vel_timeout_s, pose_*
├── include/vel_control/
│   ├── controller.hpp       # EGM: init, update(), write(), getInputs()
│   └── vel_control_node.hpp # Нода: cmd_vel, таймеры, fillPoseCommand, fillVelocityCommand
├── launch/
│   └── vel_control.launch.py
└── src/
    ├── controller.cpp       # Реализация Controller (abb_libegm)
    └── vel_control_node.cpp # Реализация ноды
```

## Ключевые файлы и потоки

| Файл | Назначение |
|------|------------|
| `vel_control_node.cpp` | Подписка на `cmd_vel` (очередь 1), таймер 4 ms → `update()`, формирование Output (поза нули + скорости), единственный вызов `write()`. |
| `controller.cpp` | Порт EGM, `io_service_` в отдельном потоке, `update()` = waitForMessage(0) + read(); `write()` передаёт Output в libegm. |
| `config/params.yaml` | `port`, `cmd_vel_lin_gain`, `cmd_vel_ang_gain`, `cmd_vel_timeout_s`, `pose_x/y/z/rx/ry/rz` (сейчас не используются в цикле). |

## Protobuf (EGM)

- **Отправляется:** `abb::egm::wrapper::Output` → внутри libegm конвертируется в EgmSensor (planned = pose, speedRef = velocity).
- **Поза:** всегда нули (0, 0, 0, 0, 0, 0) — мм и градусы Эйлера.
- **Скорости:** линейные мм/с, угловые град/с; при отсутствии актуального `cmd_vel` (таймаут `cmd_vel_timeout_s`) — нули.
- Заполнение: `fillPoseCommand()`, `fillVelocityCommand()` в ноде; в контроллер передаётся уже собранный `Output`.

## Многопоточность

- **Поток ROS:** executor (таймеры, подписка) — один поток.
- **Поток EGM:** `io_service_.run()` в `Controller` — приём/ответ UDP.
- Общие данные: `last_cmd_vel_`, `last_cmd_vel_time_` защищены `cmd_vel_mutex_`; `last_feedback_pose_` и `feedback_mutex_` есть в коде, но фидбек в `update()` не используется.

## Сборка и запуск

- Зависимости: `abb_libegm`, `rclcpp`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `tf2`.
- Сборка из корня workspace: `colcon build --packages-select vel_control`
- Запуск: `ros2 launch vel_control vel_control.launch.py`
- Параметры подставляются из `config/params.yaml`.

## Правила для агентов

1. **Форматирование:** табуляция 2, стиль кода ROS 2, C++17 по умолчанию.
2. **Команда роботу:** только из `update()` через один вызов `write()`; не вызывать `write()` из `cmdVelCallback()`.
3. **Единицы в EGM:** позиция — мм, углы — градусы, линейная скорость — мм/с, угловая — град/с.
4. **Очередь cmd_vel:** оставить размер 1, чтобы не накапливать старые команды.
5. При добавлении логики по фидбеку учитывать, что сейчас фидбек в ноде сознательно не используется (приём пакета без разбора для управления).
