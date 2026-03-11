# abb_lerobot

ROS 2 пакет для записи датасетов ABB + LeRobot: рекордер с канонической схемой (observation.state, числовой action), конвертер в LeRobot v3 и валидатор.

**Зависимости для конвертера:** `pip install pyarrow numpy` (opencv обычно уже есть через cv_bridge). Валидатор не требует дополнительных пакетов.

## Запись датасета

```bash
ros2 launch abb_lerobot dataset_recorder.launch.py
# или с кастомным каталогом:
ros2 run abb_lerobot dataset_recorder --ros-args -p dataset_root:=/path/to/dataset
```

Начать новый эпизод без перезапуска ноды:

```bash
ros2 topic pub --once /dataset_recorder/start_episode std_msgs/Empty "{}"
```

## Валидация

Проверить сырой датасет перед конвертацией:

```bash
ros2 run abb_lerobot validate_dataset dataset
ros2 run abb_lerobot validate_dataset dataset --min-episode-length 10 --check-timestamps
```

## Конвертация в LeRobot v3

Собрать из сырых эпизодов (episode_*/data.jsonl + frame_*.png) датасет в формате LeRobot v3 (parquet + mp4 + meta):

```bash
ros2 run abb_lerobot convert_to_lerobot dataset -o dataset_lerobot
ros2 run abb_lerobot convert_to_lerobot /path/to/raw -o /path/to/lerobot_v3 --camera-key front --fps 30
```

## Схема (schema version 1)

- **observation.state** — вектор длины 14: 6 joint positions, 7 ee_pose (x,y,z,qx,qy,qz,qw), 1 gripper (0.0/1.0).
- **action** — target_pose (7 float) + gripper (1 float, 0.0 или 1.0).

Подробности и топики — в [AGENTS.md](../../AGENTS.md) (раздел abb_lerobot).
