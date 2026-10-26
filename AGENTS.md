# abb_pkg — руководство для агентов

Как устроен и работает стек **MoveIt 2 + ros2_control** (симуляция рабочей ячейки с фрезой и
лазером на ABB IRB 1600-10/1.45): что где лежит, как проходит запуск, как команда с панели
превращается в движение робота, какие инварианты нельзя ломать и как проверять изменения.
Инструкции пользователя по запуску — в [README.md](README.md). EGM-стек (`abb_pkg`,
`abb_libegm`, `vel_control`, …) с MoveIt-стеком не связан; для `vel_control` есть свой
`workspace/src/vel_control/AGENTS.md`.

ROS 2 Humble, Ubuntu 22.04 (образы `ros:humble-ros-base-jammy`), RMW CycloneDDS.

## 1. Карта пакетов

```
workspace/src/
├── abb/abb_irb1600_support/        # ros-industrial: меши и xacro-макрос IRB 1600 (не трогать)
├── abb_irb1600_description/
│   ├── urdf/irb1600_10_145.urdf.xacro    # world → base_link + макрос робота + ros2_control
│   ├── urdf/irb1600.ros2_control.xacro   # <ros2_control>: плагин, joint_1..6, position/position
│   ├── config/initial_positions.yaml     # стартовая поза mock-железа (= home в SRDF)
│   └── launch/view_robot.launch.xml      # только модель + joint_state_publisher_gui
├── abb_irb1600_bringup/
│   ├── config/ros2_controllers.yaml      # 250 Гц, JSB + JTC, допуски траектории
│   └── launch/hardware.launch.xml        # RSP, ros2_control_node, два spawner'а
├── abb_irb1600_moveit_config/
│   ├── config/irb1600_10_145.srdf        # группа manipulator (base_link → tool0), home, zero
│   ├── config/kinematics.yaml            # KDL, timeout 0.05 с
│   ├── config/planning_pipelines.yaml    # OMPL, RRTConnect по умолчанию
│   ├── config/joint_limits.yaml          # скорости ABB, консервативные ускорения, scaling 0.1
│   ├── config/moveit_controllers.yaml    # MoveIt → joint_trajectory_controller/follow_joint_trajectory
│   ├── config/move_group.yaml            # публикация сцены и robot_description(_semantic)
│   ├── config/moveit.rviz                # раскладка RViz (MotionPlanning + WorkCell)
│   └── launch/move_group.launch.xml      # move_group + rviz2 (arg rviz)
├── abb_trajectory_generator/             # Python, ament_python
│   ├── abb_trajectory_generator/moveit_client.py           # тонкий клиент ROS-интерфейсов move_group
│   ├── abb_trajectory_generator/cell_layout.py             # геометрия ячейки, инструменты, ik_seed
│   ├── abb_trajectory_generator/work_cell_node.py          # нода work_cell
│   ├── abb_trajectory_generator/trajectory_generator_node.py  # нода trajectory_generator (круг/линия)
│   ├── config/params.yaml                # скорости и геометрия фигур
│   └── launch/trajectory_generator.launch.xml
└── abb_work_cell_panel/                  # C++, плагин rviz_common::Panel (Qt5)
    ├── src/work_cell_panel.cpp           # кнопки → /work_cell/<service>, подписка на статус
    └── plugin_description.xml
docker/moveit_stack/
├── docker-compose.yml                    # три сервиса, host network, CycloneDDS
├── hardware.Dockerfile                   # abb, description, bringup
├── moveit.Dockerfile                     # abb, description, moveit_config, work_cell_panel
└── trajectory.Dockerfile                 # abb_trajectory_generator
```

Граф зависимостей сборки: `abb_irb1600_bringup` и `abb_irb1600_moveit_config` зависят от
`abb_irb1600_description` (а он от `abb_irb1600_support`); `abb_irb1600_moveit_config` зависит
от `abb_work_cell_panel` (exec_depend), поэтому `--packages-up-to abb_irb1600_moveit_config`
собирает и панель. `abb_trajectory_generator` от остальных пакетов workspace не зависит: он
использует только сообщения MoveIt (`moveit_msgs`), а не его библиотеки.

## 2. Рантайм: узлы и интерфейсы

```
 trajectory                         moveit                                hardware
 ──────────                         ──────                                ────────
 work_cell ──── /move_action ─────▶ move_group ── follow_joint_trajectory ─▶ joint_trajectory_controller
   │  ▲     ─── /compute_ik ──────▶   │  ▲                                       │ position cmd
   │  │     ─── /compute_cartesian_path ▶ │                                     ▼
   │  │     ─── /execute_trajectory ─▶   │  │                          mock_components/GenericSystem
   │  │     ─── /apply_planning_scene ▶  │  │                                  │ position state
   │  │     ─── /get_planning_scene ──▶  │  │                                  ▼
   │  │     ─── /trajectory_execution_event ("stop")            joint_state_broadcaster
   │  └──── /robot_description_semantic (latched) ──┘  │                       │ /joint_states
   │                                                   └───────◀───────────────┤
   ├── /work_cell/status  (latched)  ──▶ RViz WorkCell panel                   ▼
   ├── /work_cell/markers (10 Гц)    ──▶ RViz MarkerArray        robot_state_publisher → /tf, /robot_description
   └── /work_cell/<service> ◀── RViz WorkCell panel / ros2 service call
 trajectory_generator ◀── /figure_type (std_msgs/String)
   └── /move_action, /compute_ik, /compute_cartesian_path, /execute_trajectory → move_group
```

| Узел | Контейнер | Подписки / клиенты | Публикует / сервер |
|------|-----------|--------------------|--------------------|
| `robot_state_publisher` | hardware | `/joint_states` | `/tf` (50 Гц), `/robot_description` (latched) |
| `controller_manager` (`ros2_control_node`) | hardware | `/robot_description` | контроллеры, 250 Гц |
| `joint_state_broadcaster` | hardware | — | `/joint_states` |
| `joint_trajectory_controller` | hardware | — | action `/joint_trajectory_controller/follow_joint_trajectory` |
| `move_group` | moveit | `/joint_states`, `/tf`, action JTC | `/move_action`, `/execute_trajectory`, `/compute_cartesian_path`, `/compute_ik`, `/apply_planning_scene`, `/get_planning_scene`, `/monitored_planning_scene`, `/robot_description_semantic` |
| `rviz2` | moveit | `/tf`, `/robot_description`, `/work_cell/markers`, `/work_cell/status`, `/monitored_planning_scene` | вызывает `/work_cell/*` |
| `work_cell` | trajectory | `/robot_description_semantic`, `/tf` | `/work_cell/{take_mill,take_laser,release_tool,start_work,stop_work,go_home}` (`std_srvs/Trigger`), `/work_cell/status`, `/work_cell/markers` |
| `trajectory_generator` | trajectory | `/figure_type` | — |

Система координат: `world` совпадает с `base_link` (фиксированный сустав в URDF). `work_cell`
планирует в `world`, `trajectory_generator` — в `base_link` (параметр `frame_id`). Единицы — метры
и радианы; градусы встречаются только в человекочитаемых параметрах (`orientation_rpy_deg`,
`TOOL_DOWN_RPY_DEG`) и переводятся в радианы в коде.

## 3. Пайплайн запуска

`docker compose up` стартует `hardware` → `moveit` → `trajectory`. `depends_on` задаёт только
порядок старта и не ждёт готовности; готовность узлы ждут сами.

1. **hardware** (`hardware.launch.xml`):
   - `xacro irb1600_10_145.urdf.xacro hardware_plugin:=$HARDWARE_PLUGIN` → `robot_description`
     для `robot_state_publisher`;
   - `ros2_control_node` берёт описание из `/robot_description` (remap `~/robot_description`),
     загружает плагин железа. У `mock_components/GenericSystem` начальные значения state
     interface — из `initial_positions.yaml` (`<param name="initial_value">` в xacro);
   - два `spawner` загружают и активируют `joint_state_broadcaster` и
     `joint_trajectory_controller` и завершаются (`process has finished cleanly` — это норма).
2. **moveit** (`move_group.launch.xml`):
   - URDF собирается тем же xacro **без** аргумента `hardware_plugin` (MoveIt блок
     `<ros2_control>` не читает), SRDF читается `cat`;
   - `move_group` загружает kinematics / joint_limits / planning_pipelines / moveit_controllers /
     move_group.yaml, подключается к JTC через `MoveItSimpleControllerManager` и пишет
     `You can start planning now!`;
   - RViz (если `rviz:=true`) загружает `moveit.rviz` и плагин `abb_work_cell_panel/WorkCellPanel`.
3. **trajectory** (`trajectory_generator.launch.xml`, обе ноды читают `config/params.yaml`):
   - `trajectory_generator` сразу пишет `Ready, ...`; к `move_group` подключается только при
     запуске фигуры. Если задан `autostart_figure`, фигура запускается из конструктора;
   - `work_cell` в конструкторе захватывает `busy` и в отдельном потоке выполняет
     `setup_scene`: ждёт все шесть интерфейсов `move_group` (до 30 с на каждый,
     `SERVER_TIMEOUT_S`), удаляет из сцены свои объекты от прошлого запуска (сначала
     открепляет инструмент от `tool0`, потом удаляет из мира), добавляет столы, заготовки,
     держатели и инструменты и пишет `Work cell added to the planning scene`. До этого все
     сервисы отвечают `занят: ожидание move_group`;
   - позу `home` `work_cell` берёт не из параметров, а парсит из latched-топика
     `/robot_description_semantic` (`group_state name="home"` для `group_name`).

После старта узлы молчат, пока не придёт команда.

## 4. Слой движения: `MoveItClient` (`moveit_client.py`)

Все вызовы **блокирующие**: отправляют запрос и опрашивают future (`wait_future`, шаг 10 мс).
Поэтому их можно вызывать только из рабочего потока, пока executor ноды крутится в основном
потоке (`rclpy.spin`) и доставляет ответы. Вызов из колбэка сервиса или таймера приведёт к
дедлоку.

| Метод | Что делает | Интерфейсы |
|-------|-----------|------------|
| `move_to_pose(pose, v, a, seed)` | IK для позы `tool0` (с коллизиями), потом свободное движение в найденные суставы | `/compute_ik` → `/move_action` |
| `move_to_joints(joints, v, a)` | Свободное (OMPL) движение к суставам: `JointConstraint` ±0.001 рад, 5 попыток, 5 с на планирование, `plan_only=False` | `/move_action` |
| `solve_ik(pose, seed)` | IK, начальное приближение — текущее состояние, суставы из `seed` подменяются | `/compute_ik` (timeout 0.2 с) |
| `follow_path(waypoints, v, a, avoid_collisions, max_step, min_fraction)` | Прямолинейная интерполяция с шагом 5 мм, отказ при `fraction < 0.99`, затем исполнение | `/compute_cartesian_path` → `/execute_trajectory` |
| `apply_scene(scene)` | Diff сцены планирования | `/apply_planning_scene` |
| `scene_object_ids()` | id объектов мира и прикреплённых объектов | `/get_planning_scene` |
| `stop()` | `"stop"` в `/trajectory_execution_event` и отмена текущей цели action | — |

Почему так сделано:

- **IK с seed вместо pose goal.** Если отдать OMPL цель-позу, он выберет любое решение IK
  (например, запястье, перевёрнутое на 180°). `cell_layout.ik_seed(x, y)` даёт конфигурацию
  home (локоть вверх, запястье не перевёрнуто) с `joint_1`, повёрнутым на цель, и `joint_6`,
  сохраняющим рыскание инструмента. Поэтому каждая цель достигается в одной и той же
  конфигурации, и `joint_6` не накручивается.
- **`revolute_jump_threshold = 0.3`** в декартовом пути отсекает перевороты суставов между
  соседними точками (5 мм шага не могут дать скачок больше 0.3 рад).
- **Остановка.** В Humble одна отмена `/execute_trajectory` робота не останавливает, нужен
  `"stop"` в `/trajectory_execution_event` (так делает `MoveGroupInterface::stop`).
- `start_state.is_diff = True` и пустые diff'ы сцены означают «текущее состояние и текущая
  сцена», включая прикреплённый инструмент.

## 5. Пайплайн `work_cell` (`work_cell_node.py`)

### 5.1 Модель выполнения

```
сервис (executor) ──▶ on_* ──▶ проверка состояния ──▶ submit(job)
                                  │ отказ: (False, причина)   │ busy.acquire(blocking=False)
                                  ▼                           ├─ занят → (False, 'занят: <state>')
                             ответ сразу                      └─ Thread(run(job)) → (True, 'принято')

run(job): job() → set_state('готов' | 'ошибка: …') → finally: working=False, process_on=False, busy.release()
```

- Одна работа за раз (`busy` — `threading.Lock`), сервисы отвечают сразу, работа идёт в
  daemon-потоке.
- Исключения в работе ловятся в `run`: нода остаётся живой, блокировка освобождается.
- Состояние — строка `self.state`, публикуется в `/work_cell/status` (latched) как
  `Инструмент: <фреза|лазер|нет>\nСостояние: <state>`. Значения: `ожидание move_group`,
  `беру: …`, `снимаю: …`, `работа: фрезеровка`, `работа: лазерная резка`, `еду домой`,
  `останавливаю`, `готов`, `ошибка: …`.
- Таймеры в executor: `on_sample` 50 Гц (след обработки), `on_publish_markers` 10 Гц.

### 5.2 Примитивы движения

- `move(pose)` — свободное движение без столкновений к позе `tool0` «инструментом вниз» через
  `move_to_pose` с `ik_seed`, скорость `velocity_scaling`.
- `path(waypoints, avoid_collisions, velocity_scaling)` — прямая через `follow_path`.
- Оба проверяют `stop_event`, если идёт работа, и возвращают `остановлено`.
- `avoid_collisions=False` используется для всех контактных движений: последние сантиметры
  захвата касаются инструмента и держателя, а при обработке инструмент касается заготовки.
  **На этих участках MoveIt не проверяет столкновения**, ошибку в геометрии он не поймает.

### 5.3 Взять / снять инструмент

```
take_tool(tool):
  если держим другой → release_tool()
  move(над гнездом: grasp_z + APPROACH_HEIGHT)          # без столкновений, в конфигурации ik_seed
  path([grasp], avoid_collisions=False)                 # прямо вниз до инструмента
  apply_scene(attach)                                   # объект tool_<name> прикрепляется к tool0
  held = name
  path([над гнездом], avoid_collisions=False)           # прямо вверх уже с инструментом

release_tool():
  move(над гнездом) → path([grasp]) → apply_scene(detach) → held=None → path([над гнездом])
```

- `grasp_z = TABLE_HEIGHT + HOLDER_CLEARANCE + tip`: высота `tool0`, когда инструмент стоит в
  держателе (ось +z `tool0` смотрит вниз, инструмент висит вдоль неё).
- При прикреплении `touch_links = [tool0, flange, link_6, link_5]`, MoveIt берёт геометрию
  объекта из мира. При откреплении (`REMOVE` у `AttachedCollisionObject`) объект возвращается
  в мир там, где стоит.

### 5.4 Работа (`start_work`)

```
work():
  stop_event.clear(); working = True; safe_z = None; traces[tool] = []
  program = milling | laser_cutting
  ok = program(tool)
  если не остановлено: ошибка → вернуть её; иначе go_home()
  если остановлено: state='останавливаю', ждать SETTLE_TIME_S (0.3 с), retract()
```

`process_contours(contours, z)` — общая программа обработки:

1. `safe_z = z + APPROACH_HEIGHT` (0.12 м), свободное `move` над первой точкой.
2. Для каждого контура подвод (`avoid_collisions=False`): у первого — прямо вниз; у следующих —
   подъём на `LEAD_IN_HEIGHT` (0.03 м) над концом предыдущего, перелёт и опускание.
3. Рез с `process_on = True` (след и луч) и скоростью `process_velocity_scaling`:
   - `sharp` контур — отдельный `path` на каждый отрезок, робот останавливается в вершинах,
     углы остаются углами;
   - гладкий контур (круг) — один `path` через все точки.
4. После последнего контура — подъём на `safe_z`.

Программы:

- **Фрезеровка** (`milling`): `z = MILL_SURFACE_Z + tip` (конец фрезы на верхней грани
  заготовки), растр «змейкой» по `MILL_AREA` с шагом `MILL_STEP` (проходы вдоль y), один
  sharp-контур.
- **Лазер** (`laser_cutting`): `z = LASER_SURFACE_Z + LASER_STANDOFF + tip` (сопло на 1 см над
  листом), гладкий круг из 90 отрезков, затем sharp-звезда из 10 отрезков. Оба контура
  начинаются сверху (угол π/2).

**Остановка** (`stop_work`, только если `working`): `stop_event.set()` + `moveit.stop()`.
Текущее движение прерывается, все следующие `move`/`path` возвращают `остановлено`. `work()`
ждёт 0.3 с, пока робот встанет, затем `retract()` поднимает `tool0` вертикально до `safe_z`,
если он ниже, и робот остаётся там (домой не едет). Смену инструмента и `go_home` вне работы
остановить нельзя.

**Домой** (`go_home`): `move_to_joints(home)` — суставное движение без столкновений к `home` из
SRDF.

### 5.5 Визуализация

- Сцену MoveIt RViz **не рисует** (`Scene Geometry` и `Scene Robot` выключены в `moveit.rviz`):
  `/monitored_planning_scene` приходит с частотой 4 Гц, и робот дёргался бы. Робот рисуется
  `RobotModel` из `/tf`, ячейка — маркерами `work_cell`.
- `/work_cell/markers`, пространства имён: `cell` (столы и заготовки), `holders`, `tool_mill`,
  `tool_laser`, `trace`, `beam`. Взятый инструмент и луч публикуются в кадре `tool0` с
  `frame_locked=True` и едут за роботом по TF.
- У маркеров **нет stamp**: RViz берёт последний TF. Stamp «сейчас» часто новее последнего TF
  `tool0`, и RViz прячет маркер до следующего сообщения (мерцание).
- След (`on_sample`, 50 Гц, только при `process_on`): кончик инструмента = TF
  `world → tool0` + `tip` вдоль оси z `tool0`, высота подменяется на поверхность
  (`WORK_SURFACE_Z[tool] + TRACE_LIFT`). Иначе след лазера висел бы в воздухе на высоте сопла.
  Точки не чаще 1 мм, не больше 10000 на инструмент, каждое включение процесса начинает новый
  штрих. След сбрасывается в начале каждой работы.
- Planned Path в RViz в режиме `REALTIME`: превью плана идёт в реальном темпе.

## 6. Пайплайн фигур (`trajectory_generator_node.py`)

```
/figure_type "circle"|"line"  (или autostart_figure)
  → start(): неизвестная фигура → ошибка в лог; занят → warn; иначе поток run()
  → waypoints = circle() | line()            # позы tool0 c orientation_rpy_deg, frame base_link
  → wait_for_servers()
  → move_to_pose(waypoints[0])               # IK без seed + /move_action
  → follow_path(waypoints[1:], max_step, min_fraction)   # с проверкой столкновений
  → лог '<figure>: done' или '<figure>: <ошибка>'
```

Фигура проверяется на столкновения со сценой, включая ячейку и взятый инструмент: с
инструментом в руке фигура, проходящая низко над столом, отклоняется
(`only N% of the path is feasible`). `trajectory_generator` и `work_cell` друг о друге не знают;
не запускайте фигуру во время работы ячейки.

## 7. Инварианты (меняются только вместе)

| Что | Где должно совпадать |
|-----|----------------------|
| Поза home `[0, 0, 0, 0, 90, -90]°` | `abb_irb1600_description/config/initial_positions.yaml` и `group_state home` в SRDF |
| Имена суставов `joint_1` … `joint_6` | URDF (макрос `abb_irb1600_support`), `irb1600.ros2_control.xacro`, `ros2_controllers.yaml`, `moveit_controllers.yaml`, SRDF |
| Интерфейсы `position` / `position` | `irb1600.ros2_control.xacro`, `ros2_controllers.yaml`; контракт для драйвера реального робота |
| Имя контроллера `joint_trajectory_controller` | `ros2_controllers.yaml`, `hardware.launch.xml` (spawner), `moveit_controllers.yaml` |
| Группа `manipulator`, `tool0` | SRDF, `params.yaml` (`group_name`, `ee_link`), `work_cell_node.EE_LINK` |
| Пространство имён `/work_cell/` и имена сервисов | `work_cell_node.py` (`~/<name>`, имя ноды `work_cell` из launch) и `kNamespace` + `addButton(..., "<name>")` в `work_cell_panel.cpp` |
| `publish_robot_description_semantic: true` | `move_group.yaml`: из этого топика `work_cell` берёт home |
| Ориентация «инструмент вниз» `[180, 0, -90]°` | `cell_layout.TOOL_DOWN_RPY_DEG` (и `ik_seed`), `params.yaml` `orientation_rpy_deg` |
| id объектов сцены | `work_cell_node.world_object_ids()`: по ним чистится сцена при перезапуске; новый объект нужно добавить туда |
| RMW CycloneDDS, `ROS_DOMAIN_ID` | все сервисы `docker-compose.yml` и локальные терминалы |
| Состав пакетов в образе | `COPY` в `*.Dockerfile`: новый пакет или зависимость надо добавить в нужный Dockerfile |

## 8. Цикл разработки и проверка

### Что пересобирать

Исходники **копируются** в образы при сборке (томов нет), без пересборки изменения не попадут
в контейнер.

| Изменили | Пересобрать |
|----------|-------------|
| `abb_trajectory_generator` (Python, `params.yaml`) | `docker compose up -d --build trajectory` |
| SRDF, конфиги MoveIt, `moveit.rviz`, `abb_work_cell_panel` | `docker compose up -d --build moveit` |
| `ros2_controllers.yaml`, `hardware.launch.xml` | `docker compose up -d --build hardware` |
| URDF / xacro / `initial_positions.yaml` | `hardware` и `moveit` (оба копируют `abb_irb1600_description`) |
| Dockerfile или apt-зависимости | соответствующий сервис с `--build` |

`trajectory` можно перезапускать отдельно, не трогая `move_group`: при старте `work_cell`
удаляет свои старые объекты из сцены, а взятый инструмент открепляет и убирает (он снова
появится в держателе). После перезапуска одного `hardware` mock-железо стартует из
`initial_positions.yaml`, но `work_cell` не знает об этом и считает, что держит прежний
инструмент. Проще перезапустить весь стек.

Без Docker: `colcon build --symlink-install --packages-up-to <pkg>` в `workspace`. С
`--symlink-install` правки Python и YAML подхватываются при перезапуске launch без
пересборки, а новые файлы и C++ требуют пересборки. Не запускайте локальные узлы
одновременно с контейнерами в том же `ROS_DOMAIN_ID`: получатся дубли нод.

### Как проверять

Автотестов у пакетов MoveIt-стека нет, проверка ручная на запущенном стеке:

```bash
cd docker/moveit_stack
docker compose up -d --build
docker compose logs -f                      # дождаться трёх строк готовности (см. README)
docker compose exec trajectory bash         # дальше внутри: . install/setup.sh
ros2 service call /work_cell/take_mill std_srvs/srv/Trigger
ros2 topic echo /work_cell/status           # ждать "Состояние: готов" или "ошибка: …"
ros2 service call /work_cell/start_work std_srvs/srv/Trigger
ros2 service call /work_cell/stop_work std_srvs/srv/Trigger
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: circle}"
```

Минимум перед коммитом:

1. Стек стартует, в логах три строки готовности и нет `ERROR` (кроме известных, см. ниже).
2. Для изменений в `work_cell`: взять и снять оба инструмента, полная работа каждым инструментом
   с возвратом домой, остановка посреди работы (инструмент поднимается над деталью),
   повторный старт после остановки, `go_home`.
3. Для изменений движения: в RViz нет переворотов запястья и рывков, след лежит на поверхности.
4. Python: `python3 -m py_compile` изменённых файлов; C++ панели собирается без предупреждений
   (`-Wall -Wextra -Wpedantic`).

Известные безвредные сообщения: `libGL error: ... nouveau` в RViz,
`Action server: /recognize_objects not available` (MotionPlanning), `SEVERE WARNING ...
InteractiveMarkerDisplay` (class_loader), `ResourceManager has already loaded an urdf file`.

## 9. Типовые изменения

**Новый инструмент.**
1. `cell_layout.py`: `Tool(name, label, slot, parts)` со `slot` на инструментальном столе (не
   пересекаться с держателями соседей), добавить в `TOOLS` и `WORK_SURFACE_Z`.
2. `work_cell_node.py`: сервис `take_<name>` в списке `__init__`, программа в `work()` (сейчас
   выбор `mill` / иначе лазер), стиль следа в `markers()`; если нужен луч или другой визуальный
   эффект — по образцу `beam`.
3. `work_cell_panel.cpp`: кнопка `addButton(..., "take_<name>")`.
4. README: таблица кнопок.

**Новая программа обработки.** Сформировать список контуров `(points, sharp)` в плоскости и
высоту `z` для `tool0` (поверхность + `tip` [+ зазор]) и вызвать `process_contours`. Все точки
должны быть достижимы с ориентацией «инструмент вниз» в конфигурации `ik_seed`: проверьте
`fraction` в логах.

**Новая фигура генератора.** Метод, возвращающий список поз через `self.pose(x, y, z)`, ветка в
`start()` и `execute_figure()`, параметры в `params.yaml` (и `declare_parameter` с тем же
значением по умолчанию).

**Новая кнопка панели.** Сервис `~/<name>` в `work_cell` + `addButton(text, "<name>")` в
нужной `QGroupBox` в конструкторе `WorkCellPanel`. Ответ сервиса панель показывает строкой под
статусом.

**Геометрия ячейки.** Только `cell_layout.py`. После изменения проверить захват обоих
инструментов (контактные участки не проверяются на столкновения) и обе программы.

**Реальный робот.** Драйвер — плагин `hardware_interface::SystemInterface` с контрактом из
§7. Пакет добавить в `hardware.Dockerfile`, `<param>` драйвера — в `<hardware>` в
`irb1600.ros2_control.xacro`, запуск `HARDWARE_PLUGIN=<pkg>/<System> docker compose up`. Перед
первым запуском снизить скорости в `params.yaml` и проверить `update_rate`, допуски JTC и
ускорения в `joint_limits.yaml`. Остальные контейнеры не меняются.

## 10. Грабли

- **Блокирующие вызовы MoveIt в колбэке** (сервис, таймер, подписка) — дедлок
  однопоточного executor'а. Только через рабочий поток (`submit` / `threading.Thread`).
- **Цель-поза для OMPL** — перевороты запястья. Используйте `move_to_pose` с `seed`
  (`ik_seed`) или суставную цель.
- **Отмена action без `"stop"`** робота в Humble не останавливает.
- **Stamp у маркеров в `tool0`** — мерцание.
- **`avoid_collisions=False`** выключает проверку на всём декартовом участке, а не только в
  точке контакта; держите такие участки короткими и вертикальными.
- **home в двух местах** (`initial_positions.yaml` и SRDF). Если разойдутся, «Домой» будет
  уводить робот не туда, где он стартует.
- **`work_cell` ждёт `move_group` 30 с.** Если он не успел, состояние `ошибка: move_group
  недоступен` и ячейки в сцене нет — нужен `docker compose restart trajectory`.
- **Скорости.** `joint_limits.yaml` задаёт scaling по умолчанию 0.1, но `work_cell` и генератор
  передают свои (`params.yaml`: 0.2 для свободных движений, 0.05 для обработки, 0.1 для фигур).

## 11. Соглашения

- Комментарии, docstring'и, логи (`get_logger()`) — по-английски, объясняют «почему»; плотность
  как в окружающем коде.
- Тексты для оператора — по-русски: статус, ответы сервисов, надписи панели.
- Python — PEP 8, строки до ~100 символов; C++ — стиль ROS 2, C++17, Qt-виджеты трогать только
  из GUI-потока (`QMetaObject::invokeMethod(..., Qt::QueuedConnection)`).
- Единицы СИ (м, рад, с) везде, кроме явно помеченных `*_deg`.
- Коммиты — короткий английский заголовок в повелительном наклонении («Add …», «Fix …»).
- При изменении кнопок, сервисов, параметров запуска или конфигурации обновляйте README и этот
  файл.
