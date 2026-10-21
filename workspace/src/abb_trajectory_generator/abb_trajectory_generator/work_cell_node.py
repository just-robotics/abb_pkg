"""Work cell: tool change (mill / laser) and work programs through MoveIt.

Services (std_srvs/Trigger), all return at once and run in the background:
  ~/take_mill, ~/take_laser, ~/release_tool, ~/start_work, ~/stop_work, ~/go_home
Status for the RViz panel: ~/status (std_msgs/String, latched).

Work runs the program of the held tool once and returns the robot home. Home is
the "home" group state of the SRDF, the pose the simulated robot starts in.

The cell is added to the planning scene as collision objects; a held tool is
attached to tool0, so MoveIt plans with it. Visuals go to ~/markers: the held
tool and the laser beam are drawn in the tool0 frame, so they follow the robot
via TF.
"""

import math
import threading
import time
import xml.etree.ElementTree as ElementTree

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from abb_trajectory_generator import cell_layout as cell
from abb_trajectory_generator.moveit_client import MoveItClient, make_pose, rpy_to_quaternion

WORLD = 'world'
EE_LINK = 'tool0'
HOME_STATE = 'home'
TOUCH_LINKS = ['tool0', 'flange', 'link_6', 'link_5']
TRACE_MAX_POINTS = 10000     # per tool, oldest strokes are dropped
TRACE_MIN_STEP = 0.001
TRACE_LIFT = 0.0005          # above the work surface, so the line is not hidden in it
LEAD_IN_HEIGHT = 0.03        # tool lift between contours
SETTLE_TIME_S = 0.3          # after a stop, before the retract is planned


def tool_id(tool):
    return f'tool_{tool.name}'


def color(rgba):
    return ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])


def part_primitive(part):
    height = part.z_to - part.z_from
    if part.shape == 'cylinder':
        return SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, part.size])
    return SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[part.size, part.size, height])


class WorkCell(Node):

    def __init__(self):
        super().__init__('work_cell')

        self.group_name = self.declare_parameter('group_name', 'manipulator').value
        self.velocity_scaling = self.declare_parameter('velocity_scaling', 0.2).value
        self.acceleration_scaling = self.declare_parameter('acceleration_scaling', 0.2).value
        self.process_velocity_scaling = self.declare_parameter(
            'process_velocity_scaling', 0.05).value

        self.moveit = MoveItClient(self, self.group_name, WORLD, EE_LINK)
        self.tool_down = rpy_to_quaternion(*(math.radians(a) for a in cell.TOOL_DOWN_RPY_DEG))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.home = None
        self.create_subscription(String, '/robot_description_semantic', self.on_srdf, latched)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)

        self.busy = threading.Lock()
        self.stop_event = threading.Event()
        self.held = None
        self.working = False
        self.process_on = False
        self.safe_z = None          # tool0 height clear of the part being processed
        self.state = 'ожидание move_group'
        # machining trace per tool: strokes (lists of tip points) with the process on
        self.traces = {name: [] for name in cell.TOOLS}
        self.stroke = None

        for name, callback in (('take_mill', lambda: self.on_take('mill')),
                               ('take_laser', lambda: self.on_take('laser')),
                               ('release_tool', self.on_release),
                               ('start_work', self.on_start_work),
                               ('stop_work', self.on_stop_work),
                               ('go_home', self.on_go_home)):
            self.create_service(Trigger, f'~/{name}',
                                lambda _req, res, cb=callback: self.respond(res, *cb()))

        self.create_timer(0.02, self.on_sample)
        self.create_timer(0.1, self.on_publish_markers)

        self.busy.acquire()
        threading.Thread(target=self.run, args=(self.setup_scene,), daemon=True).start()
        self.publish_status()

    # ---------------------------------------------------------------- requests

    def respond(self, response, success, message):
        response.success = success
        response.message = message
        return response

    def on_take(self, name):
        tool = cell.TOOLS[name]
        if self.held == name:
            return False, f'уже установлен инструмент: {tool.label}'
        return self.submit(self.take_tool, tool)

    def on_release(self):
        if self.held is None:
            return False, 'инструмент не установлен'
        return self.submit(self.release_tool)

    def on_start_work(self):
        if self.held is None:
            return False, 'сначала возьмите инструмент'
        return self.submit(self.work)

    def on_stop_work(self):
        if not self.working:
            return False, 'работа не идёт'
        self.stop_event.set()
        self.moveit.stop()
        return True, 'останавливаю'

    def on_go_home(self):
        if self.home is None:
            return False, f'нет позы {HOME_STATE} в SRDF'
        return self.submit(self.go_home)

    def submit(self, job, *args):
        if not self.busy.acquire(blocking=False):
            return False, f'занят: {self.state}'
        # Waiting on MoveIt must not block the executor that delivers its responses.
        threading.Thread(target=self.run, args=(job, *args), daemon=True).start()
        return True, 'принято'

    def run(self, job, *args):
        try:
            ok, message = job(*args)
            self.set_state('готов' if ok else f'ошибка: {message}')
            if not ok:
                self.get_logger().error(message)
        except Exception as e:  # keep the node alive and the lock released
            self.set_state(f'ошибка: {e}')
            self.get_logger().error(f'{job.__name__} failed: {e}')
        finally:
            self.working = False
            self.process_on = False
            self.busy.release()

    # ------------------------------------------------------------------- jobs

    def setup_scene(self):
        if not self.moveit.wait_for_servers():
            return False, 'move_group недоступен'

        # Leftovers of a previous run of this node: detach the tool, remove the cell.
        existing = self.moveit.scene_object_ids()
        if existing is None:
            return False, 'move_group не отдал сцену'
        world_ids, attached_ids = existing
        ours = set(self.world_object_ids())
        cleanup = self.scene_diff()
        for object_id in ours.intersection(attached_ids):
            aco = AttachedCollisionObject(link_name=EE_LINK)
            aco.object.id = object_id
            aco.object.operation = CollisionObject.REMOVE
            cleanup.robot_state.attached_collision_objects.append(aco)
        # a detached object goes back to the world, so remove it there as well
        for object_id in ours.intersection(world_ids + attached_ids):
            cleanup.world.collision_objects.append(
                CollisionObject(id=object_id, operation=CollisionObject.REMOVE))
        if (cleanup.robot_state.attached_collision_objects or cleanup.world.collision_objects) \
                and not self.moveit.apply_scene(cleanup):
            return False, 'не удалось очистить сцену MoveIt'

        scene = self.scene_diff()
        for box in cell.TABLES:
            scene.world.collision_objects.append(self.box_object(box))
        for tool in cell.TOOLS.values():
            scene.world.collision_objects.append(self.holder_object(tool))
            scene.world.collision_objects.append(self.tool_object(tool))
        if not self.moveit.apply_scene(scene):
            return False, 'не удалось добавить ячейку в сцену MoveIt'
        self.get_logger().info('Work cell added to the planning scene')
        return True, ''

    def take_tool(self, tool):
        if self.held is not None:
            ok, message = self.release_tool()
            if not ok:
                return False, message

        self.set_state(f'беру: {tool.label}')
        above, grasp = self.slot_poses(tool)
        ok, message = self.move(above)
        if not ok:
            return False, message
        # The last centimetres touch the tool and its holder on purpose.
        ok, message = self.path([grasp], avoid_collisions=False)
        if not ok:
            return False, message
        if not self.moveit.apply_scene(self.attach_diff(tool, attach=True)):
            return False, 'не удалось прикрепить инструмент'
        self.held = tool.name
        ok, message = self.path([above], avoid_collisions=False)
        if not ok:
            return False, message
        self.get_logger().info(f'Took {tool.name}')
        return True, ''

    def release_tool(self):
        tool = cell.TOOLS[self.held]
        self.set_state(f'снимаю: {tool.label}')
        above, grasp = self.slot_poses(tool)
        ok, message = self.move(above)
        if not ok:
            return False, message
        ok, message = self.path([grasp], avoid_collisions=False)
        if not ok:
            return False, message
        if not self.moveit.apply_scene(self.attach_diff(tool, attach=False)):
            return False, 'не удалось отцепить инструмент'
        self.held = None
        ok, message = self.path([above], avoid_collisions=False)
        if not ok:
            return False, message
        self.get_logger().info(f'Released {tool.name}')
        return True, ''

    def work(self):
        """Run the program of the held tool once, then go home.

        Stop: the robot stops, lifts the tool clear of the part and stays there.
        """
        tool = cell.TOOLS[self.held]
        program = self.milling if tool.name == 'mill' else self.laser_cutting
        self.stop_event.clear()
        self.working = True
        self.safe_z = None
        self.traces[tool.name] = []
        self.set_state('работа: фрезеровка' if tool.name == 'mill' else 'работа: лазерная резка')

        ok, message = program(tool)
        if not self.stop_event.is_set():
            if not ok:
                return False, message
            ok, message = self.go_home()
        if self.stop_event.is_set():
            self.set_state('останавливаю')
            self.process_on = False
            self.working = False
            time.sleep(SETTLE_TIME_S)
            return self.retract()
        return ok, message

    def go_home(self):
        self.set_state('еду домой')
        if self.working and self.stop_event.is_set():
            return False, 'остановлено'
        return self.moveit.move_to_joints(
            self.home, self.velocity_scaling, self.acceleration_scaling)

    def milling(self, tool):
        """Raster over the workpiece, end mill on the surface."""
        z = cell.MILL_SURFACE_Z + tool.tip
        (x0, x1), (y0, y1) = cell.MILL_AREA
        passes = int(round((x1 - x0) / cell.MILL_STEP)) + 1
        points = []
        for i in range(passes):
            x = x0 + i * cell.MILL_STEP
            ys = (y0, y1) if i % 2 == 0 else (y1, y0)
            points += [(x, ys[0]), (x, ys[1])]
        return self.process_contours([(points, True)], z)

    def laser_cutting(self, tool):
        """A five-pointed star inside a circle, both starting at the top."""
        z = cell.LASER_SURFACE_Z + cell.LASER_STANDOFF + tool.tip
        cx, cy = cell.LASER_CENTER

        def polar(radius, angle):
            return cx + radius * math.cos(angle), cy + radius * math.sin(angle)

        circle = [polar(cell.LASER_RADIUS, math.pi / 2 + 2 * math.pi * i / 90) for i in range(91)]
        star = [polar(cell.LASER_STAR_RADIUS * (1.0 if i % 2 == 0 else cell.LASER_STAR_INNER),
                      math.pi / 2 + math.pi * i / 5)
                for i in range(11)]
        return self.process_contours([(circle, False), (star, True)], z)

    def process_contours(self, contours, z):
        """Cut contours at height z: approach from above, process on along each contour,
        process off for the moves between contours, retract at the end.

        contours: list of (points, sharp). A sharp contour stops at every vertex, so its
        corners stay corners; a smooth one (circle) runs as a single path.
        """
        if self.stop_event.is_set():
            return True, ''
        self.safe_z = z + cell.APPROACH_HEIGHT
        x, y = contours[0][0][0]
        ok, message = self.move(self.pose(x, y, self.safe_z))
        if not ok:
            return False, message

        previous = None
        for points, sharp in contours:
            x, y = points[0]
            if previous is None:
                lead_in = [self.pose(x, y, z)]
            else:
                px, py = previous
                lead_in = [self.pose(px, py, z + LEAD_IN_HEIGHT),
                           self.pose(x, y, z + LEAD_IN_HEIGHT), self.pose(x, y, z)]
            ok, message = self.path(lead_in, avoid_collisions=False)
            if not ok:
                return False, message

            cut = [[self.pose(px, py, z)] for px, py in points[1:]] if sharp \
                else [[self.pose(px, py, z) for px, py in points[1:]]]
            self.process_on = True
            try:
                for waypoints in cut:
                    ok, message = self.path(waypoints, avoid_collisions=False,
                                            velocity_scaling=self.process_velocity_scaling)
                    if not ok:
                        return False, message
            finally:
                self.process_on = False
            previous = points[-1]

        x, y = previous
        return self.path([self.pose(x, y, self.safe_z)], avoid_collisions=False)

    def retract(self):
        """Lift the tool straight up to the safe height above the part, if it is below it."""
        current = self.ee_pose()
        if current is None:
            return False, 'нет TF world -> tool0'
        if self.safe_z is None or current.position.z >= self.safe_z - 0.001:
            return True, ''
        up = make_pose(current.position.x, current.position.y, self.safe_z, self.tool_down)
        return self.path([up], avoid_collisions=False)

    def move(self, pose):
        """Collision-free motion to pose, always in the same arm configuration (ik_seed)."""
        if self.working and self.stop_event.is_set():
            return False, 'остановлено'
        seed = cell.ik_seed(pose.position.x, pose.position.y)
        return self.moveit.move_to_pose(
            pose, self.velocity_scaling, self.acceleration_scaling, seed)

    def path(self, waypoints, avoid_collisions=True, velocity_scaling=None):
        if self.working and self.stop_event.is_set():
            return False, 'остановлено'
        return self.moveit.follow_path(
            waypoints, velocity_scaling or self.velocity_scaling, self.acceleration_scaling,
            avoid_collisions=avoid_collisions)

    # ------------------------------------------------------------------ scene

    def pose(self, x, y, z):
        return make_pose(x, y, z, self.tool_down)

    def slot_poses(self, tool):
        x, y = tool.slot
        return (self.pose(x, y, tool.grasp_z + cell.APPROACH_HEIGHT),
                self.pose(x, y, tool.grasp_z))

    def world_object_ids(self):
        ids = [box.name for box in cell.TABLES]
        for tool in cell.TOOLS.values():
            ids += [f'holder_{tool.name}', tool_id(tool)]
        return ids

    @staticmethod
    def scene_diff():
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        return scene

    @staticmethod
    def box_object(box):
        obj = CollisionObject(id=box.name, operation=CollisionObject.ADD)
        obj.header.frame_id = WORLD
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(box.size)))
        obj.primitive_poses.append(make_pose(*box.center, (0.0, 0.0, 0.0, 1.0)))
        return obj

    @staticmethod
    def holder_object(tool):
        obj = CollisionObject(id=f'holder_{tool.name}', operation=CollisionObject.ADD)
        obj.header.frame_id = WORLD
        obj.primitives.append(SolidPrimitive(
            type=SolidPrimitive.CYLINDER, dimensions=[cell.HOLDER_HEIGHT, cell.HOLDER_RADIUS]))
        obj.primitive_poses.append(make_pose(
            *tool.slot, cell.TABLE_HEIGHT + cell.HOLDER_HEIGHT / 2, (0.0, 0.0, 0.0, 1.0)))
        return obj

    @staticmethod
    def tool_object(tool):
        """Tool standing in its holder: tool0 +z points down, so parts go below grasp_z."""
        obj = CollisionObject(id=tool_id(tool), operation=CollisionObject.ADD)
        obj.header.frame_id = WORLD
        for part in tool.parts:
            obj.primitives.append(part_primitive(part))
            obj.primitive_poses.append(make_pose(
                *tool.slot, tool.grasp_z - (part.z_from + part.z_to) / 2, (0.0, 0.0, 0.0, 1.0)))
        return obj

    def attach_diff(self, tool, attach):
        """Move the tool between the world and tool0; MoveIt keeps its geometry."""
        aco = AttachedCollisionObject(link_name=EE_LINK, touch_links=TOUCH_LINKS)
        aco.object.id = tool_id(tool)
        aco.object.header.frame_id = WORLD
        aco.object.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
        scene = self.scene_diff()
        scene.robot_state.attached_collision_objects.append(aco)
        return scene

    def on_srdf(self, msg):
        try:
            root = ElementTree.fromstring(msg.data)
        except ElementTree.ParseError as e:
            self.get_logger().error(f'Cannot parse the SRDF: {e}')
            return
        for state in root.iter('group_state'):
            if state.get('name') == HOME_STATE and state.get('group') == self.group_name:
                self.home = {j.get('name'): float(j.get('value')) for j in state.iter('joint')}
                return
        self.get_logger().error(f'No group state "{HOME_STATE}" for {self.group_name} in the SRDF')

    # ---------------------------------------------------------- visualization

    def ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(WORLD, EE_LINK, Time())
        except tf2_ros.TransformException:
            return None
        pose = Pose()
        t, q = tf.transform.translation, tf.transform.rotation
        pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
        pose.orientation = q
        return pose

    def tip_point(self):
        pose = self.ee_pose()
        if pose is None or self.held is None:
            return None
        q, tip = pose.orientation, cell.TOOLS[self.held].tip
        # tool0 z axis in the world frame
        zx = 2 * (q.x * q.z + q.w * q.y)
        zy = 2 * (q.y * q.z - q.w * q.x)
        zz = 1 - 2 * (q.x * q.x + q.y * q.y)
        p = pose.position
        return Point(x=p.x + zx * tip, y=p.y + zy * tip, z=p.z + zz * tip)

    def on_sample(self):
        """Record the tool tip, projected on the work surface, while the process is on;
        every switch-on starts a stroke."""
        if not self.process_on or self.held is None:
            self.stroke = None
            return
        tip = self.tip_point()
        if tip is None:
            return
        tip.z = cell.WORK_SURFACE_Z[self.held] + TRACE_LIFT
        strokes = self.traces[self.held]
        if self.stroke is None:
            self.stroke = []
            strokes.append(self.stroke)
        last = self.stroke[-1] if self.stroke else None
        if last is None or math.dist((last.x, last.y, last.z),
                                     (tip.x, tip.y, tip.z)) > TRACE_MIN_STEP:
            self.stroke.append(tip)
            while len(strokes) > 1 and sum(len(s) for s in strokes) > TRACE_MAX_POINTS:
                strokes.pop(0)

    def on_publish_markers(self):
        self.marker_pub.publish(self.markers())

    def markers(self):
        # No stamp: RViz then uses the latest TF. A stamp of "now" is often newer than
        # the latest tool0 transform, and RViz hides such markers until the next message.
        array = MarkerArray()

        def marker(ns, mid, mtype, frame=WORLD):
            m = Marker(ns=ns, id=mid, type=mtype, action=Marker.ADD)
            m.header.frame_id = frame
            m.frame_locked = frame == EE_LINK
            m.pose.orientation.w = 1.0
            array.markers.append(m)
            return m

        for i, box in enumerate(cell.TABLES):
            m = marker('cell', i, Marker.CUBE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = box.center
            m.scale.x, m.scale.y, m.scale.z = box.size
            m.color = color(box.color)

        for t, tool in enumerate(cell.TOOLS.values()):
            m = marker('holders', t, Marker.CYLINDER)
            m.pose.position.x, m.pose.position.y = tool.slot
            m.pose.position.z = cell.TABLE_HEIGHT + cell.HOLDER_HEIGHT / 2
            m.scale.x = m.scale.y = 2 * cell.HOLDER_RADIUS
            m.scale.z = cell.HOLDER_HEIGHT
            m.color = color(cell.DARK)

            held = self.held == tool.name
            for i, part in enumerate(tool.parts):
                m = marker(tool_id(tool), i,
                           Marker.CYLINDER if part.shape == 'cylinder' else Marker.CUBE,
                           EE_LINK if held else WORLD)
                center = (part.z_from + part.z_to) / 2
                if held:
                    m.pose.position.z = center
                else:
                    m.pose.position.x, m.pose.position.y = tool.slot
                    m.pose.position.z = tool.grasp_z - center
                m.scale.x = m.scale.y = 2 * part.size if part.shape == 'cylinder' else part.size
                m.scale.z = part.z_to - part.z_from
                m.color = color(part.color)

        for i, (name, strokes) in enumerate(self.traces.items()):
            m = marker('trace', i, Marker.LINE_LIST)
            m.points = [p for stroke in strokes for a, b in zip(stroke, stroke[1:]) for p in (a, b)]
            if not m.points:
                m.action = Marker.DELETE
                continue
            m.scale.x = 0.004 if name == 'mill' else 0.002
            m.color = color((0.25, 0.25, 0.28, 1.0) if name == 'mill' else (0.15, 0.05, 0.0, 1.0))

        beam = marker('beam', 0, Marker.LINE_LIST, EE_LINK)
        if self.process_on and self.held == 'laser':
            tip = cell.LASER.tip
            beam.points = [Point(z=tip), Point(z=tip + cell.LASER_STANDOFF)]
            beam.scale.x = 0.003
            beam.color = color((1.0, 0.1, 0.05, 1.0))
        else:
            beam.action = Marker.DELETE
        return array

    def set_state(self, state):
        self.state = state
        self.publish_status()

    def publish_status(self):
        tool = cell.TOOLS[self.held].label if self.held else 'нет'
        self.status_pub.publish(String(data=f'Инструмент: {tool}\nСостояние: {self.state}'))


def main():
    rclpy.init()
    node = WorkCell()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
