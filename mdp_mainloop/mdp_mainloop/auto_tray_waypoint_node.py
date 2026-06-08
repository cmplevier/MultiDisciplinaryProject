"""Generate tray A/B/C/D waypoints from clicked occupied map regions."""

from collections import deque
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_PLAN = {
    'trays': [],
    'rows': [],
    'return_home_pose': None,
}


class AutoTrayWaypointNode(Node):
    """Create tray scan waypoints by clicking occupied trays in RViz."""

    def __init__(self):
        super().__init__('mdp_auto_tray_waypoint_node')
        self.get_logger().info('MDP automatic tray waypoint generator started')

        self.declare_parameter(
            'plan_path',
            '~/mdp_ws/generated_row_plan.json',
        )
        self.declare_parameter('clear_on_start', False)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('tray_id_topic', '/row_plan/tray_id')
        self.declare_parameter('plan_topic', '/planner/row_plan')
        self.declare_parameter('status_topic', '/row_plan/auto_status')
        self.declare_parameter('marker_topic', '/row_plan/auto_tray_markers')

        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('unknown_is_obstacle', False)
        self.declare_parameter('click_search_radius_m', 0.25)
        self.declare_parameter('min_component_cells', 4)
        self.declare_parameter('max_component_cells', 20000)
        self.declare_parameter('connectivity', 8)

        self.declare_parameter('longitudinal_margin_m', 0.20)
        self.declare_parameter('lateral_offset_m', 0.35)
        self.declare_parameter('pose_yaw_mode', 'face_tray')
        self.declare_parameter('reverse_second_row', True)
        self.declare_parameter('consume_tray_id', True)
        self.declare_parameter('use_fixed_scan_yaw', False)
        self.declare_parameter('fixed_scan_yaw', 0.0)

        self.plan_path = os.path.expanduser(
            str(self.get_parameter('plan_path').value)
        )
        self.clear_on_start = self.get_parameter('clear_on_start').value
        map_topic = self.get_parameter('map_topic').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value
        tray_id_topic = self.get_parameter('tray_id_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        status_topic = self.get_parameter('status_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        self.occupied_threshold = self.get_parameter(
            'occupied_threshold'
        ).value
        self.unknown_is_obstacle = self.get_parameter(
            'unknown_is_obstacle'
        ).value
        self.click_search_radius_m = self.get_parameter(
            'click_search_radius_m'
        ).value
        self.min_component_cells = self.get_parameter(
            'min_component_cells'
        ).value
        self.max_component_cells = self.get_parameter(
            'max_component_cells'
        ).value
        self.connectivity = int(self.get_parameter('connectivity').value)
        self.longitudinal_margin_m = self.get_parameter(
            'longitudinal_margin_m'
        ).value
        self.lateral_offset_m = self.get_parameter('lateral_offset_m').value
        self.pose_yaw_mode = str(
            self.get_parameter('pose_yaw_mode').value
        ).lower()
        self.reverse_second_row = self.get_parameter(
            'reverse_second_row'
        ).value
        self.consume_tray_id = self.get_parameter('consume_tray_id').value
        self.use_fixed_scan_yaw = self.get_parameter(
            'use_fixed_scan_yaw'
        ).value
        self.fixed_scan_yaw = self.get_parameter('fixed_scan_yaw').value

        self.map_msg = None
        self.pending_tray_id = None
        self.last_generated = None
        self.last_status = 'waiting_for_map_and_clicks'

        if self.clear_on_start:
            self.plan = self.empty_plan()
            self.save_plan()
        else:
            self.plan = self.load_plan()

        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.plan_pub = self.create_publisher(String, plan_topic, latched_qos)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            latched_qos,
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            clicked_point_topic,
            self.clicked_point_callback,
            10,
        )
        self.tray_id_sub = self.create_subscription(
            String,
            tray_id_topic,
            self.tray_id_callback,
            10,
        )

        self.timer = self.create_timer(0.5, self.publish_outputs)
        self.publish_plan()

    def load_plan(self):
        """Load an existing generated plan or return an empty one."""
        if not os.path.exists(self.plan_path):
            return self.empty_plan()

        try:
            with open(self.plan_path, 'r', encoding='utf-8') as handle:
                plan = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'Could not load plan {self.plan_path}: {exc}; starting empty'
            )
            return self.empty_plan()

        if not isinstance(plan, dict):
            self.get_logger().warn('Plan root is not an object; starting empty')
            return self.empty_plan()

        plan.setdefault('trays', [])
        plan.setdefault('rows', [])
        plan.setdefault('return_home_pose', None)
        return plan

    @staticmethod
    def empty_plan():
        """Return a fresh empty row-plan object."""
        return {
            'trays': [],
            'rows': [],
            'return_home_pose': None,
        }

    def save_plan(self):
        """Write the generated plan to disk."""
        directory = os.path.dirname(self.plan_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        try:
            with open(self.plan_path, 'w', encoding='utf-8') as handle:
                json.dump(self.plan, handle, indent=4)
        except OSError as exc:
            self.get_logger().error(f'Could not save plan: {exc}')

    def map_callback(self, msg):
        """Store the latest occupancy grid map."""
        self.map_msg = msg
        self.last_status = (
            f'map_ready {msg.info.width}x{msg.info.height} '
            f'@ {msg.info.resolution:.3f}m'
        )

    def tray_id_callback(self, msg):
        """Set the tray ID used by the next RViz click."""
        tray_id = msg.data.strip()
        if not tray_id:
            return
        self.pending_tray_id = tray_id
        self.last_status = f'next_click_will_generate_{tray_id}'
        self.get_logger().info(f'Next clicked tray ID: {tray_id}')

    def clicked_point_callback(self, msg):
        """Generate waypoints for the occupied map component under a click."""
        if self.map_msg is None:
            self.get_logger().warn('Ignoring click; no /map received yet')
            return

        clicked_cell = self.world_to_map_cell(msg.point.x, msg.point.y)
        if clicked_cell is None:
            self.get_logger().warn('Ignoring click outside the map bounds')
            return

        seed = self.find_nearest_occupied_cell(clicked_cell)
        if seed is None:
            self.get_logger().warn(
                'Click did not land near an occupied map cell. '
                'Try clicking closer to the black tray region.'
            )
            return

        component = self.flood_fill_component(seed)
        if component is None:
            return
        if len(component) < self.min_component_cells:
            self.get_logger().warn(
                f'Ignoring tiny occupied component ({len(component)} cells)'
            )
            return

        tray_id = self.pending_tray_id or self.next_auto_tray_id()
        tray = self.make_tray_from_component(tray_id, component)
        if tray is None:
            return

        self.upsert_tray(tray)
        self.save_plan()
        self.publish_plan()
        self.last_generated = tray
        self.last_status = (
            f'generated {tray_id} from {len(component)} occupied cells'
        )
        self.get_logger().info(
            f"Generated tray {tray_id}: "
            f"A={tray['waypoints']['A']} B={tray['waypoints']['B']} "
            f"C={tray['waypoints']['C']} D={tray['waypoints']['D']}"
        )

        if self.consume_tray_id:
            self.pending_tray_id = None

    def find_nearest_occupied_cell(self, clicked_cell):
        """Return the nearest occupied cell around a clicked map cell."""
        resolution = self.map_msg.info.resolution
        radius_cells = max(
            0,
            int(math.ceil(self.click_search_radius_m / resolution)),
        )
        cx, cy = clicked_cell
        candidates = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                candidates.append((dx * dx + dy * dy, cx + dx, cy + dy))

        for _, mx, my in sorted(candidates):
            if self.cell_is_occupied(mx, my):
                return mx, my
        return None

    def flood_fill_component(self, seed):
        """Return all connected occupied cells containing seed."""
        queue = deque([seed])
        visited = {seed}
        component = []
        neighbors = self.neighbor_offsets()

        while queue:
            cell = queue.popleft()
            component.append(cell)
            if len(component) > self.max_component_cells:
                self.get_logger().warn(
                    'Clicked obstacle is too large. This may be a wall or '
                    'merged obstacle, so no tray was generated.'
                )
                return None

            mx, my = cell
            for dx, dy in neighbors:
                neighbor = (mx + dx, my + dy)
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                if self.cell_is_occupied(neighbor[0], neighbor[1]):
                    queue.append(neighbor)

        return component

    def neighbor_offsets(self):
        """Return 4- or 8-connected grid neighbors."""
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.connectivity == 8:
            offsets.extend([
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ])
        return offsets

    def make_tray_from_component(self, tray_id, component):
        """Fit an oriented rectangle and convert it to tray waypoints."""
        points = [self.map_cell_center_world(mx, my) for mx, my in component]
        axis = self.scan_axis(points)
        if axis is None:
            return None

        ux, uy = axis
        vx, vy = -uy, ux
        projections_u = [x * ux + y * uy for x, y in points]
        projections_v = [x * vx + y * vy for x, y in points]

        min_u = min(projections_u) - self.longitudinal_margin_m
        max_u = max(projections_u) + self.longitudinal_margin_m
        min_v = min(projections_v) - self.lateral_offset_m
        max_v = max(projections_v) + self.lateral_offset_m

        if max_u <= min_u or max_v <= min_v:
            self.get_logger().warn('Could not compute a valid tray rectangle')
            return None

        def point_from_projection(u_value, v_value):
            return (
                u_value * ux + v_value * vx,
                u_value * uy + v_value * vy,
            )

        lower_yaw, upper_yaw = self.row_yaws(ux, uy, vx, vy)
        if self.reverse_second_row:
            waypoint_specs = {
                'A': (*point_from_projection(min_u, min_v), lower_yaw),
                'B': (*point_from_projection(max_u, min_v), lower_yaw),
                'C': (*point_from_projection(max_u, max_v), upper_yaw),
                'D': (*point_from_projection(min_u, max_v), upper_yaw),
            }
        else:
            waypoint_specs = {
                'A': (*point_from_projection(min_u, min_v), lower_yaw),
                'B': (*point_from_projection(max_u, min_v), lower_yaw),
                'C': (*point_from_projection(min_u, max_v), upper_yaw),
                'D': (*point_from_projection(max_u, max_v), upper_yaw),
            }

        waypoints = {
            name: [round(x, 3), round(y, 3), round(self.wrap_angle(yaw), 4)]
            for name, (x, y, yaw) in waypoint_specs.items()
        }
        return {'id': str(tray_id), 'waypoints': waypoints}

    def scan_axis(self, points):
        """Return the long-axis unit vector for an occupied component."""
        if self.use_fixed_scan_yaw:
            return math.cos(self.fixed_scan_yaw), math.sin(self.fixed_scan_yaw)

        count = len(points)
        if count < 2:
            self.get_logger().warn('Need at least two occupied cells')
            return None

        mean_x = sum(point[0] for point in points) / count
        mean_y = sum(point[1] for point in points) / count
        cov_xx = 0.0
        cov_xy = 0.0
        cov_yy = 0.0
        for x, y in points:
            dx = x - mean_x
            dy = y - mean_y
            cov_xx += dx * dx
            cov_xy += dx * dy
            cov_yy += dy * dy

        cov_xx /= count
        cov_xy /= count
        cov_yy /= count
        if cov_xx + cov_yy <= 1e-8:
            self.get_logger().warn('Clicked component is too small to orient')
            return None

        yaw = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
        ux = math.cos(yaw)
        uy = math.sin(yaw)

        # Keep generated A/B/C/D order stable in map coordinates.
        if ux < 0.0 or (abs(ux) < 1e-6 and uy < 0.0):
            ux = -ux
            uy = -uy
        return ux, uy

    def row_yaws(self, ux, uy, vx, vy):
        """Return yaw values for the two scan sides."""
        if self.pose_yaw_mode == 'scan_axis':
            lower_yaw = math.atan2(uy, ux)
            if self.reverse_second_row:
                upper_yaw = math.atan2(-uy, -ux)
            else:
                upper_yaw = lower_yaw
            return lower_yaw, upper_yaw

        # Default: face the tray while moving along the row, so the row
        # traversal is a lateral strafe in the robot frame.
        lower_yaw = math.atan2(vy, vx)
        upper_yaw = math.atan2(-vy, -vx)
        return lower_yaw, upper_yaw

    def upsert_tray(self, tray):
        """Add or replace one tray in the plan."""
        trays = self.plan.setdefault('trays', [])
        for index, existing in enumerate(trays):
            existing_id = existing.get('id') or existing.get('tray_id')
            if existing_id == tray['id']:
                trays[index] = tray
                return
        trays.append(tray)

    def next_auto_tray_id(self):
        """Return the next available tray_N identifier."""
        existing_ids = {
            str(tray.get('id') or tray.get('tray_id'))
            for tray in self.plan.get('trays', [])
        }
        index = len(existing_ids) + 1
        while f'tray_{index}' in existing_ids:
            index += 1
        return f'tray_{index}'

    def publish_outputs(self):
        """Publish status and markers periodically."""
        self.publish_status()
        self.publish_markers()

    def publish_plan(self):
        """Publish the full JSON plan."""
        msg = String()
        msg.data = json.dumps(self.plan)
        self.plan_pub.publish(msg)

    def publish_status(self):
        """Publish generator status as JSON."""
        status = {
            'status': self.last_status,
            'plan_path': self.plan_path,
            'map_received': self.map_msg is not None,
            'pending_tray_id': self.pending_tray_id,
            'tray_count': len(self.plan.get('trays', [])),
            'longitudinal_margin_m': self.longitudinal_margin_m,
            'lateral_offset_m': self.lateral_offset_m,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_markers(self):
        """Publish generated tray rectangles and waypoint labels."""
        marker_array = MarkerArray()
        marker_array.markers.append(self.delete_all_marker())
        stamp = self.get_clock().now().to_msg()
        marker_id = 1
        for tray in self.plan.get('trays', []):
            marker_id = self.add_tray_markers(
                marker_array,
                marker_id,
                tray,
                stamp,
            )
        self.marker_pub.publish(marker_array)

    def add_tray_markers(self, marker_array, marker_id, tray, stamp):
        """Add line, arrow, and label markers for one tray."""
        waypoints = tray.get('waypoints', {})
        if not all(name in waypoints for name in ['A', 'B', 'C', 'D']):
            return marker_id

        marker_array.markers.append(
            self.make_rectangle_marker(marker_id, tray, stamp)
        )
        marker_id += 1

        for name in ['A', 'B', 'C', 'D']:
            pose = waypoints[name]
            marker_array.markers.append(
                self.make_arrow_marker(marker_id, pose, stamp)
            )
            marker_id += 1
            marker_array.markers.append(
                self.make_label_marker(
                    marker_id,
                    pose,
                    f"{tray['id']} {name}",
                    stamp,
                )
            )
            marker_id += 1

        return marker_id

    def make_rectangle_marker(self, marker_id, tray, stamp):
        """Create a line marker around the generated tray waypoints."""
        marker = Marker()
        marker.header.frame_id = self.map_msg.header.frame_id if self.map_msg else 'map'
        marker.header.stamp = stamp
        marker.ns = 'auto_tray_rectangles'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9

        waypoints = tray['waypoints']
        for name in ['A', 'B', 'C', 'D', 'A']:
            marker.points.append(self.pose_to_point(waypoints[name]))
        return marker

    def make_arrow_marker(self, marker_id, pose, stamp):
        """Create an arrow marker showing one generated waypoint yaw."""
        marker = Marker()
        marker.header.frame_id = self.map_msg.header.frame_id if self.map_msg else 'map'
        marker.header.stamp = stamp
        marker.ns = 'auto_tray_waypoint_arrows'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
        marker.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
        marker.scale.x = 0.25
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 0.9
        return marker

    def make_label_marker(self, marker_id, pose, text, stamp):
        """Create a text label marker for a generated waypoint."""
        marker = Marker()
        marker.header.frame_id = self.map_msg.header.frame_id if self.map_msg else 'map'
        marker.header.stamp = stamp
        marker.ns = 'auto_tray_waypoint_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker

    @staticmethod
    def delete_all_marker():
        """Create a marker that clears stale marker state."""
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def pose_to_point(pose):
        """Return a Point from a [x, y, yaw] pose."""
        point = Point()
        point.x = float(pose[0])
        point.y = float(pose[1])
        point.z = 0.02
        return point

    def world_to_map_cell(self, x, y):
        """Convert a world point to map-grid coordinates."""
        info = self.map_msg.info
        origin = info.origin
        yaw = self.yaw_from_pose(origin)
        dx = x - origin.position.x
        dy = y - origin.position.y
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        mx = int(math.floor(local_x / info.resolution))
        my = int(math.floor(local_y / info.resolution))
        if not self.cell_in_bounds(mx, my):
            return None
        return mx, my

    def map_cell_center_world(self, mx, my):
        """Convert a map-grid cell center to world coordinates."""
        info = self.map_msg.info
        origin = info.origin
        yaw = self.yaw_from_pose(origin)
        local_x = (mx + 0.5) * info.resolution
        local_y = (my + 0.5) * info.resolution
        world_x = (
            origin.position.x
            + local_x * math.cos(yaw)
            - local_y * math.sin(yaw)
        )
        world_y = (
            origin.position.y
            + local_x * math.sin(yaw)
            + local_y * math.cos(yaw)
        )
        return world_x, world_y

    def cell_is_occupied(self, mx, my):
        """Return true if a map cell belongs to an obstacle component."""
        if not self.cell_in_bounds(mx, my):
            return False

        info = self.map_msg.info
        value = self.map_msg.data[my * info.width + mx]
        if value < 0:
            return self.unknown_is_obstacle
        return value >= self.occupied_threshold

    def cell_in_bounds(self, mx, my):
        """Return true if a cell index is inside the map."""
        info = self.map_msg.info
        return 0 <= mx < info.width and 0 <= my < info.height

    @staticmethod
    def yaw_from_pose(pose):
        """Extract planar yaw from a pose orientation."""
        orientation = pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def wrap_angle(angle):
        """Wrap an angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    """Run the automatic tray waypoint generator node."""
    rclpy.init(args=args)
    node = AutoTrayWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
