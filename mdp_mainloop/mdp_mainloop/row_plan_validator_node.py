"""Validate a generated row-plan JSON file."""

import json
import os
import sys

import rclpy
from rclpy.node import Node


class RowPlanValidatorNode(Node):
    """Load a row-plan JSON file and report validation errors."""

    def __init__(self):
        super().__init__('mdp_row_plan_validator_node')
        self.declare_parameter(
            'plan_path',
            '~/mdp_ws/generated_row_plan.json',
        )
        plan_path = self.get_parameter('plan_path').value
        self.plan_path = os.path.expanduser(plan_path)

    def validate(self):
        """Return true when the configured row-plan file is valid."""
        if not os.path.exists(self.plan_path):
            self.get_logger().error(f'Plan file does not exist: {self.plan_path}')
            return False

        try:
            with open(self.plan_path, 'r') as file:
                plan = json.load(file)
        except OSError as exc:
            self.get_logger().error(f'Could not read plan file: {exc}')
            return False
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Plan file is not valid JSON: {exc}')
            return False

        errors = self.validate_plan(plan)
        if errors:
            for error in errors:
                self.get_logger().error(error)
            return False

        rows = plan.get('rows', [])
        trays = plan.get('trays', [])
        tray_segment_count = 0
        for tray in trays:
            waypoints = tray.get('waypoints', {})
            if 'A' in waypoints and 'B' in waypoints:
                tray_segment_count += 1
            if 'C' in waypoints and 'D' in waypoints:
                tray_segment_count += 1
        total_segments = len(rows) + tray_segment_count
        self.get_logger().info(
            f'Plan OK: {self.plan_path} contains '
            f'{total_segments} valid scan segment(s)'
        )
        for index, row in enumerate(rows):
            self.get_logger().info(
                f"{index + 1}. {row['id']}: "
                f"approach={row['approach_pose']} "
                f"goal={row.get('goal_pose') or row['scan_end_pose']}"
            )
        for index, tray in enumerate(trays):
            self.get_logger().info(
                f"{index + 1}. tray {tray.get('id')}: "
                'A->B and C->D'
            )
        return True

    def validate_plan(self, plan):
        """Return a list of human-readable validation errors."""
        errors = []
        if not isinstance(plan, dict):
            return ['Plan root must be a JSON object']

        rows = plan.get('rows', [])
        trays = plan.get('trays', [])
        if not isinstance(rows, list):
            return ['rows must be a list when present']
        if not isinstance(trays, list):
            return ['trays must be a list when present']
        if not rows and not trays:
            errors.append('Plan contains no rows or trays')

        seen_ids = set()
        for index, row in enumerate(rows):
            row_label = f'rows[{index}]'
            if not isinstance(row, dict):
                errors.append(f'{row_label} must be an object')
                continue

            row_id = row.get('id')
            if not row_id:
                errors.append(f'{row_label} is missing id')
            elif row_id in seen_ids:
                errors.append(f'duplicate row id: {row_id}')
            else:
                seen_ids.add(row_id)

            if not self.is_pose(row.get('approach_pose')):
                errors.append(f'{row_label} has invalid approach_pose')

            goal_pose = row.get('scan_end_pose') or row.get('goal_pose')
            if not self.is_pose(goal_pose):
                errors.append(
                    f'{row_label} has invalid scan_end_pose/goal_pose'
                )

        seen_tray_ids = set()
        for index, tray in enumerate(trays):
            tray_label = f'trays[{index}]'
            if not isinstance(tray, dict):
                errors.append(f'{tray_label} must be an object')
                continue

            tray_id = tray.get('id') or tray.get('tray_id')
            if not tray_id:
                errors.append(f'{tray_label} is missing id')
            elif tray_id in seen_tray_ids:
                errors.append(f'duplicate tray id: {tray_id}')
            else:
                seen_tray_ids.add(tray_id)

            if not self.valid_tray_waypoints(tray.get('waypoints')):
                errors.append(f'{tray_label} needs valid waypoints A/B/C/D')

        return errors

    def valid_tray_waypoints(self, waypoints):
        """Return true when a tray has four valid waypoint poses."""
        if isinstance(waypoints, dict):
            return all(
                self.is_pose(waypoints.get(name))
                for name in ['A', 'B', 'C', 'D']
            )
        if isinstance(waypoints, list):
            return (
                len(waypoints) >= 4
                and all(self.is_pose(pose) for pose in waypoints[:4])
            )
        return False

    @staticmethod
    def is_pose(value):
        """Return true for [x, y] or [x, y, yaw] numeric arrays."""
        if not isinstance(value, list) or len(value) < 2:
            return False
        if len(value) > 3:
            return False
        try:
            for item in value:
                float(item)
        except (TypeError, ValueError):
            return False
        return True


def main(args=None):
    """Run the row-plan validator and exit with success/failure."""
    rclpy.init(args=args)
    node = RowPlanValidatorNode()
    valid = node.validate()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if valid else 1)


if __name__ == '__main__':
    main()
