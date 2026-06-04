#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


def parse_args():
    parser = argparse.ArgumentParser(
        description='Wait until lifecycle nodes reach the active state.'
    )
    parser.add_argument(
        '--nodes',
        nargs='+',
        required=True,
        help='Lifecycle node names, for example /map_server /amcl.',
    )
    parser.add_argument(
        '--poll-interval',
        type=float,
        default=0.5,
        help='Seconds between state checks.',
    )
    return parser.parse_args()


def get_state_service_name(node_name):
    return f'{node_name.rstrip("/")}/get_state'


def main():
    args = parse_args()

    rclpy.init()
    node = rclpy.create_node('wait_for_lifecycle_active')

    clients = {
        node_name: node.create_client(GetState, get_state_service_name(node_name))
        for node_name in args.nodes
    }
    active_nodes = set()

    try:
        node.get_logger().info(
            'Waiting for lifecycle nodes to become active: '
            + ', '.join(args.nodes)
        )

        while rclpy.ok():
            for node_name, client in clients.items():
                if node_name in active_nodes:
                    continue

                if not client.wait_for_service(timeout_sec=args.poll_interval):
                    continue

                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    node,
                    future,
                    timeout_sec=args.poll_interval,
                )

                if not future.done() or future.result() is None:
                    continue

                state = future.result().current_state
                if state.id == State.PRIMARY_STATE_ACTIVE:
                    active_nodes.add(node_name)
                    node.get_logger().info(f'{node_name} is active')
                else:
                    node.get_logger().info(
                        f'{node_name} is {state.label or state.id}; waiting'
                    )

            if len(active_nodes) == len(clients):
                node.get_logger().info('All requested lifecycle nodes are active')
                return 0

            time.sleep(args.poll_interval)

    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 1


if __name__ == '__main__':
    sys.exit(main())
