"""
State node — persists perception results to SQLite.

  /perception/scan_result  (sub) — JSON from perception node
  /state/get_last          (srv) — returns the most recent scan as JSON
"""

import json
import sqlite3
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

DB_PATH = Path.home() / 'greenhouse.db'


def init_db(conn):
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS scans (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            tag_id        TEXT,
            scan_time     REAL,
            total_flowers INTEGER,
            bugs          INTEGER,
            flowers_json  TEXT,
            sensor_json   TEXT
        );
    """)
    conn.commit()


class StateNode(Node):
    def __init__(self):
        super().__init__('state_node')

        self._conn = sqlite3.connect(str(DB_PATH), check_same_thread=False)
        init_db(self._conn)

        self.create_subscription(String, '/perception/scan_result', self._scan_cb, 10)
        self.create_service(Trigger, '/state/get_last', self._get_last_cb)

        self.get_logger().info(f'StateNode ready. DB: {DB_PATH}')

    def _scan_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON on scan_result')
            return

        tag_id = data.get('tag_id')
        if not tag_id:
            return

        flowers = data.get('flowers', {})
        bugs = data.get('bugs', 0)
        sensor_data = data.get('sensor_data', {})

        self._conn.execute(
            'INSERT INTO scans (tag_id, scan_time, total_flowers, bugs, flowers_json, sensor_json) '
            'VALUES (?, ?, ?, ?, ?, ?)',
            (tag_id, time.time(), sum(flowers.values()), bugs,
             json.dumps(flowers), json.dumps(sensor_data))
        )
        self._conn.commit()
        self.get_logger().info(f'Scan saved — tray {tag_id}: {sum(flowers.values())} flowers, bugs={bugs}')

    def _get_last_cb(self, request, response):
        cur = self._conn.execute(
            'SELECT tag_id, scan_time, total_flowers, bugs, flowers_json, sensor_json '
            'FROM scans ORDER BY scan_time DESC LIMIT 1'
        )
        row = cur.fetchone()
        if row:
            response.success = True
            response.message = json.dumps({
                'tag_id':        row[0],
                'scan_time':     row[1],
                'total_flowers': row[2],
                'bugs':          row[3],
                'flowers':       json.loads(row[4]),
                'sensor_data':   json.loads(row[5]),
            })
        else:
            response.success = False
            response.message = json.dumps({})
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
