import json
import smtplib
from email.message import EmailMessage

import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


DUMMY_SERVICE_JSON = """
[
    {
        "tag_id": "4",
        "last_scan_time": 1749123456.78,
        "bug_detected": false,
        "total_flowers": 12,
        "flowers": {
            "white": 10
        },
        "sensor_data": {
            "temperature": 21.5,
            "humidity": 46.1,
            "light": null,
            "co2": null
        }
    }
]
"""

USE_DUMMY_SERVICE_DATA = False


def make_message(sender_email, recipient_email, flowers, sensor_data):
    msg = EmailMessage()

    msg["From"] = sender_email
    msg["To"] = recipient_email
    msg["Subject"] = "Update on progress by Mirte Master"

    msg.set_content(
        f"Good day,\n\n"
        f"The latest tray I scanned has scanned "
        f"{flowers.get('red', 0)} red plants and "
        f"{flowers.get('white', 0)} white plants.\n"
        f"Current greenhouse conditions are a temperature of: "
        f"{sensor_data.get('temperature', 'not available')} "
        f"and a humidity of: "
        f"{sensor_data.get('humidity', 'not available')}.\n\n"
        f"Best regards,\n"
        f"Your mirte master"
    )

    return msg


class SimpleClockNode(Node):
    def __init__(self):
        super().__init__("simple_clock_node")

        self.publisher = self.create_publisher(Clock, "clock", 10)
        self.seconds = 0
        self.timer = self.create_timer(1.0, self.publish_clock)

    def publish_clock(self):
        msg = Clock()

        msg.clock.sec = self.seconds
        msg.clock.nanosec = 0

        self.publisher.publish(msg)
        self.seconds += 1


class UpdateEmailNode(Node):
    def __init__(self, sender_email, app_password, recipient_email):
        super().__init__("update_email_node")

        self.sender_email = sender_email
        self.app_password = app_password
        self.recipient_email = recipient_email

        self.last_sent_minute = None
        self.last_waiting_second = None
        self.service_call_in_progress = False

        self.client = self.create_client(Trigger, "/state/get_last")

        self.create_subscription(
            Clock,
            "clock",
            self.clock_listener_callback,
            10
        )

    def clock_listener_callback(self, msg):
        seconds = msg.clock.sec

        if seconds % 10 == 0 and seconds != self.last_waiting_second:
            self.last_waiting_second = seconds
            self.get_logger().info("waiting")

        current_minute = seconds // 60

        if current_minute != self.last_sent_minute:
            self.last_sent_minute = current_minute
            self.send_email_update()

    def send_email_update(self):
        if USE_DUMMY_SERVICE_DATA:
            data = json.loads(DUMMY_SERVICE_JSON)
            flowers, sensor_data = self.parse_state_data(data)
            self.send_email_with_values(flowers, sensor_data)
            return

        if self.service_call_in_progress:
            self.get_logger().warn("Previous /state/get_last call still in progress")
            return

        if not self.client.service_is_ready():
            self.get_logger().error("/state/get_last service not available")
            flowers, sensor_data = self.default_values()
            self.send_email_with_values(flowers, sensor_data)
            return

        request = Trigger.Request()

        self.service_call_in_progress = True

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_state_response)

    def handle_state_response(self, future):
        self.service_call_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"/state/get_last call failed: {error}")
            flowers, sensor_data = self.default_values()
            self.send_email_with_values(flowers, sensor_data)
            return

        if response is None:
            self.get_logger().error("/state/get_last did not return a response")
            flowers, sensor_data = self.default_values()
            self.send_email_with_values(flowers, sensor_data)
            return

        if not response.success:
            self.get_logger().error(f"/state/get_last failed: {response.message}")
            flowers, sensor_data = self.default_values()
            self.send_email_with_values(flowers, sensor_data)
            return

        try:
            data = json.loads(response.message)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON received: {response.message}")
            flowers, sensor_data = self.default_values()
            self.send_email_with_values(flowers, sensor_data)
            return

        flowers, sensor_data = self.parse_state_data(data)
        self.send_email_with_values(flowers, sensor_data)

    def parse_state_data(self, data):
        if isinstance(data, list):
            if len(data) == 0:
                self.get_logger().error("Received empty tray list")
                return self.default_values()

            newest_tray = max(
                data,
                key=lambda tray: tray.get("last_scan_time", tray.get("scan_time", 0))
            )
        else:
            newest_tray = data

        flowers_raw = newest_tray.get("flowers", {})

        flowers = {
            "red": flowers_raw.get("red", 0),
            "white": flowers_raw.get("white", 0)
        }

        sensor_data_raw = newest_tray.get("sensor_data", {})

        temperature = sensor_data_raw.get("temperature")
        humidity = sensor_data_raw.get("humidity")

        sensor_data = {
            "temperature": temperature if temperature is not None else "not available",
            "humidity": humidity if humidity is not None else "not available"
        }

        return flowers, sensor_data

    def default_values(self):
        return (
            {
                "red": 0,
                "white": 0
            },
            {
                "temperature": "not available",
                "humidity": "not available"
            }
        )

    def send_email_with_values(self, flowers, sensor_data):
        msg = make_message(
            self.sender_email,
            self.recipient_email,
            flowers,
            sensor_data
        )

        try:
            with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
                server.login(self.sender_email, self.app_password)
                server.send_message(msg)

            self.get_logger().info("Email update sent.")

        except Exception as error:
            self.get_logger().error(f"Failed to send email: {error}")


def email_loop(sender_email, app_password, recipient_email, rclpy_args=None):
    rclpy.init(args=rclpy_args)

    email_node = UpdateEmailNode(
        sender_email,
        app_password,
        recipient_email
    )

    clock_node = SimpleClockNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(clock_node, timeout_sec=0.1)
            rclpy.spin_once(email_node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        email_node.destroy_node()
        clock_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()