import smtplib
from email.message import EmailMessage

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


# remove clock once using the actual loop. !!!!!!

def make_message(sender_email, recipient_email, completion_state, temperature, humidity, anomaly_bool):
    msg = EmailMessage()
    msg["From"] = sender_email
    msg["To"] = recipient_email

    if anomaly_bool:
        msg["Subject"] = "Anomaly detected by Mirte Master"
        msg.set_content(
            "Good day,\n\n"
            "An anomaly has been detected during the current inspection run, please inspect me and the current "
            "greenhouse conditions at my location.\n\n"
            "Best regards,\n"
            "Your mirte master"
        )
    else:
        msg["Subject"] = "Update on progress by Mirte Master"
        msg.set_content(
            f"Good day,\n\n"
            f"I am currently {completion_state}% finished with my current inspection run of the assigned plants.\n"
            f"Current greenhouse conditions are a temperature of {temperature} and a humidity of {humidity}.\n\n"
            f"Best regards,\n"
            f"Your mirte master"
        )

    return msg

# can be removed once actual sim is implemented. 
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

        self.latest_data_gh = None
        self.latest_data_plant = None
        self.latest_data_locations = None

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
            self.get_logger().info("waiting\n")

        current_minute = seconds // 60

        if current_minute != self.last_sent_minute:
            self.last_sent_minute = current_minute
            self.send_email_update()

    def send_email_update(self):
        anomaly_bool = None                                                         # implement
        completion_state, temperature, humidity = None, None, None                  # implement

        msg = make_message(
            self.sender_email,
            self.recipient_email,
            completion_state,
            temperature,
            humidity,
            anomaly_bool
        )

        with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
            server.login(self.sender_email, self.app_password)
            server.send_message(msg)

        self.get_logger().info("Email update sent.")


def email_loop(sender_email, app_password, recipient_email, rclpy_args=None):
    rclpy.init(args=rclpy_args)

    email_node = UpdateEmailNode(sender_email, app_password, recipient_email)
    clock_node = SimpleClockNode()                       # remove once implementing in actual sim

    try:
        while rclpy.ok():
            rclpy.spin_once(clock_node, timeout_sec=0.1)
            rclpy.spin_once(email_node, timeout_sec=0.1) # remove once implementing in actual sim

    finally:
        email_node.destroy_node()
        clock_node.destroy_node()                        # remove once implementing in actual sim
        rclpy.shutdown()