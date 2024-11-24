import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from .lib.drive import stop, forward, backward
from .lib.steering import turnLeft, turnRight


class Autocar(Node):
  def __init__(self):
    super().__init__("autocar")
    self.create_subscription(AckermannDriveStamped, "/cmd_vel", self.drive_msg_callback, 10)

  def drive_msg_callback(self, msg: AckermannDriveStamped):
    self.ackermann_msgs = msg
    self.get_logger().info(f"msg.drive.speed : {msg.drive.speed}")
    self.get_logger().info(f"msg.drive.acceleration : {msg.drive.acceleration}")
    self.get_logger().info(f"msg.drive.steering_angle : {msg.drive.steering_angle}")
    self.get_logger().info(f"msg.drive.steering_angle_velocity : {msg.drive.steering_angle_velocity}")
    turnLeft(msg.drive.steering_angle)

def main():
  rclpy.init()
  node = Autocar()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.destroy_node()

if __name__ == "__main__":
  main()