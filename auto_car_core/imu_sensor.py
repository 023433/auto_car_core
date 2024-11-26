import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from .lib.imu import get_accel_x, get_accel_y, get_accel_z, get_gyro_x, get_gyro_y, get_gyro_z

class ImuSensor(Node):
  def __init__(self):
    super().__init__("imu_sensor")
    self.create_timer(1/60, self.update)
    self.publisher_ = self.create_publisher(Imu, "/imu", 10)

  def update(self):
    message = Imu()
    message.header.stamp = self.get_clock().now().to_msg()
    message.header.frame_id = "base_link"
    message.linear_acceleration.x = get_accel_x()
    message.linear_acceleration.y = get_accel_y()
    message.linear_acceleration.z = get_accel_z()
    message.angular_velocity.x = get_gyro_x()
    message.angular_velocity.y = get_gyro_y()
    message.angular_velocity.z = get_gyro_z()
    # Invalidate quaternion
    message.orientation.x = 0
    message.orientation.y = 0
    message.orientation.z = 0
    message.orientation.w = 0
    self.publisher_.publish(message)

def main():
  rclpy.init()
  node = ImuSensor()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.destroy_node()

if __name__ == "__main__":
  main()