import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_bno055


class ReadIMU(Node):
    def __init__(self):
        super().__init__('imu_driver')
        
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        # Declaring parameters
        self.declare_parameter('imu_topic', 'imu')
                        
        # Reading in parameters for node, by default the parameters can be found
        # under config/params.yaml
        self.imu_topic = self.get_parameter('imu_topic')
                                      
        # Create topic publishers for IMU        
        self.imu_output = self.create_publisher(Imu, self.imu_topic.value , 1)
        
        # Create timer for IMU publisher.
        self.rate = 100 # 100 Hz
        self.timer_period = 1.0 / self.rate
        self.timer = self.create_timer(self.timer_period, self._publish_imu_msg)
      
    def _publish_imu_msg(self):
        
        imuMsg = Imu()
        imuMsg.header.stamp =  self.get_clock().now().to_msg()
        imuMsg.header.frame_id = 'imu'                      
        
        imuMsg.orientation.x = float(self.sensor.quaternion[0])
        imuMsg.orientation.y = float(self.sensor.quaternion[1])
        imuMsg.orientation.z = float(self.sensor.quaternion[2])
        imuMsg.orientation.w = float(self.sensor.quaternion[3])

        imuMsg.angular_velocity.x = float(self.sensor.gyro[0])
        imuMsg.angular_velocity.y = float(self.sensor.gyro[1])
        imuMsg.angular_velocity.z = float(self.sensor.gyro[2])

        imuMsg.linear_acceleration.x = float(self.sensor.linear_acceleration[0])
        imuMsg.linear_acceleration.y = float(self.sensor.linear_acceleration[1])
        imuMsg.linear_acceleration.z = float(self.sensor.linear_acceleration[2])

        self.imu_output.publish(imuMsg)
            

def main(args=None):
    rclpy.init(args=args)
    imu_driver = ReadIMU()
    rclpy.spin(imu_driver)
    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()