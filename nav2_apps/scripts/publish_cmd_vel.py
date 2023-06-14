import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', 10)
        self.duration = Duration(seconds=12)
        self.rate1 = self.create_rate(10,self.get_clock())  # 10 Hz, cada 0.1 segundos

    def publish_velocity(self):
        start_time = self.get_clock().now()
        print('Init')
        while rclpy.ok() and (self.get_clock().now() - start_time) < self.duration:
            print('Forward')
            msg = Twist()
            msg.linear.x = 0.1
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            print('Forward')
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()
    velocity_publisher.publish_velocity()

    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
