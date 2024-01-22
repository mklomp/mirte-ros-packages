from telemetrix_rpi_pico import telemetrix_rpi_pico
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
        try:
           self.board = telemetrix_rpi_pico.TelemetrixRpiPico()
        except:
           print("no connection, shuttign down")
           exit(0)

        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2 %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def shutdown(self):
        self.board.shutdown()



def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    try:
      rclpy.spin(node)
    except:
      print("done")
      #node.shutdown()
    finally:
      node.shutdown()
      node.destroy_node()


if __name__ == '__main__':
    main()
