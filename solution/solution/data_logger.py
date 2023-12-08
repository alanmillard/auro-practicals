import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import ItemLog

class DataLogger(Node):

    def __init__(self, args):
        super().__init__('data_logger')

        parser = argparse.ArgumentParser()
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument('--filename', type=str, metavar='FILENAME', help='Filename')
        self.args = parser.parse_args(args[1:])

        self.counter = 0
        self.log_file = open(self.args.filename, 'w')

        self.log_file.write('counter,')
        self.log_file.write('red_count,green_count,blue_count,total_count,')
        self.log_file.write('red_value,green_value,blue_value,total_value\n')
        self.log_file.flush()        

        self.item_log_subscriber = self.create_subscription(
            ItemLog,
            '/item_log',
            self.item_log_callback,
            10)


    def item_log_callback(self, msg):
        self.log_file.write(f'{self.counter},')
        self.log_file.write(f'{msg.red_count},{msg.green_count},{msg.blue_count},{msg.total_count},')
        self.log_file.write(f'{msg.red_value},{msg.green_value},{msg.blue_value},{msg.total_value}\n')
        self.log_file.flush()
        self.counter += 1
        

    def destroy_node(self):

        self.log_file.close()
        super().destroy_node()


def main(args=sys.argv):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = DataLogger(args_without_ros)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()