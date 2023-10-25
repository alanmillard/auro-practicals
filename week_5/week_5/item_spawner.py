import os
import sys
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, GetModelList, GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Twist


class ItemSpawner(Node):

    def __init__(self):
        super().__init__('item_spawner')

        self.first_run = True
        self.spawn_new_item = True

        models_path = os.path.join(get_package_share_directory("week_5"), "models")
        item_model_path = os.path.join(models_path, "item", "model.sdf")
        self.item_model = open(item_model_path, 'r').read()

        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity', callback_group=client_callback_group)
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list', callback_group=client_callback_group)
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=client_callback_group)
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=client_callback_group)

        self.timer = self.create_timer(0.1, self.control_loop, callback_group=timer_callback_group)


    def generate_item_pose(self):
        pose = Pose()
        pose.position.x = random.uniform(-2.0, 2.0)
        pose.position.y = random.uniform(-2.0, 2.0)
        self.get_logger().info(f"Generated item position: ({pose.position.x:.2f}, {pose.position.y:.2f})")        
        return pose


    def spawn_item(self):
        request = SpawnEntity.Request()
        request.name = "item"
        request.xml = self.item_model
        request.initial_pose = self.generate_item_pose()
        request.reference_frame = "world"

        self.get_logger().info("Spawning entity")
        return self.spawn_entity_client.call(request)


    def get_model_list(self):
        request = GetModelList.Request()
        return self.get_model_list_client.call(request)


    def get_entity_state(self, name):
        request = GetEntityState.Request()
        request.name = name
        return self.get_entity_state_client.call(request)
    

    def set_entity_state(self, name, pose):
        state = EntityState()
        state.name = name
        state.pose = pose
        state.twist = Twist()
        state.reference_frame = "world"

        request = SetEntityState.Request()
        request.state = state

        self.get_logger().info("Setting entity state")
        return self.set_entity_state_client.call(request)
    

    def control_loop(self):

        if self.first_run:
            self.first_run = False
            self.spawn_item()            

        model_list_msg = self.get_model_list()
        entity_state_msg = self.get_entity_state("waffle_pi")
        robot_position = entity_state_msg.state.pose.position

        if "item" in model_list_msg.model_names:

            entity_state_msg = self.get_entity_state("item")
            item_position = entity_state_msg.state.pose.position

            distance = math.sqrt((robot_position.x - item_position.x) ** 2 +
                                 (robot_position.y - item_position.y) ** 2)

            if distance < 0.1: # Radius of item
                self.get_logger().info("Collected item")
                self.set_entity_state("item", self.generate_item_pose())


def main(args=None):

    rclpy.init(args = args)

    node = ItemSpawner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt: # TODO: Why does this take 2x Ctrl+c to kill?
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()