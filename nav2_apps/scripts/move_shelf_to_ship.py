#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from attach_shelf.srv import GoToLoading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Polygon,Point32
# Shelf positions for picking
shelf_positions = {
    "loading_position": [5.645, -0.3]
    }

# Shipping destination for picked products
shipping_destinations = {
    "shipping_position": [0.232 , -3.3]
    }

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''




def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'loading_position'
    request_destination = 'shipping_position'
    ####################

    rclpy.init()


    navigator = BasicNavigator()

    #create client
    client_cb_group = MutuallyExclusiveCallbackGroup()
    navigator.cli = navigator.create_client(GoToLoading, '/approach_shelf',callback_group=client_cb_group)
    while not navigator.cli.wait_for_service(timeout_sec=1.0):
        navigator.get_logger().info('service not available, waiting again...')
    navigator.req = GoToLoading.Request()
    #create oublish
    pub_local = navigator.create_publisher(Polygon, '/local_costmap/footprint', 10)
    pub_global = navigator.create_publisher(Polygon, '/global_costmap/footprint', 10)
    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    #initial_pose.pose.position.x = 5.645
    #initial_pose.pose.position.y = -0.3
    #initial_pose.pose.orientation.z = -0.7071067
    #initial_pose.pose.orientation.w = 0.7073883

    initial_pose.pose.position.x = 3.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = -0.7071067
    shelf_item_pose.pose.orientation.w = 0.7073883
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:

        navigator.req.attach_to_shelf = True
        print('Callback services')
        navigator.future = navigator.cli.call_async(navigator.req)
        print('Until')
        rclpy.spin_until_future_complete(navigator, navigator.future)
        print('Until')
        status = navigator.future.result()
        if status != GoToLoading.Response():
            navigator.error('Change map request failed!')
        else:
            navigator.info('Change map request was successful!')

        # publish footprint
        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.5
        point1.y = 0.4

        point2 = Point32()
        point2.x = 0.5
        point2.y = -0.4

        point3 = Point32()
        point3.x = -0.5
        point3.y = -0.4

        point4 = Point32()
        point4.x = -0.5
        point4.y = 0.4

        footprint.points = [point1, point2, point3, point4]
        pub_local.publish(footprint)
        pub_global.publish(footprint)

        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')
        
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        shipping_destination.pose.orientation.z = 0.0
        shipping_destination.pose.orientation.w = 1.0
        navigator.goToPose(shipping_destination)
        

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
    