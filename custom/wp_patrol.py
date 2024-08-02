import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped,Quaternion
from std_msgs.msg import String , Header

from geometry_msgs.msg import Vector3
from mavros_msgs.msg import Waypoint , State
from mavros_msgs.srv import CommandBool , CommandTOLLocal , SetMode,WaypointPush
import time
import math
import numpy as np
class Setpointer(Node):

    def __init__(self):
        super().__init__('set_goal')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        #------- pub -------------#
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/uas_2/setpoint_position/local', qos_profile)
        #Initialize target position
        self.target_pos = None
        self.position_index = 0
        # Define positions as a list of tuples
        self.positions = [
            (0.0, 0.0, 10.0),
            (10.0, 0.0, 10.0),
            (10.0, 10.0, 10.0),
            (0.0, 10.0, 10.0),
            (0.0, 0.0, 10.0)
        ]
        # Timer for publishing positions
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ------- sub -------------#
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/uas_2/local_position/pose',
            self.listener_callback,
            qos_profile)
        self.local_pos=PoseStamped()
        
        # ------- client -------------#
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/uas_2/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        # self.set_mode_srv = self.create_client(CommandBool, '/mavros/uas_2/cmd/arming',qos_profile=qos_profile)
        self.set_arm = self.create_client(CommandBool, '/mavros/uas_2/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # self.set_takeoff = self.create_client(CommandTOLLocal, '/mavros/uas_2/cmd/takeoff_local')
        # while not self.set_takeoff.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')

        # self.set_land = self.create_client(CommandTOLLocal, '/mavros/uas_2/cmd/land_local')
        # while not self.set_land.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')

        self.set_way = self.create_client(WaypointPush, '/mavros/uas_2/mission/push')
        while not self.set_way.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        

        # ----------Main-------------#
        self.Main()
    
    def set_mode(self, custom_mode):
        request = SetMode.Request()
        request.custom_mode = custom_mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.mode_sent:
            self.get_logger().info(f'Mode changed to {custom_mode} successfully.')
        else:
            self.get_logger().error(f'Failed to set mode to {custom_mode}.')

    def arm_call(self, val):
        self.req = CommandBool.Request()
        self.req.value = val
        self.get_logger().info('arm: "%s"' % val)
        self.future = self.set_arm.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        arm_resp = self.future.result()
        if arm_resp is not None:
            self.get_logger().info('Arming response: %s' % arm_resp.success)
        else:
            self.get_logger().error('Failed to receive arm response')
        return arm_resp
    
    # def send_request(self):
    #     self.taskoff_req = CommandTOLLocal.Request()
    #     self.taskoff_req.min_pitch = 0.0
    #     self.taskoff_req.offset = 0.0
    #     self.taskoff_req.rate = 1.0
    #     self.taskoff_req.yaw = 0.0
    #     self.taskoff_req.position = Vector3(x=0.0, y=0.0, z=5.0)
        
    #     self.future = self.set_takeoff.call_async(self.taskoff_req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     response = self.future.result()
    #     if response is not None:
    #         self.get_logger().info('Takeoff response: %s' % response.success)
    #         self.get_logger().info('Takeoff result: %s' % response.result)
    #     else:
    #         self.get_logger().error('No response from takeoff call')
    #     return response
    # def land_request(self):
    #     self.land_req = CommandTOLLocal.Request()
    #     self.land_req.min_pitch = 0.0
    #     self.land_req.offset = 0.0
    #     self.land_req.rate = 1.0
    #     self.land_req.yaw = 0.0
    #     self.land_req.position = Vector3(x=0.0, y=0.0, z=0.0)
        
    #     self.future = self.set_land.call_async(self.land_req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     response = self.future.result()
    #     if response is not None:
    #         self.get_logger().info('Takeoff response: %s' % response.success)
    #         self.get_logger().info('Takeoff result: %s' % response.result)
    #     else:
    #         self.get_logger().error('No response from takeoff call')
    #     return response
    def set_way_func(self):
        wp = Waypoint()
        wp.frame = 3 # Global frame wth rel alt
        wp.command = 16 # Navigate to WayPoints
        wp.is_current = True
        wp.autocontinue = True
        wp.param1 = 0.0 # Hover in sec
        wp.param2 = 0.0 # Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
        wp.param3 = 0.0 # radius to consider passing through waypoint
        wp.param4 = float('nan') # Yaw angle at waypoint
        wp.x_lat = 2.909484
        wp.y_long = 101.655246
        wp.z_alt = 5.0
        self.setWayPoint(0, [wp])
    def setWayPoint(self, index, waypoints):
        print('Set WayPoints \n')
        self.way_req = WaypointPush.Request()
        self.way_req.start_index = index
        self.way_req.waypoints = waypoints
        future = self.set_way.call_async(self.way_req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info('setWayPoint response: %s' % response.success)
            self.get_logger().info('setWayPoint result: %s' % response.wp_transfered)
        else:
            self.get_logger().error('No response from setWayPoint call')
        self.set_mode('AUTO.MISSION')
        return response
    def set_next_target(self):
        if self.position_index < len(self.positions):
            self.target_pos = self.positions[self.position_index]
            self.get_logger().info(f'Setting next target position: {self.target_pos}')
            self.position_index += 1
        else:
            self.get_logger().info('All target positions reached.')
            self.target_pos = None

    def timer_callback(self):
        # for OFFBOARD MODE, we must send to position order before mode change
        if self.target_pos is None:
            # 어차피 모드가 달라서 아무 좌표나 보내도 안들어간다. 초기화라 생각!
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 5.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0 
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
            return
        
        x, y, z = self.target_pos
        if not self.is_at_position(x, y, z, 0.1):
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0 
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Reached target position.')
            self.set_next_target()

    def listener_callback(self, msg):
        self.local_pos = PoseStamped()
        self.local_pos.pose.position.x=msg.pose.position.x
        self.local_pos.pose.position.y=msg.pose.position.y
        self.local_pos.pose.position.z=msg.pose.position.z

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        desired = np.array((x, y, z))
        pos = np.array((self.local_pos.pose.position.x,
                        self.local_pos.pose.position.y,
                        self.local_pos.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def target_position(self,x,y,z):
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.get_logger().info(f'Setting target position: ({x}, {y}, {z})')
        while not self.is_at_position(x, y, z, 0.1):
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
            
        self.get_logger().info('Reached target position.')

    def Main(self):
        self.arm_call(True)
    
        time.sleep(4)
        self.set_mode('AUTO.TAKEOFF')
        time.sleep(5)
        self.set_mode('OFFBOARD')
        time.sleep(1)
        self.set_next_target()
        # self.set_way_func()
def main(args=None):
    rclpy.init(args=args)
    set_goal = Setpointer()
    try:
        rclpy.spin(set_goal)
    except KeyboardInterrupt:
        set_goal.set_mode('AUTO.LAND')
        pass
    finally:
        # Cleanup code
        set_goal.destroy_node()
        rclpy.shutdown()
    
   
    

if __name__ == '__main__':
    main()
