import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import CommandBool,SetMode
from std_msgs.msg import Header
import argparse
import time
import numpy as np
class UAVAgent(Node):
    def __init__(self, id):
        super().__init__('uav_agent'+id)
        self.get_logger().info(f'UAV Agent started with ID: {id}')
        self.id =id
        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        #------- pub -------------#
        self.topic_base = f'/mavros/uas_{id}'
        self.publisher_ = self.create_publisher(PoseStamped, self.topic_base+'/setpoint_position/local', qos_profile)

        self.target_pos = None
        self.position_index=0
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ------- sub -------------#
        self.subscription = self.create_subscription(
            PoseStamped,
            self.topic_base+'/local_position/pose',
            self.listener_callback,
            qos_profile)
        self.local_pos=PoseStamped()

        # ------- client -------------#
        
        self.set_mode_client = self.create_client(SetMode, self.topic_base+'/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        self.set_arm = self.create_client(CommandBool, self.topic_base+'/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')


        self.Control()

    def Control(self):
        self.arm_call(True)
        time.sleep(4)
        self.set_mode('AUTO.TAKEOFF')
        time.sleep(5)
        self.set_mode('OFFBOARD')
        time.sleep(1)

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

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        desired = np.array((x, y, z))
        pos = np.array((self.local_pos.pose.position.x,
                        self.local_pos.pose.position.y,
                        self.local_pos.pose.position.z))
        
        return np.linalg.norm(desired - pos) < offset   
    
    def listener_callback(self, msg):
        self.local_pos = PoseStamped()
        self.local_pos.pose.position.x=msg.pose.position.x
        self.local_pos.pose.position.y=msg.pose.position.y
        self.local_pos.pose.position.z=msg.pose.position.z
        
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
            self.get_logger().info('init')
            if self.is_at_position(0.0,0.0,5.0,1.0):
                self.get_logger().info('Ready for moving')
                self.wait_start()
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
            self.decision()

    def wait_start(self):
        self.start=True
        self.get_logger().info('start to move ~~')
        #TODO
        #subcribe to sign for moving to target position 
        #time.sleep(4)
        self.decision()
        pass 
    
    def decision(self):
        #TODO
        # Define positions as a list of tuples
        self.total_task = [
            (0.0, 0.0, 3.0*float(self.id)),
            (10.0, 0.0, 3.0*float(self.id)),
            (10.0, 10.0, 3.0*float(self.id)),
            (0.0, 10.0, 3.0*float(self.id)),
            (0.0, 0.0, 3.0*float(self.id))
        ]
        if self.position_index < len(self.total_task):
            self.target_pos = self.total_task[self.position_index]
            self.get_logger().info(f'Setting next target position: {self.target_pos}')
            self.position_index += 1
        else:
            self.get_logger().info('All target positions reached.')
            self.target_pos = None
        pass
        
def main(args=None):
    # Parse custom arguments separately
    parser = argparse.ArgumentParser(description='UAV Agent Node')
    parser.add_argument('--id', type=str, help='Instance ID for the UAV agent')
    parsed_args, unknown_args = parser.parse_known_args()

    # Initialize rclpy with the ROS arguments
    rclpy.init(args=unknown_args)

    agent = UAVAgent(id=parsed_args.id)

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.set_mode('AUTO.LAND')
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

