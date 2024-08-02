import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String , Header

from geometry_msgs.msg import Vector3
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import CommandBool , CommandTOLLocal , SetMode
import time
class Setpointer(Node):

    def __init__(self):
        super().__init__('set_goal')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        #------- pub -------------#
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/uas_2/setpoint_position/local', qos_profile)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # ------- client -------------#
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/uas_2/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        # self.set_mode_srv = self.create_client(CommandBool, '/mavros/uas_2/cmd/arming',qos_profile=qos_profile)
        self.set_arm = self.create_client(CommandBool, '/mavros/uas_2/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
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
    
    def timer_callback(self):
        # Prepare the PoseStamped message
        self.msg = PoseStamped()
        self.msg.header = Header()
        self.msg.header.frame_id = 'map'
        self.msg.pose.position.x = 0.0
        self.msg.pose.position.y = 0.0
        self.msg.pose.position.z = 5.0
        self.msg.pose.orientation.x = 0.0
        self.msg.pose.orientation.y = 0.0
        self.msg.pose.orientation.z = 0.0
        self.msg.pose.orientation.w = 1.0
        self.msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
        self.publisher_.publish(self.msg)
        
    
    def Main(self):
        self.arm_call(True)
    
        time.sleep(4)
        self.set_mode('AUTO.TAKEOFF')
        time.sleep(5)
        self.set_mode('OFFBOARD')
        time.sleep(5)
        
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
