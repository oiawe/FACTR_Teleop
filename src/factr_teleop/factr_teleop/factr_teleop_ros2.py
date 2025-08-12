from factr_teleop.factr_teleop import FACTRTeleop
from sensor_msgs.msg import JointState

import rclpy
import numpy as np

class Factr_Ros_node(FACTRTeleop):

    def __init__(self):
        super().__init__()
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.follower_external_torque = np.zeros(7)
        self.follower_velocity = np.zeros(7)
        self.follower_gripper_torque = 0.0

        self.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

    def set_up_communication(self):
        # initial joint and gripper command publisher for follower arm
        self.leader_joint_pos_pub = self.create_publisher(JointState, '/factr_teleop/joint_cmd', 10)
        self.leader_gripper_pos_pub = self.create_publisher(JointState, '/factr_teleop/gripper_cmd', 10)
        self.leader_joint_vel_pub = self.create_publisher(JointState, '/factr_teleop/joint_velocity', 10)
        
        self.follower_joint_vel_sub = self.create_subscription(
            JointState,
            '/franka_robot_state_broadcaster/measured_joint_states',
            self.follower_vel_callback,
            1
        )

        if self.enable_torque_feedback:
            # initial external torque subscriber from follower arm
            self.follower_torque_sub = self.create_subscription(
                JointState,
                '/franka_robot_state_broadcaster/external_joint_torques',
                self.follower_torque_callback,
                1
            )

        if self.enable_gripper_feedback:
            # initial gripper torque subsriber from follower arm
            self.gripper_pub = self.create_subscription(
                JointState, 
                '/franka_state_controller/gripper_torque', 
                self.gripper_torque_callback, 
                10
        )

    def follower_vel_callback(self,msg):
        self.follower_velocity = np.array(msg.velocity)

    def follower_torque_callback(self, msg):
        self.follower_external_torque = np.array(msg.effort)

    def get_leader_arm_external_joint_torque(self):
        return self.follower_external_torque

    def get_follower_velocity(self):
        return self.follower_velocity

    def gripper_torque_callback(self, msg):
        gripper_torque = np.array(msg.effort)
        self.follower_gripper_torque = gripper_torque[0]

    def get_leader_gripper_feedback(self):
        return self.follower_gripper_torque
    
    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0*gripper_feedback / self.gripper_feedback_gain
        return torque_gripper

    def update_communication(self, leader_arm_pos, leader_gripper_pos, leader_arm_vel, leader_arm_torque):

        # publish joint position
        leader_joint_states = JointState()
        leader_joint_states.header.stamp = self.get_clock().now().to_msg()
        leader_joint_states.name = self.joint_names
        leader_joint_states.header.frame_id = "fr3_base_link"
        leader_joint_states.position = list(map(float, leader_arm_pos))
        self.leader_joint_pos_pub.publish(leader_joint_states)

        # publish gripper state
        leader_gripper_states = JointState()
        leader_gripper_states.header.stamp = self.get_clock().now().to_msg()
        leader_gripper_states.name = ["gripper"]
        leader_gripper_states.position = list(map(float, [leader_gripper_pos]))
        self.leader_gripper_pos_pub.publish(leader_gripper_states)

        # publish joint velocity
        leader_vel_state = JointState()
        leader_vel_state.header.stamp = self.get_clock().now().to_msg()
        leader_vel_state.name = self.joint_names
        leader_vel_state.velocity = list(map(float, leader_arm_vel))
        leader_vel_state.effort = list(map(float, leader_arm_torque))
        self.leader_joint_vel_pub.publish(leader_vel_state)
        
    
def main(args=None):
    rclpy.init(args=args)

    factr_node = Factr_Ros_node()

    try:
        while rclpy.ok():
            rclpy.spin(factr_node)
    except KeyboardInterrupt:
        factr_node.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_node.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
