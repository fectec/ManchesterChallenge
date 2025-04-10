#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import json
from custom_interfaces.msg import OpenLoopPose
from geometry_msgs.msg import Quaternion

def normalize_angle_diff(current_angle, goal_angle):
    diff = goal_angle - current_angle
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return diff

def angle_diff_signed(sign, diff):
    return diff if sign>0 else -diff

def yaw_to_quaternion(angle):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(angle / 2.0)
    q.w = math.cos(angle / 2.0)
    return q

class OpenLoopPathGenerator(Node):
    def __init__(self):
        super().__init__('open_loop_path_generator')
        self.pose_pub = self.create_publisher(OpenLoopPose, 'pose', 10)

        self.declare_parameter('waypoints_json', '[]')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('safety_margin', 0.1)

        raw_json = self.get_parameter('waypoints_json').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.margin = self.get_parameter('safety_margin').value

        try:
            self.waypoints = json.loads(raw_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            self.waypoints = None

        if not isinstance(self.waypoints, list):
            self.get_logger().error("Invalid waypoints_json: must be a JSON array.")
            self.destroy_node()
            return
        
        self.xcur = 0.0
        self.ycur = 0.0
        self.thcur = 0.0
        self.index = 0
        self.timer = self.create_timer(1.0, self.process_waypoint)

    def process_waypoint(self):
        if not self.waypoints or self.index >= len(self.waypoints):
            self.get_logger().info("No more waypoints or invalid input. Stopping.")
            self.timer.cancel()
            return

        wp = self.waypoints[self.index]
        x_target = float(wp.get('x', 0.0))
        y_target = float(wp.get('y', 0.0))

        dx = x_target - self.xcur
        dy = y_target - self.ycur
        distance = math.sqrt(dx*dx + dy*dy)

        raw_goal_angle = math.atan2(dy, dx)
        angle_diff_signed = normalize_angle_diff(self.thcur, raw_goal_angle)
        angle_diff = abs(angle_diff_signed)
        sign = 1.0 if angle_diff_signed >= 0 else -1.0

        if 'total_time' in wp and 'rot_speed' not in wp and 'lin_speed' not in wp:
            total_time = float(wp['total_time'])
            
            time_for_rotation_at_max = angle_diff / self.max_ang if self.max_ang > 0 else 0
            time_for_translation_at_max = distance / self.max_lin if self.max_lin > 0 else 0

            raw_time_sum = time_for_rotation_at_max + time_for_translation_at_max
            scaled_time_sum = raw_time_sum * (1.0 + self.margin)

            if scaled_time_sum > total_time:
                self.get_logger().error("Impossible speeds for time mode (scaled sum exceeds total_time).")
                self.stop()
                return
            
            scale_factor = total_time / scaled_time_sum if scaled_time_sum > 0 else 1.0
            effective_rotation_speed = self.max_ang * scale_factor
            effective_translation_speed = self.max_lin * scale_factor

            if effective_rotation_speed > self.max_ang:
                effective_rotation_speed = self.max_ang
            if effective_translation_speed > self.max_lin:
                effective_translation_speed = self.max_lin

            rotation_time = angle_diff / effective_rotation_speed if effective_rotation_speed > 0 else 0
            translation_time = distance / effective_translation_speed if effective_translation_speed > 0 else 0

            if (rotation_time + translation_time) > total_time:
                self.get_logger().error("Even clamped speeds exceed total_time.")
                self.stop()
                return
            
            self.publish_subcommands(
                effective_rotation_speed, 
                effective_translation_speed, 
                angle_diff, 
                distance, 
                rotation_time, 
                translation_time, 
                raw_goal_angle, 
                x_target, 
                y_target, 
                sign
            )
            
        elif 'rot_speed' in wp and 'lin_speed' in wp and 'total_time' not in wp:
            rs = float(wp['rot_speed'])
            ls = float(wp['lin_speed'])
            if rs > self.max_ang or ls > self.max_lin:
                self.get_logger().error("User-defined speeds exceed puzzlebot limits.")
                self.stop()
                return
            rotation_time = angle_diff / rs if rs>0 else 0
            translation_time = distance / ls if ls>0 else 0

            self.publish_subcommands(
                rs, 
                ls, 
                angle_diff, 
                distance, 
                rotation_time, 
                translation_time, 
                raw_goal_angle, 
                x_target, 
                y_target, 
                sign
            )
        else:
            self.get_logger().error("Waypoint must define either total_time OR rot_speed+lin_speed.")
            self.stop()
            return

        self.index += 1

    def publish_subcommands(
        self, rot_spd, lin_spd, angle_diff, dist, rot_time, lin_time, final_angle, x_tar, y_tar, sign
    ):
        if angle_diff > 1e-6 and rot_time > 1e-6:
            msg_rot = OpenLoopPose()
            msg_rot.pose.position.x = self.xcur
            msg_rot.pose.position.y = self.ycur
            msg_rot.pose.orientation = yaw_to_quaternion(self.thcur)
            msg_rot.linear_velocity = 0.0
            msg_rot.angular_velocity = sign * rot_spd
            msg_rot.execution_time = rot_time
            self.pose_pub.publish(msg_rot)

        if dist > 1e-6 and lin_time > 1e-6:
            msg_trans = OpenLoopPose()
            msg_trans.pose.position.x = self.xcur
            msg_trans.pose.position.y = self.ycur
            msg_trans.pose.orientation = yaw_to_quaternion(self.thcur + angle_diff_signed(sign, angle_diff))
            msg_trans.linear_velocity = lin_spd
            msg_trans.angular_velocity = 0.0
            msg_trans.execution_time = lin_time
            self.pose_pub.publish(msg_trans)

        self.xcur = x_tar
        self.ycur = y_tar
        raw_diff = angle_diff if (sign > 0) else -angle_diff
        self.thcur = (self.thcur + raw_diff + math.pi) % (2 * math.pi) - math.pi

    def stop(self):
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPathGenerator()
    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()