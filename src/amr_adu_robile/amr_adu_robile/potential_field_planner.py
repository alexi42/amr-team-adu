#!usr/bin/env python3 

import rclpy
import math 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class PotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')

        self.get_logger().info("Potential Field Planner Started")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.scan_msg = None

        self.goal_x = 5.0
        self.goal_y = -2.0
        self.goal_theta = -1.0
        self.avoid_mode = False

        self.timer = self.create_timer(
           0.1,
           self.control_loop
        )       

        

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
    
    def normalize_angle(self, angle):
        
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if self.scan_msg is None:
            return
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance_to_goal = math.sqrt(dx**2 + dy**2)
        
        k_att = 0.5
        k_rep = 0.25
        rho_0 = 1.3

        v_att_x = k_att * dx / distance_to_goal
        v_att_y = k_att * dy / distance_to_goal
        self.get_logger().info(f"Attractive: ({v_att_x:.2f}, {v_att_y:.2f})")
        

        v_rep_x = 0.0
        v_rep_y = 0.0

        angle = self.scan_msg.angle_min

        for r in self.scan_msg.ranges:
            if math.isnan(r) or math.isinf(r):
                angle += self.scan_msg.angle_increment
                continue

            if r < self.scan_msg.range_min or r > self.scan_msg.range_max:
                angle += self.scan_msg.angle_increment
                continue

            if r < rho_0:
                strength = k_rep * ((1.0 / r) - (1.0 / rho_0)) * (1.0 / (r * r))
                obs_x = math.cos(angle)
                obs_y = math.sin(angle)

                v_rep_x += -strength * obs_x
                v_rep_y += -strength * obs_y
            angle += self.scan_msg.angle_increment 

        
        v_total_x = v_att_x + v_rep_x
        v_total_y = v_att_y + v_rep_y

        self.get_logger().info(
            f"att=({v_att_x:.2f},{v_att_y:.2f}) "
            f"rep=({v_rep_x:.2f},{v_rep_y:.2f}) "
            f"total=({v_total_x:.2f},{v_total_y:.2f})"
        )

        target_angle = math.atan2(v_total_y, v_total_x)

        angle_error = self.normalize_angle(target_angle - self.theta)
        
        cmd = Twist()

        if distance_to_goal < 0.3:
            final_error = self.normalize_angle(self.goal_theta - self.theta)

            if abs(final_error) > 0.08:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6 * final_error
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("GOAL POSITION AND ORIENTATION REACHED!")

            self.cmd_pub.publish(cmd)
            return
        
        # Safety / side selection
        valid_ranges = [
            r for r in self.scan_msg.ranges
            if not math.isnan(r)
            and not math.isinf(r)
            and r >= self.scan_msg.range_min
            and r <= self.scan_msg.range_max
        ]

        if len(valid_ranges) == 0:
            self.cmd_pub.publish(cmd)
            return

        min_range = min(valid_ranges)

        front_ranges = []
        left_ranges = []
        right_ranges = []

        angle = self.scan_msg.angle_min

        for r in self.scan_msg.ranges:
            if math.isnan(r) or math.isinf(r):
                angle += self.scan_msg.angle_increment
                continue

            if r < self.scan_msg.range_min or r > self.scan_msg.range_max:
                angle += self.scan_msg.angle_increment
                continue

            if -0.52 < angle < 0.52:
                front_ranges.append(r)

            elif 0.52 <= angle < 1.57:
                left_ranges.append(r)

            elif -1.57 < angle <= -0.52:
                right_ranges.append(r)

            angle += self.scan_msg.angle_increment

        front_min = min(front_ranges) if front_ranges else 10.0
        left_avg = sum(left_ranges) / len(left_ranges) if left_ranges else 10.0
        right_avg = sum(right_ranges) / len(right_ranges) if right_ranges else 10.0

        self.get_logger().info(
            f"front_min={front_min:.2f}, left_avg={left_avg:.2f}, right_avg={right_avg:.2f}, avoid={self.avoid_mode}"
)
     
        # Start avoid mode when obstacle is in front
        if front_min < 1.2:
            self.avoid_mode = True

        # Leave avoid mode earlier when front is reasonably free
        if self.avoid_mode and front_min > 1.6:
            self.avoid_mode = False

        if self.avoid_mode:
            # Move around obstacle, not just rotate
            cmd.linear.x = 0.15
            cmd.angular.z = 0.45
        else:
            cmd.linear.x = min(0.38, 0.25 * distance_to_goal)
            cmd.angular.z = 0.8 * angle_error

        self.get_logger().info(
            f"front_min={front_min:.2f}, avoid={self.avoid_mode}, "
            f"cmd_x={cmd.linear.x:.2f}, cmd_z={cmd.angular.z:.2f}"
)
        self.cmd_pub.publish(cmd)
        

            
    def scan_callback(self, msg):
        self.scan_msg = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.theta =math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)

    node = PotentialFieldPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()