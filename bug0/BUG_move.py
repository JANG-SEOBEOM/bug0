import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations as transform

class Bug0Publisher(Node):
    state = "go_straight"
    rotate_state = "turn"
    goal_position_x = 1.7
    goal_position_y = 1

    def __init__(self):
        super().__init__('bug0_publisher')
        self.pose_ = None
        self.location_sub_ = self.create_subscription(Odometry, '/odom', self.location_callback, 10) 
        self.location_sub_
        self.subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription_
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        if self.pose_ is None:
            self.get_logger().info("Waiting for initial pose...")
            return
        move = Twist()
        if self.state == "go_straight":
            self.Go_straight(move)
        elif self.state ==  "follow_wall":
            self.Follow_wall(move)
        elif self.state == "finish":
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.get_logger().info("Goal reached!")

        self.get_logger().info(f"state = {self.state}, rotate_state = {self.rotate_state}  {self.pose_.x}  {self.pose_.y} ")
        self.get_logger().info(f"Goal distance: {self.goal_distance()}")         
        self.publisher_.publish(move)

    def Go_straight(self, move):
        angle_to_goal = math.atan2(self.goal_position_y - self.pose_.y, self.goal_position_x - self.pose_.x)
        angle_error = angle_to_goal - self.yaw
        move.linear.x = 0.2 if abs(angle_error) < 0.1 else 0.0
        move.linear.y = 0.0
        move.linear.z = 0.0
        move.angular.x = 0.0
        move.angular.y = 0.0
        move.angular.z = 0.5 * angle_error
    
    def Follow_wall(self, move):
        if self.rotate_state == "turn":
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.linear.z = 0.0
            move.angular.x = 0.0
            move.angular.y = 0.0
            move.angular.z = 0.2
            if self.max_range_index(self.scan_data_,-45,45) >= 0.4:
                self.rotate_state = "go_straight"
            else:
                self.rotate_state = "turn"
        
        elif self.rotate_state == "go_straight":
            move.linear.x = 0.2
            move.linear.y = 0.0
            move.linear.z = 0.0
            move.angular.x = 0.0
            move.angular.y = 0.0
            move.angular.z = 0.0
            if self.max_range_index(self.scan_data_,270,330) < 0.4:
                self.rotate_state = "turn"
            else:
                self.rotate_state = "go_straight"
        
        else:
            self.get_logger().error("Unknown rotate_state: %s" % self.rotate_state)
        
    def location_callback(self, location):
        self.pose_ = location.pose.pose.position
        quarternion = (
                            location.pose.pose.orientation.x,
                            location.pose.pose.orientation.y,
                            location.pose.pose.orientation.z,
                            location.pose.pose.orientation.w)
        self.yaw = transform.euler_from_quaternion(quarternion)[2]

    def scan_callback(self, scan_data):
        self.scan_data_ = scan_data.ranges
        min = self.max_range_index(self.scan_data_, -45, 45)

        if self.goal_distance() < 0.05:
            self.state = "finish"

        if self.state == "go_straight":
            if min < 0.4:
                if self.goal_distance() < 0.05:
                    self.state = "finish"
                else:
                    self.state = "follow_wall"
            else:
                self.state = "go_straight"            
        elif self.state == "follow_wall":
            if self.goal_distance() < 0.05:
                self.state = "finish"
            else:
                if self.should_leave_wall() == False:
                    if self.goal_distance() < 0.05:
                        self.state = "finish"
                    else:
                        self.state = "follow_wall"
                else:
                    self.state = "go_straight"

    def goal_distance(self):
        distance = math.sqrt((self.goal_position_x - self.pose_.x)**2 +(self.goal_position_y - self.pose_.y)**2)
        self.get_logger().info(f"Calculated Goal Distance: {distance}")
        return distance

    def max_range_index(self, ranges, i, j):
        n = len(ranges)
        i = (i + n) % n
        j = (j + n) % n
        if i <= j:
            slice_of_array = ranges[i:j+1]
        else:  
            slice_of_array = ranges[i:] + ranges[:j+1]
        valid = [x for x in slice_of_array if not math.isnan(x) and not math.isinf(x)]
        return min(valid) if valid else float('inf')

    def should_leave_wall(self):
        if (self.max_range_index(self.scan_data_,self.direction_to_goal()-30, self.direction_to_goal()+30)) < 0.4:
            return False
        else:
            self.rotate_state = "turn"
            return True   
              
    def direction_to_goal(self):
        angle_to_goal = math.atan2(self.goal_position_y - self.pose_.y, self.goal_position_x - self.pose_.x)
        direction = (angle_to_goal - self.yaw ) * 180.0 / math.pi
        return int(direction % 360)

def main(args=None):
    rclpy.init(args=args)

    bug0_publisher = Bug0Publisher()

    rclpy.spin(bug0_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bug0_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()