import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations as transform

class Bug0Publisher(Node):
    # Robot states
    state = "go_straight"
    rotate_state = "turn"

    # Goal position
    GOAL_X = 1.7
    GOAL_Y = 1.0

    # Motion parameters
    LINEAR_SPEED = 0.2
    ANGULAR_SPEED_FOR_GO_STRAIGHT = 0.5
    ANGULAR_SPEED = 0.2

    # Thresholds
    OBSTACLE_DISTANCE = 0.4
    GOAL_TOLERANCE = 0.05
    SENSOR_WIDTH = 45   # angle range for obstacle detection

    def __init__(self):
        super().__init__('bug0_publisher')
        self.pose_ = None

        # Subscribers
        self.location_sub_ = self.create_subscription(Odometry, '/odom', self.location_callback, 10) 
        self.subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Main loop executed at fixed time intervals"""
        if self.pose_ is None:
            self.get_logger().info("Waiting for initial pose...")
            return
        
        move = Twist()

        # Behavior depends on state
        if self.state == "go_straight":
            self.go_straight(move)
        elif self.state ==  "follow_wall":
            self.follow_wall(move)
        elif self.state == "finish":
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.get_logger().info("Goal reached!")

        # Debug logging
        self.get_logger().debug(f"state = {self.state}, rotate_state = {self.rotate_state}  {self.pose_.x}  {self.pose_.y} ")
        self.get_logger().debug(f"Goal distance: {self.goal_distance()}")         
        
        # Publish velocity command
        self.publisher_.publish(move)

    def go_straight(self, move):
        """Move straight toward the goal unless large angular error"""
        angle_to_goal = math.atan2(self.GOAL_Y - self.pose_.y, self.GOAL_X - self.pose_.x)
        angle_error = angle_to_goal - self.yaw
        move.linear.x = self.LINEAR_SPEED if abs(angle_error) < 0.1 else 0.0
        move.linear.y = 0.0
        move.linear.z = 0.0
        move.angular.x = 0.0
        move.angular.y = 0.0
        move.angular.z = self.ANGULAR_SPEED_FOR_GO_STRAIGHT * angle_error
    
    def follow_wall(self, move):
        """Wall-following behavior when obstacle detected"""
        if self.rotate_state == "turn":
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.linear.z = 0.0
            move.angular.x = 0.0
            move.angular.y = 0.0
            move.angular.z = self.LINEAR_SPEED
            if self.max_range_index(self.scan_data_,-1 * self.SENSOR_WIDTH, self.SENSOR_WIDTH) >= self.OBSTACLE_DISTANCE:
                self.rotate_state = "go_straight"
            else:
                self.rotate_state = "turn"
        
        elif self.rotate_state == "go_straight":
            move.linear.x = self.LINEAR_SPEED
            move.linear.y = 0.0
            move.linear.z = 0.0
            move.angular.x = 0.0
            move.angular.y = 0.0
            move.angular.z = 0.0
            if self.max_range_index(self.scan_data_,270,270+ self.SENSOR_WIDTH) < self.OBSTACLE_DISTANCE:
                self.rotate_state = "turn"
            else:
                self.rotate_state = "go_straight"
        
        else:
            self.get_logger().error("Unknown rotate_state: %s" % self.rotate_state)
        
    def location_callback(self, location):
        """Update robot pose and orientation from Odometry"""
        self.pose_ = location.pose.pose.position
        quarternion = (
                            location.pose.pose.orientation.x,
                            location.pose.pose.orientation.y,
                            location.pose.pose.orientation.z,
                            location.pose.pose.orientation.w)
        self.yaw = transform.euler_from_quaternion(quarternion)[2]

    def scan_callback(self, scan_data):
        """Update laser scan data and decide state transitions"""
        self.scan_data_ = scan_data.ranges
        min = self.max_range_index(self.scan_data_, -1 * self.SENSOR_WIDTH, self.SENSOR_WIDTH)

        # Check if goal is reached
        if self.goal_distance() < self.GOAL_TOLERANCE:
            self.state = "finish"

        # State transition logic
        if self.state == "go_straight":
            if min < self.OBSTACLE_DISTANCE:
                self.state = "follow_wall"
            else:
                self.state = "go_straight"            
        elif self.state == "follow_wall":
            if self.should_leave_wall() == False:
                self.state = "follow_wall"
            else:
                self.state = "go_straight"

    def goal_distance(self):
        """Compute Euclidean distance to goal"""
        if self.pose_ is None:
            return float('inf')
        distance = math.sqrt((self.GOAL_X - self.pose_.x)**2 +(self.GOAL_Y - self.pose_.y)**2)
        self.get_logger().debug(f"Calculated Goal Distance: {distance}")
        return distance

    def max_range_index(self, ranges, i, j):
        """Get minimum valid range value between index i and j (handles wrap-around)"""
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
        """Decide whether to stop wall-following and go toward goal"""
        if (self.max_range_index(self.scan_data_,self.direction_to_goal()-self.SENSOR_WIDTH, self.direction_to_goal()+self.SENSOR_WIDTH)) < self.OBSTACLE_DISTANCE:
            return False
        else:
            self.rotate_state = "turn"
            return True   
              
    def direction_to_goal(self):
        """Compute relative angle (in degrees) from robot heading to goal"""
        angle_to_goal = math.atan2(self.GOAL_Y - self.pose_.y, self.GOAL_X - self.pose_.x)
        direction = (angle_to_goal - self.yaw ) * 180.0 / math.pi
        return int(direction % 360)

def main(args=None):
    rclpy.init(args=args)
    bug0_publisher = Bug0Publisher()
    rclpy.spin(bug0_publisher)
    bug0_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()