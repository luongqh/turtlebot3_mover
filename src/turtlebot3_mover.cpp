#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <tf/transform_datatypes.h>

class TurtleBot3Mover {
public:
    TurtleBot3Mover() : ac_("move_base", true), odom_received_(false) {
        // Initialize publisher for /initialpose
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

        // Initialize subscriber for /odom
        odom_sub_ = nh_.subscribe("/odom", 10, &TurtleBot3Mover::odomCallback, this);

        // Wait for Gazebo to be ready (check for /odom and /scan)
        ROS_INFO("Waiting for Gazebo to initialize...");
        while (ros::ok() && !ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh_, ros::Duration(5.0))) {
            ROS_WARN("Waiting for /odom topic...");
        }
        while (ros::ok() && !ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh_, ros::Duration(5.0))) {
            ROS_WARN("Waiting for /scan topic...");
        }
        ROS_INFO("Gazebo topics are available!");

        // Wait for odom data to be received
        ros::Rate rate(10); // 10 Hz
        while (ros::ok() && !odom_received_) {
            ROS_INFO("Waiting for odometry data...");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Odometry data received!");

        // Wait for the action server to come up
        ROS_INFO("Waiting for move_base action server...");
        ac_.waitForServer();
        ROS_INFO("Connected to move_base action server");

        // Set initial pose using the latest /odom data
        setInitialPose();

        // Define map boundaries (adjust based on TurtleBot3 World)
        x_min_ = -2.0;
        x_max_ = 2.0;
        y_min_ = -2.0;
        y_max_ = 2.0;

        // Start moving
        moveRobot();
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    ros::Publisher initial_pose_pub_;
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry latest_odom_;
    bool odom_received_;
    double x_min_, x_max_, y_min_, y_max_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Store the latest odometry data
        latest_odom_ = *msg;
        odom_received_ = true;
        ROS_INFO("Received odometry: x=%.3f, y=%.3f", 
                 latest_odom_.pose.pose.position.x, 
                 latest_odom_.pose.pose.position.y);
    }

    void setInitialPose() {
        // Construct the initial pose message using /odom data
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.header.frame_id = "map";  // Must be in the map frame

        // Get position from /odom
        double x = latest_odom_.pose.pose.position.x;
        double y = latest_odom_.pose.pose.position.y;
        double z = 0.0;  // Force z to 0 for 2D navigation

        // Get orientation (quaternion) and convert to yaw
        tf::Quaternion quat(
            latest_odom_.pose.pose.orientation.x,
            latest_odom_.pose.pose.orientation.y,
            latest_odom_.pose.pose.orientation.z,
            latest_odom_.pose.pose.orientation.w
        );

        // Set initial position
        initial_pose.pose.pose.position.x = x;
        initial_pose.pose.pose.position.y = y;
        initial_pose.pose.pose.position.z = z;
        initial_pose.pose.pose.orientation.x = latest_odom_.pose.pose.orientation.x;
        initial_pose.pose.pose.orientation.y = latest_odom_.pose.pose.orientation.y;
        initial_pose.pose.pose.orientation.z = latest_odom_.pose.pose.orientation.z;
        initial_pose.pose.pose.orientation.w = latest_odom_.pose.pose.orientation.w;

        // Set covariance (indicating confidence in the initial pose)
        initial_pose.pose.covariance[0] = 0.25;   // x variance
        initial_pose.pose.covariance[7] = 0.25;   // y variance
        initial_pose.pose.covariance[35] = 0.068; // yaw variance (0.068 rad^2 ~ 4 degrees)

        // Publish the initial pose
        ROS_INFO("Setting initial pose from /odom: x=%.3f, y=%.3f, yaw=%.3f rad", 
                 initial_pose.pose.pose.position.x, 
                 initial_pose.pose.pose.position.y, yaw);
        initial_pose_pub_.publish(initial_pose);

        // Give AMCL time to process the initial pose
        ros::Duration(2.0).sleep();
    }

    move_base_msgs::MoveBaseGoal generateRandomGoal() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(x_min_, x_max_);
        std::uniform_real_distribution<> dis_y(y_min_, y_max_);

        double x = dis_x(gen);
        double y = dis_y(gen);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        return goal;
    }

    void moveToGoal(const move_base_msgs::MoveBaseGoal& goal) {
        ROS_INFO("Sending goal: x=%.2f, y=%.2f", 
                 goal.target_pose.pose.position.x, 
                 goal.target_pose.pose.position.y);
        ac_.sendGoal(goal);

        bool finished = ac_.waitForResult(ros::Duration(60.0));
        if (finished) {
            actionlib::SimpleClientGoalState state = ac_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal reached successfully!");
            } else {
                ROS_WARN("Failed to reach goal: %s", state.toString().c_str());
            }
        } else {
            ROS_WARN("Action server timeout. Canceling goal.");
            ac_.cancelGoal();
        }
    }

    void moveRobot() {
        ros::Rate rate(0.2);  // Send a new goal every 5 seconds
        while (ros::ok()) {
            move_base_msgs::MoveBaseGoal goal = generateRandomGoal();
            moveToGoal(goal);
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_mover");
    TurtleBot3Mover mover;
    ros::spin();
    return 0;
}