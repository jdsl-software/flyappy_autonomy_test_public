#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
    nh.getParam("/flyappy_autonomy_code/control_type", control_type_);
    ROS_INFO_STREAM("Running game autonomously with " << control_type_ << " control.");
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;
    auto acc = flyappy_.getAcceleration(msg->x, msg->y);
    acc_cmd.x = acc.x;
    acc_cmd.y = acc.y;
    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Example of printing laser angle and range
    // ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
    if (control_type_ == "collision_avoidance")
    {
        flyappy_.collisionAvoidance(msg->ranges, msg->angle_min, msg->angle_increment);
    }
    else
    {
        flyappy_.magnetDrive(msg->ranges, msg->angle_min, msg->angle_increment);
    }
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    flyappy_.reset();
}
