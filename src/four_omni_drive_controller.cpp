#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

namespace tsuten_simulation
{
  class FourOmniDriveController
  {
  public:
    FourOmniDriveController()
    {
      ros::NodeHandle nh, pnh("~");

      pnh.param("wheel_radius", wheel_radius_, 30.6e-3);

      wheel_vels_pub_ = nh.advertise<std_msgs::Float64MultiArray>("wheel_vels", 10);

      cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &FourOmniDriveController::cmdVelCallback, this);
    }

  private:
    void cmdVelCallback(const geometry_msgs::Twist &cmd_vel)
    {
      std_msgs::Float64MultiArray wheel_vels;
      wheel_vels.data.resize(4);

      for (size_t i = 0; i < 4; i++)
      {
        double theta = M_PI / 4 + i * M_PI / 2;
        wheel_vels.data.at(i) =
            (-cmd_vel.linear.x * std::sin(theta) + cmd_vel.linear.y * std::cos(theta)) /
                wheel_radius_ +
            cmd_vel.angular.z;
      }

      wheel_vels_pub_.publish(wheel_vels);
    }

    ros::Publisher wheel_vels_pub_;

    ros::Subscriber cmd_vel_sub_;

    double wheel_radius_;
  };
} // namespace tsuten_simulation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_omni_drive_controller");

  tsuten_simulation::FourOmniDriveController four_omni_drive_controller;

  ros::spin();

  return 0;
}