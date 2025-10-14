#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <mutex>
#include <csignal>
#include <atomic>

#define NETWORK_INTERFACE "eth0"
using namespace unitree::common;

class Custom
{
public:
  Custom(ros::NodeHandle &nh) : should_exit(false)
  {
    // 初始化Unitree运动客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    sport_client.StandUp();
    sleep(1);
    sport_client.BalanceStand();
    sleep(1);
    sport_client.ClassicWalk(true);
    cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &Custom::CmdVelCallback, this);
  }

  ~Custom()
  {
    SafeShutdown();
  }
  void SafeShutdown()
  {
    if (!should_exit.exchange(true)) 
    {
      std::cout << "Initiating safe shutdown..." << std::endl;
      sport_client.StopMove();
      sport_client.StandDown();
      std::cout << "Safety procedures completed." << std::endl;
    }
  }

private:
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    sport_client.Move(msg->linear.x, msg->linear.y, msg->angular.z);
  }
  unitree::robot::go2::SportClient sport_client;

  ros::Subscriber cmd_vel_sub;

  std::atomic<bool> should_exit;
};

Custom* global_custom = nullptr;

void signalHandler(int signum)
{
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  if (global_custom != nullptr) {
    global_custom->SafeShutdown();
  }
  ros::shutdown();
}

int main(int argc, char **argv)
{
  unitree::robot::ChannelFactory::Instance()->Init(0, NETWORK_INTERFACE);
  ros::init(argc, argv, "unitree_cmd_vel_controller");
  ros::NodeHandle nh;

  Custom custom(nh);
  global_custom = &custom;
 
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  ros::spin();
  global_custom = nullptr;
  return 0;
}
