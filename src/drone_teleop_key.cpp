#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <mutex>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class SIGVerseTb3OpenManipulatorGraspingTeleopKey
{
private:
   /*static const char KEY_1 = 0x31;   *these keys no longer have 
  static const char KEY_2 = 0x32;      *functions withing the drone 
  static const char KEY_3 = 0x33;      *controller, but I'm leaving
  static const char KEY_4 = 0x34;      *them here in case further
  static const char KEY_5 = 0x35;      *development is necessary
  static const char KEY_6 = 0x36;
  static const char KEY_7 = 0x37;
  static const char KEY_8 = 0x38;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;*/

  static const char KEY_A = 0x61; //keys are assigned value based
  static const char KEY_D = 0x64; //on hexadecimal ascii values
  static const char KEY_J = 0x6a;
  static const char KEY_S = 0x73;
  static const char KEY_W = 0x77;
  static const char KEY_L = 0x6c;
  static const char KEY_I = 0x69;
  static const char KEY_K = 0x6b;
  static const char KEY_H = 0x68;

  static const char KEYCODE_SPACE  = 0x20;


  const double LINEAR_VEL  = 0.2;
  const double ANGULAR_VEL = 0.4;
  const double VERTICAL_VEL = 2.5;
  const double HORIZONTAL_VEL = 0.2;

public:
  SIGVerseTb3OpenManipulatorGraspingTeleopKey();

  void keyLoop(int argc, char** argv);

private:

  static void rosSigintHandler(int sig);
  static int  canReceiveKey( const int fd );

  void moveBase(ros::Publisher &publisher, const double linear_x, const double linear_y, const double linear_z, const double angular_z);

  static int calcTrajectoryDuration(const double val, const double current_val);

  void showHelp();

  // Current positions that is updated by JointState
  double joint1_pos1_, joint2_pos1_, joint3_pos1_, joint4_pos1_, grip_joint_pos1_;
  double joint1_pos2_, joint2_pos2_, joint3_pos2_, joint4_pos2_;
};


SIGVerseTb3OpenManipulatorGraspingTeleopKey::SIGVerseTb3OpenManipulatorGraspingTeleopKey()
{
  joint1_pos1_ = 0.0; joint2_pos1_ = 0.0; joint3_pos1_ = 0.0; joint4_pos1_ = 0.0; grip_joint_pos1_ = 0.0;
  joint1_pos2_ = 0.0; joint2_pos2_ = 0.0; joint3_pos2_ = 0.0; joint4_pos2_ = 0.0;
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseTb3OpenManipulatorGraspingTeleopKey::canReceiveKey( const int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}



void SIGVerseTb3OpenManipulatorGraspingTeleopKey::moveBase(ros::Publisher &publisher, const double linear_x, const double linear_y, const double linear_z, const double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.linear.z  = linear_z;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher.publish(twist);
}


int SIGVerseTb3OpenManipulatorGraspingTeleopKey::calcTrajectoryDuration(const double val, const double current_val)
{
  return std::max<int>((int)(std::abs(val - current_val) / 0.5), 1);
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::showHelp()
{
  puts("\n");
  puts("---------------------------");
  puts("Operate from keyboard");
  puts("---------------------------");
  puts("Space: Emergency Stop");
  puts("---------------------------");
  puts("w: Fly Forward");
  puts("s: Fly Back");
  puts("d: Strafe Right");
  puts("a: Strafe Left");
  puts("---------------------------");
  puts("i: Upward");
  puts("k: Downward");
  puts("---------------------------");
  puts("j: Turn Left");
  puts("l: Turn Right");
  puts("---------------------------");
  puts("h: Show help");
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::keyLoop(int argc, char** argv)
{
  char c;
  int  ret;
  char buf[1024];

  /////////////////////////////////////////////
  // get the console in raw mode
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  ros::init(argc, argv, "tb3_omc_teleop_key", ros::init_options::NoSigintHandler);

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(10);

  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_joint_trajectory_topic_name;

  node_handle.param<std::string>("grasping_teleop_key/sub_joint_state_topic_name",      sub_joint_state_topic_name,      "/tb3omc/joint_state");
  node_handle.param<std::string>("grasping_teleop_key/pub_twist_topic_name",            pub_base_twist_topic_name,       "/tb3omc/cmd_vel");
  node_handle.param<std::string>("grasping_teleop_key/pub_joint_trajectory_topic_name", pub_joint_trajectory_topic_name, "/tb3omc/joint_trajectory");

  ros::Publisher pub_base_twist = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
  ros::Publisher pub_joint_traj = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_joint_trajectory_topic_name, 10);

  sleep(2);

  showHelp();

  while (ros::ok())
  {
    if(canReceiveKey(kfd))
    {
      // get the next event from the keyboard
      if((ret = read(kfd, &buf, sizeof(buf))) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      c = buf[ret-1];

      switch(c)
      {
        case KEYCODE_SPACE:
        {
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0, 0.0, 0.0);
          break;
        }
        // X, Y, Z, Turn Z
        case KEY_W:
        //case KEYCODE_UP:
        {
          ROS_DEBUG("Go Forward");
          moveBase(pub_base_twist, +LINEAR_VEL, 0.0, 0.0, 0.0);
          break;
        }
        case KEY_D:
        {
          ROS_DEBUG("Strafe Right");
          moveBase(pub_base_twist, 0.0, +HORIZONTAL_VEL, 0.0, 0.0);
          break;
        }
        case KEY_A:
        {
          ROS_DEBUG("Strafe Left");
          moveBase(pub_base_twist, 0.0, -HORIZONTAL_VEL, 0.0, 0.0);
          break;
        }
        case KEY_S:
        //case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Back");
          moveBase(pub_base_twist, -LINEAR_VEL, 0.0, 0.0, 0.0);
          break;
        }
        case KEY_L:
        //case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Turn Right");
          moveBase(pub_base_twist, 0.0, 0.0, 0.0, +ANGULAR_VEL);
          break;
        }
        case KEY_J:
        //case KEYCODE_LEFT:
        {
          ROS_DEBUG("Turn Left");
          moveBase(pub_base_twist, 0.0, 0.0, 0.0, -ANGULAR_VEL);
          break;
        }
        case KEY_I:
        {
          ROS_DEBUG("Go Up");
          moveBase(pub_base_twist, 0.0, 0.0, +VERTICAL_VEL, 0.0);
          break;
        }
        case KEY_K:
        {
          ROS_DEBUG("Go Down");
          moveBase(pub_base_twist, 0.0, 0.0, -VERTICAL_VEL, 0.0);
          break;
        }
        
        case KEY_H:
        {
          ROS_DEBUG("Show Help");
          showHelp();
          break;
        }
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3OpenManipulatorGraspingTeleopKey grasping_teleop_key;

  grasping_teleop_key.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

