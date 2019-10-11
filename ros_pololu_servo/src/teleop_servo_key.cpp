#include <ros/ros.h>
#include <ros_pololu_servo/PololuController.h>
#include <ros_pololu_servo/PololuMath.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopServo
{
public:
  TeleopServo();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double pos;//double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher command_pub;
  
};

TeleopServo::TeleopServo():
	pos(0)
{
  command_pub = nh_.advertise<ros_pololu_servo::MotorCommand>("pololu/command", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_servo_key");
  TeleopServo teleop_servo;

  signal(SIGINT,quit);

  teleop_servo.keyLoop();
  
  return(0);
}


void TeleopServo::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the car.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

   // linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
    
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        pos = -0.3;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        pos = 0.3;
        break;
      case KEYCODE_U:
       // ROS_DEBUG("UP");
       // linear_ = 1.0;
       // dirty = true;
        break;
      case KEYCODE_D:
       // ROS_DEBUG("DOWN");
       // linear_ = -1.0;
       //dirty = true;
        break;
    }
   

    ros_pololu_servo::MotorCommand msg;
    msg.joint_name = "servo";
    msg.position = pos;
    msg.speed = 0.1;
    msg.acceleration = 0.0;
    command_pub.publish(msg);
    //twist.angular.z = a_scale_*angular_;
    //twist.linear.x = l_scale_*linear_;
   // if(dirty ==true)
   // {
    //  twist_pub_.publish(twist);    
     // dirty=false;
   // }
  }


  return;
}

