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
  double pos = 0, throttle = 0;
  float r_pos;
  void motorstateCallback(const ros_pololu_servo::MotorStateList::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_; // = nh_.subscribe("pololu/motor_states", 1, &TeleopServo::motorstateCallback, &teleop_servo);
  ros::Publisher command_pub_;// = nh_.advertise<ros_pololu_servo::MotorCommand>("pololu/command", 1);

};

TeleopServo::TeleopServo():
r_pos(0)
{
sub_ = nh_.subscribe("pololu/motor_states", 1, &TeleopServo::motorstateCallback, this);
command_pub_ = nh_.advertise<ros_pololu_servo::MotorCommand>("pololu/command", 1);
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

void TeleopServo::motorstateCallback(const ros_pololu_servo::MotorStateList::ConstPtr& msg)
{
  r_pos = msg->motor_states[1].radians; //received position
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "teleop_servo_key");
//  ros::NodeHandle nh_;
  TeleopServo teleop_servo;
//  command_pub = nh_.advertise<ros_pololu_servo::MotorCommand>("pololu/command", 1);
 // ros::Subscriber sub = nh_.subscribe("pololu/motor_states", 1, &TeleopServo::motorstateCallback, &teleop_servo);
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

 // ROS_INFO_THROTTLE(1,"Hi %f",r_pos);
  for(;;)
  {
   // ROS_INFO_THROTTLE(1,"Hi %f",r_pos);

    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

   // key_flag = 0 when left or right is pressed, key_flag = 1 when up or down is pressed 
   int key_flag = -1;
   
   // if(pos>=-0.549779 && pos<=0.549779)
   // {
  	switch(c)
       	{
       	  case KEYCODE_L:
            ROS_DEBUG("LEFT");
            key_flag = 0;
            pos = pos - 0.1;
		if(pos<-0.549779 || pos>0.549779)
			pos = pos + 0.1;
            break;
          case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            key_flag = 0;
            pos = pos + 0.1;
		if(pos<-0.549779 || pos>0.549779)
                        pos = pos - 0.1;
          break;
          case KEYCODE_U:
            ROS_DEBUG("UP");
            key_flag = 1;
            throttle = throttle + 0.05;
		if(throttle>0.5)
                        throttle = throttle - 0.05;
            break;
          case KEYCODE_D:
            ROS_DEBUG("DOWN");
            key_flag = 1;
            throttle = throttle - 0.05;
                if(throttle<-0.5)
                        throttle = throttle + 0.05;
            break;
        }
   
    ros_pololu_servo::MotorCommand msg;
    if(key_flag == 0) {
    	msg.joint_name = "servo";
        msg.position = pos;
        msg.speed = 0.1;
        msg.acceleration = 0.0;
        command_pub_.publish(msg);
    }
    else if(key_flag == 1) {
        msg.joint_name = "motor";
        msg.position = throttle;
        msg.speed = 0.1;
        msg.acceleration = 0.0;
        command_pub_.publish(msg);
    }
        ros::spinOnce();
  }


  return;
}

