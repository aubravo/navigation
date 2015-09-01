#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/stat.h>

// POLOLU SERVO MASTER TAGS
const uint cmpct_target= 0x84;
const uint mini_target  =0xFF;
const uint cmpct_set_speed = 0x87;
const uint cmpct_set_accel = 0x89;


// MOTOR CHANNEL FOR EACH MOTOR AT POLOLU SERVO MASTER
const uint m1_ = 0x00;
const uint m2_ = 0x01;
const uint m3_ = 0x02;
const uint m4_ = 0x03;

const char* device = "/dev/ttyACM0";
int fd = open(device,O_RDWR | O_NOCTTY);

class MotorControl
{
public:
MotorControl();

private:
void velCallback(const geometry_msgs::Twist::ConstPtr& Twist);
void accelCallback(const geometry_msgs::Vector3::ConstPtr& Accel);
void orientCallback(const geometry_msgs::Vector3::ConstPtr& Orient);
void Pololu_set_target(int mn_, int trgt);
//void Pololu_init();



double a_, b_, m1_target, m2_target, m3_target, m4_target, m1_vel, m2_vel, m3_vel, m4_vel;

//ROS
ros::NodeHandle nh_;

ros::Subscriber get_vel;
ros::Subscriber get_count;
ros::Subscriber get_accel;
ros::Subscriber get_orient;
};

MotorControl::MotorControl()
{
	nh_.param("a",a_,a_);
	nh_.param("b",b_,b_);
	
	
	
	if(fd == -1)
	{
    perror(device);
 	}

	struct termios options;
	tcgetattr(fd, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(fd, TCSANOW, &options);

	//Pololu_init();

	get_vel = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &MotorControl::velCallback, this);
	get_accel = nh_.subscribe<geometry_msgs::Vector3>("sensor_accel", 10, &MotorControl::accelCallback, this);
	get_orient = nh_.subscribe<geometry_msgs::Vector3>("sensor_orient", 10, &MotorControl::orientCallback, this);
}

void MotorControl::Pololu_set_target(int mn_, int trgt){
	char wr[] = {mini_target,mn_,trgt};
	if(write(fd,wr,sizeof(wr)) == -1){
		ROS_ERROR("ERROR WRITING POSITION");
	}
}

/*void MotorControl::Pololu_init(){
	char wr[] = {cmpct_set_speed,m1_,0x0C,0x01,cmpct_set_speed,m2_,0x0C,0x01,cmpct_set_speed,m3_,0x0C,0x01,cmpct_set_speed,m4_,0x0C,0x01,cmpct_set_accel,m1_,0x00,0x00};

	if(write(fd,wr,sizeof(wr)) == -1);
	{
		ROS_ERROR("ERROR INITIALIZING DEVICE:SPEED SET");
	}
	
	char wr2[] = {cmpct_set_accel,m2_,0x00,0x00,cmpct_set_accel,m3_,0x00,0x00,cmpct_set_accel,m4_,0x00,0x00};

	if(write(fd,wr2,sizeof(wr2)) == -1);
	{
		ROS_ERROR("ERROR INITIALIZING DEVICE:ACCEL SET");
	}

	char wr3[] = {mini_target,m1_,127,mini_target,m2_,127,mini_target,m3_,127,mini_target,m4_,127};

	if(write(fd,wr3,sizeof(wr3)) == -1);
	{
		ROS_ERROR("ERROR INITIALIZING DEVICE:TARGET SET");
	}
	
}*/

void MotorControl::velCallback(const geometry_msgs::Twist::ConstPtr& Twist)
{
	float x_vel, y_vel, w_vel;

	x_vel = Twist->linear.x;
	y_vel = Twist->linear.y;
	w_vel = Twist->angular.z;

	m1_target = 127+127*((y_vel - x_vel + w_vel*(a_+b_)));
	m2_target = 127+127*(-(y_vel + x_vel - w_vel*(a_+b_)));
	m3_target = 127+127*(-(y_vel - x_vel - w_vel*(a_+b_)));
	m4_target = 127+127*((y_vel + x_vel + w_vel*(a_+b_)));
	ROS_INFO("CHANGING TARGET");

	Pololu_set_target(m1_,m1_target);
	Pololu_set_target(m2_,m2_target);
	Pololu_set_target(m3_,m3_target);
	Pololu_set_target(m4_,m4_target);
}

void MotorControl::accelCallback(const geometry_msgs::Vector3::ConstPtr& Accel)
{
	double x_accel, y_accel;

}

void MotorControl::orientCallback(const geometry_msgs::Vector3::ConstPtr& Orient)
{
	double x_rel_position, y_rel_position;

}

int main(int argc, char** argv){

	ros::init(argc, argv, "move_base");
	MotorControl motor_control;
	ros::spin();
}