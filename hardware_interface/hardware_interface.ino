#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "Motor.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Encoder.h>
#include <PID_v1.h>
/*
    Left Motor:
    Kp : 3.00
    Ki : 10.00
    Kd : 0.1
    Right Motor:
    Kp : 3.00
    Ki : 10.00
    Kd : 0.1
*/
Encoder right_motor(2,3);
Encoder left_motor(18,19);

double input_left;
double input_right;
double output_left;
double output_right;
double target_left;
double target_right;
double kp_left=3.0, ki_left=110.00, kd_left=0.0;
double kp_right=3.0, ki_right=110.00, kd_right=0.0;

PID pidLeft(&input_left, &output_left, &target_left, kp_left, ki_left, kd_left, DIRECT);
PID pidRight(&input_right, &output_right, &target_right, kp_right, ki_right, kd_right, DIRECT);


float ticks_per_rev = 536.60;

#define LOOPTIME 10
Motor right(7,22,3,2);
Motor left(6,23,18,19);

ros::NodeHandle  nh;

geometry_msgs::TransformStamped odom_ts;
geometry_msgs::Quaternion odom_quat; 
tf::TransformBroadcaster odom_bro;
nav_msgs::Odometry odom_msg;

ros::Publisher pub_odom("/odometry",&odom_msg);

long odom_time;
double  xo=0.0 , yo=0.0 , tho = 0.0;
double vx = 0.0;
double vth = 0.0;
double vy = 0.0;
double dt=0.00, delta_xo=0.00, delta_yo=0.00, delta_tho=0.00;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Prev;
float encoder1Prev;

float deltat = 0.00;

float demandx=0;
float demandz=0;
float demand_speed_left=0;
float demand_speed_right=0;

float speed_act_left = 0;                   
float speed_act_right = 0;

float axle = 0.36;
float diameter = 0.10;

//Object to hold the filter, initially held the PID function. Now a PID library is used.
class MotorController{
  public:
  float vFilt=0.00;
  float vPrev=0.00;
  float vt;

  float filter(float vel){
  this->vFilt = 0.86*(this->vFilt) + 0.07*vel + 0.07*(this->vPrev);
  this->vPrev = vel;
  return (this->vFilt);
  }

};


//Callback function for subscribing to cmd_vel
void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

void setup(){
  
//initilize the node, subscriber and publisher
 nh.initNode();
 odom_bro.init(nh);
 nh.advertise(pub_odom);
 nh.subscribe(sub);
 
// Set up the PID Library
  pidLeft.SetMode(AUTOMATIC);
  pidLeft.SetOutputLimits(-255, 255);
  pidLeft.SetSampleTime(5);
  pidRight.SetMode(AUTOMATIC);
  pidRight.SetOutputLimits(-255, 255);
  pidRight.SetSampleTime(5);
}

MotorController controllerLeft;
MotorController controllerRight;

void loop(){

//Attain the encoder reading
  encoder0Pos = left_motor.read();
  encoder1Pos = right_motor.read();
  
  if(millis() >= odom_time){

  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){

    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;

    deltat = (currentMillis - prevMillis)/1000.00;
    
// Speed in Radians/sec
    speed_act_left = (2*3.14*encoder0Diff)/(ticks_per_rev * (deltat));                    
    speed_act_right = (2*3.14*encoder1Diff)/(ticks_per_rev * (deltat));
//Filter the speed data
    speed_act_left = controllerLeft.filter(speed_act_left);
    speed_act_right = controllerRight.filter(speed_act_right);
// Calculate the velocities 
    vth = (diameter/2.00)*(speed_act_right-speed_act_left)/axle;
    vx = (diameter/2.00)*((speed_act_left + speed_act_right)/2.00);
    dt = (currentMillis - prevMillis)/1000.0;

    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
    prevMillis = currentMillis;
  }

//Calculate the X and Y coordinate
  delta_xo = (vx * cos(tho)) * dt;
  delta_yo = (vx * sin(tho)) * dt;
  delta_tho = vth * dt;

  xo += delta_xo;
  yo += delta_yo;
  tho += delta_tho;

//To keep radians between [-pi, pi] not necessary but convenient.
  if(tho>3.14){
    tho = tho - 2.00*3.14;
  }else if(tho < -3.14){
    tho = tho + 2.00*3.14;
  }

//publish the transform over tf
  odom_ts.header.stamp = nh.now();
  odom_ts.header.frame_id = "odom";
  odom_ts.child_frame_id = "base_link";
  odom_ts.transform.translation.x = xo;
  odom_ts.transform.translation.y = yo;
  odom_ts.transform.translation.z = 0.0;
  odom_ts.transform.rotation.x = 0.0;
  odom_ts.transform.rotation.y = 0.0;
  odom_ts.transform.rotation.z = sin(tho/2.00);
  odom_ts.transform.rotation.w = cos(tho/2.00);

//send the transform
  odom_bro.sendTransform(odom_ts);

//publish the odometry message over ROS
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";

//set the position 
  odom_msg.pose.pose.position.x = xo;
  odom_msg.pose.pose.position.y = yo;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(tho/2.00);
  odom_msg.pose.pose.orientation.w = cos(tho/2.00);

//set the velocity
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = vth;

  pub_odom.publish(&odom_msg);
  odom_time = millis()+10;
  }

/*
    the diff_drive_controller provides speed in rad/sec
    rated speed of motor = 256 RPM = 26.8 rad/sec
    we therefore map the input from diff_drive-controller from 0 - 26.8 -> 0 - 100
*/

//find the target velocity in rad/sec of each wheel based on the required linear and angular velocity in m/sec
  demand_speed_left = (demandx - (demandz*(axle/2.00)))/(diameter/2.00);
  demand_speed_right = (demandx + (demandz*(axle/2.00)))/(diameter/2.00);

//Set the target for the PID controller
  target_left = demand_speed_left;
  target_right = demand_speed_right;
//provide actual speed as feedback to the PID controller
  input_left = speed_act_left;
  input_right = speed_act_right;
//Compute the necessary adjustments to be made
  pidLeft.Compute();
  pidRight.Compute(); 
//provide the output of the PID controller to the motor driving function
  left.rotate(output_left);
  right.rotate(output_right);
  
  nh.spinOnce();
  delay(50);
}
Footer
