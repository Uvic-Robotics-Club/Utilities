/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Accel.h>

#include <PID_v1.h>

ros::NodeHandle  nh;

double setpoint = 0;
double actual;
double output;
PID pid(&actual, &output, &setpoint, 0.6, 1.5, 0, P_ON_E, DIRECT);

void messageCb( const std_msgs::Float32& new_setpoint){
  setpoint = new_setpoint.data;
}


ros::Subscriber<std_msgs::Float32> sub("motor_actual_setpoint", messageCb );
geometry_msgs::Accel accel_msg;
ros::Publisher topic("motor_actual", &accel_msg);


void setup()
{
  nh.initNode();
  nh.advertise(topic);
  nh.subscribe(sub);
  pid.SetOutputLimits(-100,100);
  pid.SetMode(AUTOMATIC);
}

void loop()
{
  pid.Compute();
  actual += output;
  accel_msg.linear.x = setpoint;
  accel_msg.linear.y = actual;
  accel_msg.linear.z = output;
  accel_msg.angular.x = setpoint-actual;
  topic.publish( &accel_msg);
  nh.spinOnce();
  delay(100);
}
