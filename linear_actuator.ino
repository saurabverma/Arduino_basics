/*Aim : This ROS code runs on an Arduino platform to indirectly control the
  linear actuation of using a set of Relay switches.
*/

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8MultiArray.h>

#define pinOut1 9
#define pinOut2 10

void messageCallback( const std_msgs::UInt8& toggle_msg)
{
  //digitalWrite(13, HIGH-digitalRead(13));   // DEBUG: blink the led

  uint8_t input1 = toggle_msg.data;

  if (input == 1)
  {
    digitalWrite(pinOut1, LOW);
    digitalWrite(pinOut2, HIGH);
  }
  else if (input == 2)
  {
    digitalWrite(pinOut1, HIGH);
    digitalWrite(pinOut2, LOW);
  }
  else
  {    
    digitalWrite(pinOut1, LOW);
    digitalWrite(pinOut2, LOW);
  }
}

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int8MultiArray> sub("linear actuator", &messageCallback );

void setup()
{
  pinMode(pinOut1, OUTPUT);
  pinMode(pinOut2, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
