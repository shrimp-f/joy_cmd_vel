#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  ros::Publisher twist_pub_serial_;//rosserialでarduinoの方に送る用．
  ros::Subscriber joy_sub_;


  geometry_msgs::Twist twist;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  //turtle_simの方にトピックを送る．サービス？
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  twist_pub_serial_ = nh_.advertise<geometry_msgs::Twist>("cabbage1/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//  twist.angular.z = a_scale_*joy->axes[angular_];
//  twist.linear.x = l_scale_*joy->axes[linear_];

  twist.linear.x = joy->axes[7];
  twist.angular.z = joy->axes[3];

  twist_pub_.publish(twist);
  twist_pub_serial_.publish(twist);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

