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
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist twist;
  geometry_msgs::Twist twist_old;
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

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

  //joyはコントローラからの入力が変わった時にしか反応しないので，変化ないときはそのまま前の値をpubし続けるようにする．
/*  do{
    //joy1からのトピックを受け取る．
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
    //一個前の値保持
    twist_old.angular.z = twist.angular.z;
    twist_old.linear.x = twist.linear.x;
  }while(twist_old.angular.z == twist.angular.z && twist_old.linear.x == twist.linear.x);
*/
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  twist_pub_.publish(twist);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

