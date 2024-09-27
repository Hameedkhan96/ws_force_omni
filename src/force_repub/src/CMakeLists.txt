#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Eigen>

using namespace std;

Eigen::Vector4f cmd_vel;

 
void cmdvelCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
  cmd_vel << msg->x, msg->y, msg->z, msg->w;
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/iris/cmd_vel_temp", 1000, cmdvelCallback);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("/iris/cmd/motor_vel", 1000);

  ros::Rate loop_rate(200);

  cmd_vel.setZero();
  
  while (ros::ok()) {
    
    std_msgs::Float32MultiArray pub_msg;
        pub_msg.data.resize( 4 );
//    cout <<"Data: " << cmd_vel.transpose() << endl;
    for(int i=0;i<4;i++){
      pub_msg.data[i] = cmd_vel(i);
    }
    
    chatter_pub.publish(pub_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
