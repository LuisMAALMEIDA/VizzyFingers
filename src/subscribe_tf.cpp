#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vizzy_fingers/JointAngles.h>
#include <tf/transform_listener.h>
#include <stdint.h>
#include <iostream>
#include <tf/transform_broadcaster.h> 

using namespace std;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise< vizzy_fingers::JointAngles >("/joint_angles_fingers_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("tf", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const tf2_msgs::TFMessage::ConstPtr& input)
  {
  vizzy_fingers::JointAngles::Ptr JointAnglesArray =vizzy_fingers::JointAngles::Ptr(new vizzy_fingers::JointAngles());
  JointAnglesArray->joint_angles.clear();
  JointAnglesArray->joint_angles.resize(3);
  tf::TransformListener listener; 
  tf::StampedTransform transform; 

  ROS_INFO("In a frame, I read :\n");

  try{
      // "1a TRANSFORMAÇÃO (ENTRE A 2a FALANGE E A 1a) (1a falange é a mais proxima da mao)
      // Aqui esperas que a TF esteja disponivel
      listener.waitForTransform("son_frame_1", "son_frame_0", ros::Time(0), ros::Duration(3));
      //Aqui vais obter uma tf (transform) relativamente ao marker anterior
      // We want the transform from frame son_frame_0 to frame son_frame_1. 
      listener.lookupTransform("son_frame_1", "son_frame_0",ros::Time(0), transform);

      tf::Quaternion q = transform.getRotation();
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      ROS_INFO("A primeira rotacao (na primeira falange)\n");
      ROS_INFO("Roll: [%f],Pitch: [%f],Yaw: [%f]",roll,pitch,yaw);
      // output[index][angle] -> 0- roll, 1- pitch, 2 yaw
      JointAnglesArray->joint_angles.at(0).x= roll;
      JointAnglesArray->joint_angles.at(0).y=pitch;
      JointAnglesArray->joint_angles.at(0).z=yaw;
      
      JointAnglesArray->header.stamp=transform.stamp_;
      cout << JointAnglesArray->header.stamp;
      

      // "2a TRANSFORMAÇÃO (ENTRE A 3a FALANGE E A 2a)

      listener.waitForTransform("son_frame_2", "son_frame_1", ros::Time(0), ros::Duration(3));
      listener.lookupTransform("son_frame_2", "son_frame_1",ros::Time(0), transform);

      q = transform.getRotation();
      tf::Matrix3x3 m1(q);
      m1.getRPY(roll, pitch, yaw);
      ROS_INFO("A segunda rotacao:\n");
      ROS_INFO("Roll: [%f],Pitch: [%f],Yaw: [%f]",roll,pitch,yaw);
      JointAnglesArray->joint_angles.at(1).x= roll;
      JointAnglesArray->joint_angles.at(1).y=pitch;
      JointAnglesArray->joint_angles.at(1).z=yaw;

        // "3a TRANSFORMAÇÃO (ENTRE A 4a FALANGE E A 3a)

      listener.waitForTransform("son_frame_3", "son_frame_2", ros::Time(0), ros::Duration(3));
      listener.lookupTransform("son_frame_3", "son_frame_2",ros::Time(0), transform);

      q = transform.getRotation();
      tf::Matrix3x3 m2(q);
      m2.getRPY(roll, pitch, yaw);
      ROS_INFO("A terceira rotacao:\n");
      ROS_INFO("Roll: [%f],Pitch: [%f],Yaw: [%f]",roll,pitch,yaw);
      JointAnglesArray->joint_angles.at(2).x= roll;
      JointAnglesArray->joint_angles.at(2).y=pitch;
      JointAnglesArray->joint_angles.at(2).z=yaw;

      pub_.publish(JointAnglesArray);  
     }
    catch (tf::TransformException &ex) {
      ROS_ERROR("Deu erro... \n");
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }



   
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}



