#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>
#include <Eigen/Geometry> 
#include <tf_conversions/tf_eigen.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "calibration_tf");

  ros::NodeHandle node;

  tf::TransformListener brix1listener, brix2listener, brix3listener;
  tf::TransformBroadcaster brix1Broadcaster, brix2Broadcaster, brix3Broadcaster;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform brix1neck, brix2neck, brix3neck, brix1Transform, brix2Transform, brix3Transform;
    try{
      brix1listener.lookupTransform("global_space", "brix_1/user_1/neck", ros::Time(0), brix1neck);
      brix2listener.lookupTransform("global_space", "brix_2/user_1/neck", ros::Time(0), brix2neck);
      brix3listener.lookupTransform("global_space", "brix_3/user_1/neck", ros::Time(0), brix3neck);

//These repeative pieces of code need to be packaged into functions!!

//set the global Vec3 to a unit vector on the x axis
              tf::Vector3 globalVec3 = tf::Vector3(1,0,0);
              Eigen::Vector3d eigenGlobalVec3;
              tf::vectorTFToEigen(globalVec3, eigenGlobalVec3);

//retrieve the positions of the neck as a 3Vector
              tf::Vector3 brix1NeckVec3 = tf::Vector3(brix1neck.getOrigin().x(), brix1neck.getOrigin().y(), brix1neck.getOrigin().z());
              Eigen::Vector3d eigenBrix1NeckVec3;
              tf::vectorTFToEigen(brix1NeckVec3, eigenBrix1NeckVec3);

              tf::Vector3 brix2NeckVec3 = tf::Vector3(brix2neck.getOrigin().x(), brix2neck.getOrigin().y(), brix2neck.getOrigin().z());
              Eigen::Vector3d eigenBrix2NeckVec3;
              tf::vectorTFToEigen(brix2NeckVec3, eigenBrix2NeckVec3);

              tf::Vector3 brix3NeckVec3 = tf::Vector3(brix3neck.getOrigin().x(), brix3neck.getOrigin().y(), brix3neck.getOrigin().z());
              Eigen::Vector3d eigenBrix3NeckVec3;
              tf::vectorTFToEigen(brix3NeckVec3, eigenBrix3NeckVec3);

//create quaternion classes and perform setFromTwoVectors before assigning to Transform
              Eigen::Quaterniond eigenQuaternionBrix1Neck;
              eigenQuaternionBrix1Neck.setFromTwoVectors(eigenGlobalVec3, eigenBrix1NeckVec3);
              tf::Quaternion tfQuaternionBrix1Neck;
              tf::quaternionEigenToTF(eigenQuaternionBrix1Neck, tfQuaternionBrix1Neck);
            brix1neck.setRotation(tfQuaternionBrix1Neck);

              Eigen::Quaterniond eigenQuaternionBrix2Neck;
              eigenQuaternionBrix2Neck.setFromTwoVectors(eigenGlobalVec3, eigenBrix2NeckVec3);
              tf::Quaternion tfQuaternionBrix2Neck;
              tf::quaternionEigenToTF(eigenQuaternionBrix2Neck, tfQuaternionBrix2Neck);
            brix2neck.setRotation(tfQuaternionBrix2Neck);

              Eigen::Quaterniond eigenQuaternionBrix3Neck;
              eigenQuaternionBrix3Neck.setFromTwoVectors(eigenGlobalVec3, eigenBrix3NeckVec3);
              tf::Quaternion tfQuaternionBrix3Neck;
              tf::quaternionEigenToTF(eigenQuaternionBrix3Neck, tfQuaternionBrix3Neck);
            brix3neck.setRotation(tfQuaternionBrix3Neck);

//Broadcast the new Neck transform
      brix1Broadcaster.sendTransform(tf::StampedTransform(brix1neck, ros::Time::now(), "global_space", "brix1neck"));
      brix2Broadcaster.sendTransform(tf::StampedTransform(brix2neck, ros::Time::now(), "global_space", "brix2neck"));
      brix3Broadcaster.sendTransform(tf::StampedTransform(brix3neck, ros::Time::now(), "global_space", "brix3neck"));

//lookup the difference between the two neck transforms for each brix
      brix1listener.lookupTransform("brix1neck", "brix1neck", ros::Time(0), brix1Transform);
      brix2listener.lookupTransform("brix2neck", "brix1neck", ros::Time(0), brix2Transform);
      brix3listener.lookupTransform("brix3neck", "brix1neck", ros::Time(0), brix3Transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
	}

        ROS_INFO_STREAM("brix1: " << " X pos " <<  brix1Transform.getOrigin().x() << "  Y pos " <<  brix1Transform.getOrigin().y() << "  z pos " <<  brix1Transform.getOrigin().z() );
        ROS_INFO_STREAM("brix2: " << " X pos " <<  brix2Transform.getOrigin().x() << "  Y pos " <<  brix2Transform.getOrigin().y() << "  z pos " <<  brix2Transform.getOrigin().z() );
        ROS_INFO_STREAM("brix3: " << " X pos " <<  brix3Transform.getOrigin().x() << "  Y pos " <<  brix3Transform.getOrigin().y() << "  z pos " <<  brix3Transform.getOrigin().z() );

        std::ofstream myfile;
        myfile.open ("brix1calibration.txt" , std::ios::out | std::ios::app);
        myfile << "tx: -" << brix1Transform.getOrigin().x() << " ty:  " << brix1Transform.getOrigin().y() << " tz: " << brix1Transform.getOrigin().z() << " rx: " << brix1Transform.getRotation().x() << " ry: " << brix1Transform.getRotation().y() << " rz: " << brix1Transform.getRotation().z() << " rw: " << brix1Transform.getRotation().w() << std::endl;
        myfile.close();

        myfile.open ("brix2calibration.txt" , std::ios::out | std::ios::app);
        myfile << "tx: " << brix2Transform.getOrigin().x() << " ty:  " << brix2Transform.getOrigin().y() << " tz: " << brix2Transform.getOrigin().z() << " rx: " << brix2Transform.getRotation().x() << " ry: " << brix2Transform.getRotation().y() << " rz: " << brix2Transform.getRotation().z() << " rw: " << brix2Transform.getRotation().w() << std::endl;
        myfile.close();

        myfile.open ("brix3calibration.txt" , std::ios::out | std::ios::app);
        myfile << "tx: " << brix3Transform.getOrigin().x() << " ty:  " << brix3Transform.getOrigin().y() << " tz: " << brix3Transform.getOrigin().z() << " rx: " << brix3Transform.getRotation().x() << " ry: " << brix3Transform.getRotation().y() << " rz: " << brix3Transform.getRotation().z() << " rw: " << brix3Transform.getRotation().w() << std::endl;
        myfile.close();

    rate.sleep();
  }
  return 0;
};
