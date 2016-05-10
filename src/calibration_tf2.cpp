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
//lookup the difference between the two calib transforms for each brix
      brix1listener.lookupTransform("brix1/user_1/calibrationOrigin", "brix1/user_1/calibrationOrigin", ros::Time(0), brix1Transform);
      brix2listener.lookupTransform("brix1/user_1/calibrationOrigin", "brix2/user_1/calibrationOrigin", ros::Time(0), brix2Transform);
//      brix3listener.lookupTransform("brix1/user_1/calibrationOrigin", "brix3/user_1/calibrationOrigin", ros::Time(0), brix3Transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
	}

        ROS_INFO_STREAM("brix1: " << " X pos " <<  brix1Transform.getOrigin().x() << "  Y pos " <<  brix1Transform.getOrigin().y() << "  z pos " <<  brix1Transform.getOrigin().z() );
        ROS_INFO_STREAM("brix2: " << " X pos " <<  brix2Transform.getOrigin().x() << "  Y pos " <<  brix2Transform.getOrigin().y() << "  z pos " <<  brix2Transform.getOrigin().z() );
//        ROS_INFO_STREAM("brix3: " << " X pos " <<  brix3Transform.getOrigin().x() << "  Y pos " <<  brix3Transform.getOrigin().y() << "  z pos " <<  brix3Transform.getOrigin().z() );

        std::ofstream myfile;
        myfile.open ("brix1calibration.txt" , std::ios::out | std::ios::app);
        myfile << "tx: -" << brix1Transform.getOrigin().x() << " ty:  " << brix1Transform.getOrigin().y() << " tz: " << brix1Transform.getOrigin().z() << " rx: " << brix1Transform.getRotation().x() << " ry: " << brix1Transform.getRotation().y() << " rz: " << brix1Transform.getRotation().z() << " rw: " << brix1Transform.getRotation().w() << std::endl;
        myfile.close();

        myfile.open ("brix2calibration.txt" , std::ios::out | std::ios::app);
        myfile << "tx: " << brix2Transform.getOrigin().x() << " ty:  " << brix2Transform.getOrigin().y() << " tz: " << brix2Transform.getOrigin().z() << " rx: " << brix2Transform.getRotation().x() << " ry: " << brix2Transform.getRotation().y() << " rz: " << brix2Transform.getRotation().z() << " rw: " << brix2Transform.getRotation().w() << std::endl;
        myfile.close();

//        myfile.open ("brix3calibration.txt" , std::ios::out | std::ios::app);
//        myfile << "tx: " << brix3Transform.getOrigin().x() << " ty:  " << brix3Transform.getOrigin().y() << " tz: " << brix3Transform.getOrigin().z() << " rx: " << brix3Transform.getRotation().x() << " ry: " << brix3Transform.getRotation().y() << " rz: " << brix3Transform.getRotation().z() << " rw: " << brix3Transform.getRotation().w() << std::endl;
//        myfile.close();

    rate.sleep();
  }
  return 0;
};
