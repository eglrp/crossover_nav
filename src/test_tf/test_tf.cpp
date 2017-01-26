#include <geometry_msgs/Quaternion.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <turtlesim/Pose.h>

// std::string turtle_name;
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define toRad(x) (x*0.01745329)


int main(int argc, char** argv){
  ros::init(argc, argv, "test_tf");

  ros::NodeHandle node;
  ros::Rate r(10);

  while(ros::ok())
  {
  	//Eigen quaternion wxyz
  	Eigen::Quaternion<double> Q;
  	Eigen::Matrix<double, 3,1> G;




















	static tf::TransformBroadcaster TFB_wi;
	tf::Transform TFwi;
	tf::Quaternion Qwi;


	//Twi
	TFwi.setOrigin( tf::Vector3(3, 0, 0.0) );
	Qwi.setRPY(0, 0, 0);
	TFwi.setRotation(Qwi);
	TFB_wi.sendTransform(tf::StampedTransform(TFwi, ros::Time::now(), "world", "imu"));
	////////////////////////////////////////////////

	static tf::TransformBroadcaster TFB_ic;
	tf::Transform TFic;
	tf::Quaternion Qic;

	//Twi
	TFic.setOrigin( tf::Vector3(0, 0, 0.0) );
	Qic.setRPY(0, 0, toRad(30));
	TFic.setRotation(Qic);
	TFB_wi.sendTransform(tf::StampedTransform(TFic, ros::Time::now(), "imu", "camera"));
	////////////////////////////////////////////////

	// static tf::TransformBroadcaster TFB_wi;
	// tf::Transform TFwi;
	// tf::Quaternion Qwi;

	// //Twi
	// TFwi.setOrigin( tf::Vector3(0, 0, 0.0) );
	// Qwi.setRPY(0, 0, 0);
	// TFwi.setRotation(Qwi);
	// TFB_wi.sendTransform(tf::StampedTransform(TFwi, ros::Time::now(), "camera", "imu"));
	////////////////////////////////////////////////

	static tf::TransformBroadcaster TFB_vc;
	tf::Transform TFvc;
	tf::Quaternion Qvc;
	//Twi
	TFvc.setOrigin( tf::Vector3(2, 0, 0.0) );
	Qvc.setRPY(0, 0, toRad(-45));
	TFvc.setRotation(Qvc);
	TFB_wi.sendTransform(tf::StampedTransform(TFvc, ros::Time::now(), "camera", "vision"));


	r.sleep();
	ros::spinOnce();
  }
  return 0;
};