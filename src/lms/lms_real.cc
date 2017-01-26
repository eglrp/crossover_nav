// /**
//  *  @brief Robot 2D trajectory optimization
//  *
//  *  @author Atsushi Sakai
//  **/

// #include "ceres/ceres.h"
// #include "glog/logging.h"
// #include "math.h"
// #include "csvparser.h"
// #include "matplotlibcpp.h"
// #include <vector>

// using namespace std;

// namespace plt = matplotlibcpp;

// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::CauchyLoss;
// using ceres::Problem;
// using ceres::Solver;
// using ceres::Solve;

// struct GPSConstraint{
// 	GPSConstraint(double x, double y, double n)
// 	:x_(x), y_(y), n_(n) {}

//   template <typename T>
// 	bool operator()(
// 		const T* const x,
// 		const T* const y,
// 		T* residual) const {
// 		residual[0]=(x[0]-T(x_))/n_;
// 		residual[1]=(y[0]-T(y_))/n_;
// 		return true;
// 	}

// 	static ceres::CostFunction* Create(
// 		const double gx,
// 		const double gy,
// 		const double gn
// 		){
// 		return (new ceres::AutoDiffCostFunction<GPSConstraint,2,1,1>(
// 			new GPSConstraint(gx,gy,gn)));
// 	}

// private:
//     const double x_;//gps position x
//     const double y_;//gps position y
//     const double n_;//gps xy accuracy
// };

// struct OdometryConstraint{
// 	OdometryConstraint(double dl, double dtheta, double dl_n, double dtheta_n)
// 	:dl_(dl), dtheta_(dtheta), dl_n_(dl_n), dtheta_n_(dtheta_n) {}

//   template <typename T>
// 	bool operator()(
// 		const T* const cx,
// 		const T* const cy,
// 		const T* const cyaw,
// 		const T* const nx,
// 		const T* const ny,
// 		const T* const nyaw,
// 		T* residual) const {
// 		residual[0]=(nx[0]-(cx[0]+dl_*cos(cyaw[0])))/dl_n_;
// 		residual[1]=(ny[0]-(cy[0]+dl_*sin(cyaw[0])))/dl_n_;
// 		residual[2]=(nyaw[0]-(cyaw[0]+dtheta_))/dtheta_n_;
// 		return true;
// 	}

// 	static ceres::CostFunction* Create(
// 		const double dl,
// 		const double dtheta,
// 		const double dl_n,
// 		const double dtheta_n
// 		){
// 		return (new ceres::AutoDiffCostFunction<OdometryConstraint,3,1,1,1,1,1,1>(
// 			new OdometryConstraint(dl,dtheta,dl_n, dtheta_n)));
// 	}

// private:
//     const double dl_;//move distance
//     const double dtheta_;//change angle
//     const double dl_n_;// move distance accuracy
//     const double dtheta_n_;// change angle accuracy
// };




// #include <ros/ros.h>
// #include <ros/time.h>
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include <sensor_fusion_comm/DoubleArrayStamped.h>

// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Quaternion.h>

// enum {
// 	px=0,
// 	py,
// 	pz,
// 	vx,
// 	vy,
// 	vz,
// 	qw,
// 	qx,
// 	qy,
// 	qz,
// 	b_wx,
// 	b_wy,
// 	b_wz,
// 	b_ax,
// 	b_ay,
// 	b_az,
// 	L,
// 	pwvx,
// 	pwvy,
// 	pwvz,
// 	bp,
// 	qifw,
// 	qifx,
// 	qify,
// 	qifz,
// 	sizestate
// };



// geometry_msgs::PoseWithCovarianceStamped poseMsg;
// sensor_fusion_comm::DoubleArrayStamped state_out;
// ros::Time  time_start;
// double ekf_x,ekf_y,ekf_yaw;
// double dx,dy,dyaw;
// double zx,zy,hacc;
// double yaw_0;

// void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {
// 	state_out = *data;
	
// 	  //just for get raw yaw that cam is -- to calibrate yaw offset online -- not the same place at 
// 	tf::Matrix3x3 R(tf::Quaternion(	state_out.data[qw],
// 		state_out.data[qx],
// 		state_out.data[qy],
// 		state_out.data[qz]));
// 	double r_,p_,y_;
// 	R.getRPY(r_,p_,y_);


// 	static double x_old 	=state_out.data[px];
// 	static double y_old 	=state_out.data[py];
// 	static double yaw_old	=y_;


// 	ekf_x = state_out.data[px];
// 	ekf_y = state_out.data[py];
// 	ekf_yaw=y_;
// 	dx = state_out.data[px]-x_old;
// 	dy = state_out.data[py]-y_old;
// 	dyaw = y_-yaw_old;
// 	x_old   = state_out.data[px];
// 	y_old   = state_out.data[py];
// 	yaw_old = y_;
// }


// void gps_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data) {
// 	poseMsg = *data;
// 	zx = poseMsg.pose.pose.position.x;
// 	zy = poseMsg.pose.pose.position.y;
// 	hacc = poseMsg.pose.covariance[0];
// }






// int main(int argc, char** argv){


// 	ros::init(argc, argv, "quad_talker");
// 	ros::NodeHandle n;
// 	ros::Rate r(100);
// 	ros::Subscriber gps_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 10, gps_callback);
// 	ros::Subscriber state_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 10, state_out_callback);

// 	state_out.data.resize(sizestate);

// 	//start timer
// 	time_start = ros::Time::now();
// 	ros::Time cur_time = time_start;
// 	ros::Time 	state_ts, 
// 	gps_ts;

// 	//start callback event in main loop
// 	state_ts =
// 	gps_ts = 
// 	ros::Time::now();





// 	cout<<"Start similation"<<endl;
// 	google::InitGoogleLogging(argv[0]);


//   //GPS vector for plot
// 	vector<double> vzx;
// 	vector<double> vzy;
//   //parameter
// 	vector<double> x;
// 	vector<double> y;
// 	vector<double> yaw;

//   //init param
// 	vector<double> ix;
// 	vector<double> iy;
// 	vector<double> iyaw;


//   //====Optimization=====
// 	ceres::Problem problem;
// 	// static int gps_count=0;
// 	x.resize(250);
// 	y.resize(250);
// 	yaw.resize(250);



// 	static int i = 0;

// 	while (ros::ok() && i < 250 )
// 	{	
		
// 		r.sleep();
// 		ros::spinOnce();
// 		cur_time = ros::Time::now();
// 		if(state_ts !=	state_out.header.stamp) {
// 			state_ts=state_out.header.stamp;

// 			//to plot
// 			ix.push_back(ekf_x);
// 			iy.push_back(ekf_y);

// 			i++;

// 			static bool inited = false;
// 			if (!inited) {
// 				x[0]=ekf_x;
// 				y[0]=ekf_y;
// 				yaw[0]=ekf_yaw;
// 				inited=true;
// 			}
// 			//displacement (need fix to used x y directly)
// 			double dL;
// 			dL=sqrtf(dx*dx+dy*dy);


// 			//odometry real x y

			
// 			// odometry constraint
// 			problem.AddResidualBlock(
// 				OdometryConstraint::Create(dL, dyaw, 0.02,0.002),
// 				NULL,
// 				&(x[i]),
// 				&(y[i]),
// 				&(yaw[i]),
// 				&(x[i+1]),
// 				&(y[i+1]),
// 				&(yaw[i+1])
// 				);

			

// 		}
// 		if(gps_ts	!=	poseMsg.header.stamp)	{
// 			gps_ts = poseMsg.header.stamp;
// 			vzx.push_back(zx);
// 			vzy.push_back(zy);
// 			//gps constraint
// 			problem.AddResidualBlock(
// 				GPSConstraint::Create(zx,zy,hacc),
// 				NULL,
// 				&x[i],
// 				&y[i]
// 				);
// 			// gps_count++;
// 			// printf("(gps count = %d)\n", gps_count);
// 		}


// 	}


//   //Optimization
// 	Solver::Options options;
// 	options.linear_solver_type=ceres::DENSE_QR;
// 	options.minimizer_progress_to_stdout=true;
// 	Solver::Summary summary;
// 	Solve(options,&problem,&summary);
// 	plt::named_plot("ekf",ix,iy, "-g");
// 	plt::named_plot("optimized",x, y, "-r");
// 	plt::named_plot("GPS", vzx, vzy, "xk");
// 	plt::legend();
// 	plt::axis("equal");
// 	plt::grid(true);
// 	plt::show();
// 	return 0;
// }

/**
 *  @brief Robot 2D trajectory optimization
 *
 *  @author Atsushi Sakai
 **/

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "math.h"
#include "csvparser.h"
#include "matplotlibcpp.h"
#include <vector>

using namespace std;

namespace plt = matplotlibcpp;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct GPSConstraint{
	GPSConstraint(double x, double y, double n)
	:x_(x), y_(y), n_(n) {}

  template <typename T>
	bool operator()(
		const T* const x,
		const T* const y,
		T* residual) const {
		residual[0]=(x[0]-T(x_))/n_;
		residual[1]=(y[0]-T(y_))/n_;
		return true;
	}

	static ceres::CostFunction* Create(
		const double gx,
		const double gy,
		const double gn
		){
		return (new ceres::AutoDiffCostFunction<GPSConstraint,2,1,1>(
			new GPSConstraint(gx,gy,gn)));
	}

private:
    const double x_;//gps position x
    const double y_;//gps position y
    const double n_;//gps xy accuracy
};

struct OdometryConstraint{
	OdometryConstraint(double dl, double dtheta, double dl_n, double dtheta_n)
	:dl_(dl), dtheta_(dtheta), dl_n_(dl_n), dtheta_n_(dtheta_n) {}

  template <typename T>
	bool operator()(
		const T* const cx,
		const T* const cy,
		const T* const cyaw,
		const T* const nx,
		const T* const ny,
		const T* const nyaw,
		T* residual) const {
		residual[0]=(nx[0]-(cx[0]+dl_*cos(cyaw[0])))/dl_n_;
		residual[1]=(ny[0]-(cy[0]+dl_*sin(cyaw[0])))/dl_n_;
		residual[2]=(nyaw[0]-(cyaw[0]+dtheta_))/dtheta_n_;
		return true;
	}

	static ceres::CostFunction* Create(
		const double dl,
		const double dtheta,
		const double dl_n,
		const double dtheta_n
		){
		return (new ceres::AutoDiffCostFunction<OdometryConstraint,3,1,1,1,1,1,1>(
			new OdometryConstraint(dl,dtheta,dl_n, dtheta_n)));
	}

private:
    const double dl_;//move distance
    const double dtheta_;//change angle
    const double dl_n_;// move distance accuracy
    const double dtheta_n_;// change angle accuracy
};




#include <ros/ros.h>
#include <ros/time.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_fusion_comm/DoubleArrayStamped.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

enum {
	px=0,
	py,
	pz,
	vx,
	vy,
	vz,
	qw,
	qx,
	qy,
	qz,
	b_wx,
	b_wy,
	b_wz,
	b_ax,
	b_ay,
	b_az,
	L,
	pwvx,
	pwvy,
	pwvz,
	bp,
	qifw,
	qifx,
	qify,
	qifz,
	sizestate
};



geometry_msgs::PoseWithCovarianceStamped poseMsg;
sensor_fusion_comm::DoubleArrayStamped state_out;
ros::Time  time_start;
double ekf_x,ekf_y,ekf_yaw;
double dx,dy,dyaw;
double zx,zy,hacc;
double yaw_0;

void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {
	state_out = *data;
	
	  //just for get raw yaw that cam is -- to calibrate yaw offset online -- not the same place at 
	tf::Matrix3x3 R(tf::Quaternion(	state_out.data[qw],
		state_out.data[qx],
		state_out.data[qy],
		state_out.data[qz]));
	double r_,p_,y_;
	R.getRPY(r_,p_,y_);


	static double x_old 	=state_out.data[px];
	static double y_old 	=state_out.data[py];
	static double yaw_old	=y_;


	ekf_x = state_out.data[px];
	ekf_y = state_out.data[py];
	ekf_yaw=y_;
	dx = state_out.data[px]-x_old;
	dy = state_out.data[py]-y_old;
	dyaw = y_-yaw_old;
	x_old   = state_out.data[px];
	y_old   = state_out.data[py];
	yaw_old = y_;
}


void gps_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data) {
	poseMsg = *data;
	zx = poseMsg.pose.pose.position.x;
	zy = poseMsg.pose.pose.position.y;
	hacc = poseMsg.pose.covariance[0];
}






int main(int argc, char** argv){


	ros::init(argc, argv, "quad_talker");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Subscriber gps_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 10, gps_callback);
	ros::Subscriber state_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 10, state_out_callback);

	state_out.data.resize(sizestate);

	//start timer
	time_start = ros::Time::now();
	ros::Time cur_time = time_start;
	ros::Time 	state_ts, 
	gps_ts;

	//start callback event in main loop
	state_ts =
	gps_ts = 
	ros::Time::now();





	cout<<"Start similation"<<endl;
	google::InitGoogleLogging(argv[0]);


  //GPS vector for plot
	vector<double> vzx;
	vector<double> vzy;
  //parameter
	vector<double> x;
	vector<double> y;
	vector<double> yaw;

  //init param
	vector<double> ix;
	vector<double> iy;
	vector<double> iyaw;


  //====Optimization=====
	ceres::Problem problem;
	// static int gps_count=0;
	x.resize(250);
	y.resize(250);
	yaw.resize(250);



	static int i = 0;

	while (ros::ok())
	{	
		
		r.sleep();
		ros::spinOnce();
		cur_time = ros::Time::now();

		if(gps_ts	!=	poseMsg.header.stamp)	{
			gps_ts = poseMsg.header.stamp;
		}else{
			zx = 0;
			zy = 0;
			hacc=0;
		}
		if(state_ts !=	state_out.header.stamp) {
			state_ts=state_out.header.stamp;

			i++;

			//displacement (need fix to used x y directly)
			double dL;
			dL=sqrtf(dx*dx+dy*dy);
			printf("%d,55,55,55,%.4f,%.4f,%.4f,%.4f,%.4f,udl,udth,%.4f,0.01,0.002\n", i,
																					 ekf_x,
																					 ekf_y,
																					 ekf_yaw,
																					 zx,
																					 zy,
																					 hacc);

		}


	}


  //Optimization
	Solver::Options options;
	options.linear_solver_type=ceres::DENSE_QR;
	options.minimizer_progress_to_stdout=true;
	Solver::Summary summary;
	Solve(options,&problem,&summary);
	plt::named_plot("ekf",ix,iy, "-g");
	plt::named_plot("optimized",x, y, "-r");
	plt::named_plot("GPS", vzx, vzy, "xk");
	plt::legend();
	plt::axis("equal");
	plt::grid(true);
	plt::show();
	return 0;
}

