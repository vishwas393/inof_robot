#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>


static geometry_msgs::Vector3 ToEulerAngles(geometry_msgs::Quaternion q) {
	geometry_msgs::Vector3 angle;


	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angle.x = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (std::abs(sinp) >= 1)
		angle.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		angle.y = std::asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angle.z = std::atan2(siny_cosp, cosy_cosp);

	return angle;
}

static int get_robot_frame_position(const gazebo_msgs::ModelStates mdl)
{
	std::vector<std::basic_string<char>>::const_iterator str = std::find(std::begin(mdl.name), std::end(mdl.name), "robot");
	if(str != std::end(mdl.name))
	{
		return str - std::begin(mdl.name);
	}
	else { return -1; }
}

class inof_robot
{
	private:
	double goal[3];
	std::vector<std::pair<int, int>> path;
	double vd = 0.0, wd = 0.0;
	double k1=10, k2 =3, k3 =16;
	int path_point_cnt = 0;
	ros::Subscriber sub_pos, sub_twst;
	ros::Publisher pub;
	
	geometry_msgs::Twist get_control_values(const gazebo_msgs::ModelStates curr)
	{
		
		geometry_msgs::Twist ret_cmd;
		int fid = get_robot_frame_position(curr);
		if (fid == -1) { ROS_WARN("Model not found!\n"); return ret_cmd; }
		
		geometry_msgs::Vector3 euler = ToEulerAngles(curr.pose[fid].orientation);
		
		double A[3][3] = { {1, 0, 0}, {0, cos(goal[0]), sin(goal[0])}, {0, -sin(goal[0]), cos(goal[0])}};
		double err[3] = {-goal[0]+euler.z, -goal[1]+curr.pose[fid].position.x, -goal[2]+curr.pose[fid].position.y};
		double lsterr[3] = {0, 0, 0};
		if(abs(err[1]) < 0.5 and abs(err[2]) < 0.5 and path_point_cnt < path.size()-1)
		{
			path_point_cnt++;
			
			goal[1] = path.at(path_point_cnt).first; 	
			goal[2] = path.at(path_point_cnt).second;	
			goal[0] = atan2(goal[2]-curr.pose[fid].position.y, goal[1]-curr.pose[fid].position.x);
			
			if      (goal[0] <= M_PI and goal[0] > M_PI/2) { goal[0] -= M_PI; }
			else if (goal[0] > -M_PI and goal[0] < -M_PI/2) { goal[0] += M_PI; }
			
			ROS_WARN_STREAM("Goal Change: theta: " << goal[0] << "   x: " << goal[1] << "   y: " << goal[2] << "  cnt:  " << path_point_cnt << "\n");
		}
		

		memcpy(lsterr, err, sizeof(err));

		double qe[3];
	
		for(int i=0; i<3; i++)
		{
			qe[i] += A[i][0]*err[0] + A[i][1]*err[1] + A[i][2]*err[2];		
		}

		//ROS_INFO_STREAM("t: " << (180*err[0]/M_PI) << "   x: " << err[1] << "   y: " << err[2]);
		//ROS_INFO_STREAM("te: " << (180*qe[0]/M_PI) << "   xe: " << qe[1] << "   ye: " << qe[2] << "\nvd: " << vx << "   wd: " << wz << "v: " << ret_cmd.linear.x  << "   w: " << ret_cmd.angular.z );

		wd = curr.twist[fid].angular.z;
		vd = curr.twist[fid].linear.x;

		ret_cmd.linear.x = ( vd - ( k1*abs(vd)*(qe[1] + qe[2]*tan(qe[0])) ) )/cos(qe[0]);
		ret_cmd.linear.y = 0;
		ret_cmd.linear.z = 0;

		ret_cmd.angular.x = 0;
		ret_cmd.angular.y = 0;
		ret_cmd.angular.z = (wd - ( ( (k2*vd*qe[2]) + k3*abs(vd)*tan(qe[0]) ) * ( (1+cos(2*qe[0]))/2 ) ) );

		//ROS_INFO_STREAM("v: " << ret_cmd.linear.x  << "   w: " << ret_cmd.angular.z );
		ROS_INFO_STREAM("\nte: " << (180*qe[0]/M_PI) << "   xe: " << qe[1] << "   ye: " << qe[2] << "\nvd: " << vd << "   wd: " << wd << "   v: " << ret_cmd.linear.x  << "   w: " << ret_cmd.angular.z << "\n" << "gt: " << (180*goal[0]/M_PI) << "   gx:" << goal[1] << "   gy: " << goal[2] << "\n");
		
		

		return ret_cmd;
	}

	void callback_pos(const gazebo_msgs::ModelStates& msg)
	{
		geometry_msgs::Twist cmd = get_control_values(msg);
		pub.publish(cmd);
	}
	
	public:
	inof_robot(int argc, char **argv)
	{
		ros::init(argc, argv, "openspace_mover_node");
		ros::NodeHandle nh;
		
		path.push_back(std::make_pair( 3, 3));
		path.push_back(std::make_pair(-3, 3));
		path.push_back(std::make_pair( 0, 0));
		path.push_back(std::make_pair( 3,-3));
		path.push_back(std::make_pair(-3,-3));
		path.push_back(std::make_pair( 0, 0));
		
		goal[1] = path.at(path_point_cnt).first; 	
		goal[2] = path.at(path_point_cnt).second;	
		goal[0] = atan2(goal[2], goal[1]); 

		if      (goal[0] <= M_PI and goal[0] > M_PI/2) { goal[0] -= M_PI; }
		else if (goal[0] > -M_PI and goal[0] < -M_PI/2) { goal[0] += M_PI; }
		
		pub  = nh.advertise<geometry_msgs::Twist>("/inof_diff_drive_controller/cmd_vel", 10);
		while(pub.getNumSubscribers() == 0) {}
		sub_pos = nh.subscribe("/gazebo/model_states", 10, &inof_robot::callback_pos, this);
	//	sub_twst = nh.subscribe("/inof_diff_drive_controller/odom", 10, &inof_robot::callback_twst, this);
		ros::spin();
	}
};


int main(int argc, char **argv)
{
	inof_robot robot(argc, argv);
	return 1;
}



