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
#include <gazebo_msgs/ModelState.h>
#include <inof_robot/path_points.h>
#include <inof_robot/Pose.h>


class INOF_robot
{
	public:
	INOF_robot (std::string n, geometry_msgs::Point& p) 
	{
		name = n;
		goal[0] = p.z;	goal[1] = p.x; goal[2] = p.y;
		goal[0] = atan2(goal[2], goal[1]);
		
		if      (goal[0] <= M_PI and goal[0] > M_PI/2) { goal[0] -= M_PI; }
		else if (goal[0] > -M_PI and goal[0] < -M_PI/2) { goal[0] += M_PI; }
	}
	
	std::string name;
	double goal[3];
	
	
	//private:
	int path_point_cnt;
	ros::NodeHandle nh;
	ros::Subscriber sub_pos;
	ros::Publisher pub;
	ros::ServiceClient servc;
	std::vector<std::pair<int, int>> path;
	void callback_pos(const gazebo_msgs::ModelStates& msg);
	void set_goalpoint(std::pair<int, int> p, double *goal);
};


void INOF_robot::set_goalpoint(std::pair<int, int> p)
{

}




static std::vector<std::pair<int, int>> filter_path_points(inof_robot::path_points msg)
{
	std::vector<std::pair<int, int>> path;
	int c_theta = 0, p_theta = 0;
	for(int i=1; i<msg.response.path_len; i++)
	{
		int x = msg.response.points[i].x - msg.response.points[i-1].x;
		int y = msg.response.points[i].y - msg.response.points[i-1].y;
		
		c_theta = atan2(y, x);

		if (c_theta != p_theta)
		{
			path.push_back(std::make_pair(msg.response.points[i-1].x, msg.response.points[i-1].y));
		}

		p_theta = c_theta;
	}
	path.push_back(std::make_pair(msg.response.points[msg.response.path_len-1].x, msg.response.points[msg.response.path_len-1].y));
	return path;
}




static geometry_msgs::Vector3 ToEulerAngles(geometry_msgs::Quaternion q) 
{
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





geometry_msgs::Twist get_control_values(const gazebo_msgs::ModelStates c, INOF_robot * robot)
{
	geometry_msgs::Twist ret_cmd;
	//gazebo_msgs::ModelStates curr = c;

	int fid = get_robot_frame_position(c);
	if (fid == -1) 
	{ 
		ROS_WARN("Model not found!\n"); return ret_cmd; 
	}
	
	
	gazebo_msgs::ModelState curr;
	curr.pose = c.pose[fid];
	curr.twist = c.twist[fid];

	double vd = 0.0, wd = 0.0;
	//double k1=10, k2 =3, k3 =16;
	double k1=3, k2 =3, k3 =5;


	geometry_msgs::Vector3 euler = ToEulerAngles(curr.pose.orientation);
	
	double A[3][3] = { {1, 0, 0}, {0, cos(robot->goal[0]), sin(robot->goal[0])}, {0, -sin(robot->goal[0]), cos(robot->goal[0])}};
	double err[3] = {-robot->goal[0]+curr.pose.orientation.z, -robot->goal[1]+curr.pose.position.x, -robot->goal[2]+curr.pose.position.y};
	
	if(abs(err[1]) < 0.15 and abs(err[2]) < 0.15)
	{
		if (robot->path_point_cnt < robot->path.size())
		{
			robot->goal[1] = (robot->path.at(robot->path_point_cnt).second)*15.0/100.0; 	//X-axis is second element in pair which corresponds to column in map-array
			robot->goal[2] = (robot->path.at(robot->path_point_cnt).first)*15.0/100.0;	// Vice versa


			if      (robot->goal[0] <= M_PI and robot->goal[0] > M_PI/2) { robot->goal[0] -= M_PI; }
			else if (robot->goal[0] > -M_PI and robot->goal[0] < -M_PI/2) { robot->goal[0] += M_PI; }
		
			robot->path_point_cnt++;
		
			ROS_WARN_STREAM("Goal Change: theta: " << robot->goal[0] << "   x: " << robot->goal[1] << "   y: " << robot->goal[2] << "  cnt:  " << robot->path_point_cnt << "\n");
		}
		else
		{
			err[0] = 0;	err[1] = 0;	err[2] = 0;
		}
	}
	
	robot->goal[0] = atan2(robot->goal[2]-curr.pose.position.y, robot->goal[1]-curr.pose.position.x);

	double qe[3];

	for(int i=0; i<3; i++)
	{
		qe[i] += A[i][0]*err[0] + A[i][1]*err[1] + A[i][2]*err[2];		
	}

	wd = curr.twist.angular.z;
	vd = curr.twist.linear.x;

	ret_cmd.linear.x = ( vd - ( k1*abs(vd)*(qe[1] + qe[2]*tan(qe[0])) ) )/cos(qe[0]);
	ret_cmd.linear.y = 0;
	ret_cmd.linear.z = 0;

	ret_cmd.angular.x = 0;
	ret_cmd.angular.y = 0;
	ret_cmd.angular.z = (wd - ( ( (k2*vd*qe[2]) + k3*abs(vd)*tan(qe[0]) ) * ( (1+cos(2*qe[0]))/2 ) ) );

	//ROS_INFO_STREAM("v: " << ret_cmd.linear.x  << "   w: " << ret_cmd.angular.z );
	//ROS_INFO_STREAM("\nte: " << (180*qe[0]/M_PI) << "   xe: " << qe[1] << "   ye: " << qe[2] << "\nvd: " << vd << "   wd: " << wd << "   v: " << ret_cmd.linear.x  << "   w: " << ret_cmd.angular.z << "\n" << "gt: " << (180*robot->goal[0]/M_PI) << "   gx:" << robot->goal[1] << "   gy: " << robot->goal[2] << "\n");
	
	
	ROS_INFO_STREAM("\nte: " << (180*err[0]/M_PI) << "   xe: " << err[1] << "   ye: " << err[2] << "\n" << "gt: " << (180*robot->goal[0]/M_PI) << "   gx:" << robot->goal[1] << "   gy: " << robot->goal[2] << "\n" << "ct: " << (180*curr.pose.orientation.z/M_PI) << "   cx: " << curr.pose.position.x << "   cy: " << curr.pose.position.y << "\nCount: " << robot->path_point_cnt);

	return ret_cmd;
}



void INOF_robot::callback_pos(const gazebo_msgs::ModelStates& msg)
{
	geometry_msgs::Twist cmd = get_control_values(msg, this);
	pub.publish(cmd);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "app_inof_node");
		
	float grid_fac = 15.0;
	float unit_fac = 0.01;

	geometry_msgs::Point src,des;
	src.x = 150;	des.x = 240;
	src.y = 90;	des.y = 255;
	src.z = 0;	des.z = 0;

	inof_robot::path_points srvmsg;
	srvmsg.request.pose_start.x = src.y/grid_fac;
	srvmsg.request.pose_start.y = src.x/grid_fac;
	srvmsg.request.pose_start.t = 0;
	srvmsg.request.pose_end.x = des.y/grid_fac;
	srvmsg.request.pose_end.y = des.x/grid_fac;
	srvmsg.request.pose_end.t = 0;
	
	geometry_msgs::Point p;
	p.x = src.x*unit_fac;	// X-axis in gazebo is column for array
	p.y = src.y*unit_fac;	// Vice versa
	p.z = src.z*unit_fac;
	
	INOF_robot app("inof1", p);
	
	app.servc = app.nh.serviceClient<inof_robot::path_points>("path_planning_srvc");
	app.servc.waitForExistence();
	
	app.pub = app.nh.advertise<geometry_msgs::Twist>("/inof_diff_drive_controller/cmd_vel", 10);
	while(app.pub.getNumSubscribers() == 0) {}

	if(app.servc.call(srvmsg))
	{
		if(srvmsg.response.path_len)
		{
			ROS_INFO("Path available");
			app.path = filter_path_points(srvmsg);
			app.path_point_cnt = 0;
			ROS_INFO_STREAM("Total Path Points: " << srvmsg.response.path_len << std::endl << "Total Filtered Points: " << app.path.size());
			
			for(auto x:app.path)
				ROS_INFO_STREAM("Path points are: " << x.first << "," << x.second);



			app.sub_pos = app.nh.subscribe("/gazebo/model_states", 10, &INOF_robot::callback_pos, &app);
			ros::spin();

		}
		else
		{
			ROS_INFO("Destination out of reach");
		}
	}
}
