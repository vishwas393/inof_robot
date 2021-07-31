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
#include <inof_robot/PathPoints.h>
#include <inof_robot/Pose.h>


//grid factors. one block of 15x15 cm. unit_fac for converting centimeter-->meter.
float grid_fac = 15.0;
float unit_fac = 0.01;


class INOF_robot
{
	public:
	INOF_robot (std::string n) 
	{
		name = n;
	}
	
	std::string name;
	double goal[3];
	inof_robot::Pose src,des;
	
	int path_point_cnt;
	ros::NodeHandle nh;
	ros::Subscriber sub_pos, tmpGUISub;
	ros::Publisher pub, pathPub;
	ros::ServiceClient servc;
	std::vector<std::pair<int, int>> path;
	void callback_pos(const gazebo_msgs::ModelStates& msg);
	void set_goal_angle(geometry_msgs::Pose p);
	void set_goal_point(const inof_robot::Pose& p);
};


/*
 * Brief: Setting the goal variable with data received from GUI
 *
 */
void INOF_robot::set_goal_point(const inof_robot::Pose& p)
{
	src.x = 10*15;	des.x = p.x*15;
	src.y =  6*15;	des.y = p.y*15;
	src.t = 0;	des.t = 0;

	goal[0] = float(src.t)*0.01;	goal[1] = float(src.x)*0.01; goal[2] = float(src.y)*0.01;
	goal[0] = atan2(goal[2], goal[1]);

	ROS_INFO_STREAM(src.x << "," << src.y << "," << des.x << "," << des.y);
	//This Feed-Forward Algorithm has limitation on angle-error. -pi/2 < angle-error < pi/2
	if      (goal[0] <= M_PI and goal[0] > M_PI/2) { goal[0] -= M_PI; }
	else if (goal[0] > -M_PI and goal[0] < -M_PI/2) { goal[0] += M_PI; }
}

/*
 * Brief: Setting the goal-angle
 *
 * */
void INOF_robot::set_goal_angle(geometry_msgs::Pose p)
{

	double m1 = 1;
	double m2 = (p.position.y - goal[2]) / (p.position.x - goal[1]);

	if (m1*m2 == -1) { 
		goal[0] = M_PI/2; 
	}
	else {
		goal[0] = atan(abs(m2));
	}
}



/*
 * Brief: Filters path point. Removes the points from the path-vector which are straight in the line.
 * 	  Points where there is a turn(difference in angle) are only kept, rest are removed.
 */
static std::vector<std::pair<int, int>> filter_path_points(inof_robot::path_points msg)
{
	std::vector<std::pair<int, int>> path;
	double c_theta = 0, p_theta = 0;


	for(int i=1; i<msg.response.path.path_len; i++)
	{
		int x = msg.response.path.points[i].x - msg.response.path.points[i-1].x;
		int y = msg.response.path.points[i].y - msg.response.path.points[i-1].y;
		
		c_theta = atan2(y, x);

		if (c_theta != p_theta)
		{
			path.push_back(std::make_pair(msg.response.path.points[i-1].x, msg.response.path.points[i-1].y));

		}

		p_theta = c_theta;
	}
	path.push_back(std::make_pair(msg.response.path.points[msg.response.path.path_len-1].x, msg.response.path.points[msg.response.path.path_len-1].y));
	return path;
}



/*
 * Brief: Converts Quaternion to Euler
 */
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


/*
 * Brief: Returns element-position of robot-model in ModelStates array
 */
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
	//New twist-command which would be returned and publish
	geometry_msgs::Twist ret_cmd;

	//As ModelStates returns data for every model in an array form, retrive position of robot's data in the array
	int fid = get_robot_frame_position(c);
	if (fid == -1) 
	{ 
		ROS_WARN("Model not found!\n"); return ret_cmd; 
	}
	
	
	gazebo_msgs::ModelState curr;
	curr.pose = c.pose[fid];
	curr.twist = c.twist[fid];

	//Convert Robots Orientation from Quaternion to Euler Angle
	geometry_msgs::Vector3 euler = ToEulerAngles(curr.pose.orientation);
	
	//Matrices from the Feed-Forward Algorithm
	double A[3][3] = { {1, 0, 0}, {0, cos(robot->goal[0]), sin(robot->goal[0])}, {0, -sin(robot->goal[0]), cos(robot->goal[0])}};
	double err[3] = {-robot->goal[0]+euler.z, -robot->goal[1]+curr.pose.position.x, -robot->goal[2]+curr.pose.position.y};
	
	//Due limitation of control algorithm, error can not be absolutely zero, an error of 0.3 is assumed okay.
	//Once error is in acceptable range, next point in the retrived path (from path planner) is set as new goal-point.
	if(abs(err[1]) < 0.3 and abs(err[2]) < 0.3)
	{
		//If it is not the last point in path(distination), enter.
		if (robot->path_point_cnt < robot->path.size())
		{
			//goal = [angle, X-poisiton, Y-position];
			robot->goal[1] = (robot->path.at(robot->path_point_cnt).second)*15.0/100.0; 	//X-axis is second element in pair which corresponds to column in map-array
			robot->goal[2] = (robot->path.at(robot->path_point_cnt).first)*15.0/100.0;	// Vice versa
		
			double tmpX = 0, tmpY = 0;

			if(robot->path_point_cnt>0){
			tmpX = (robot->path.at(robot->path_point_cnt-1).second)*15.0/100.0; 	
			tmpY = (robot->path.at(robot->path_point_cnt-1).first)*15.0/100.0;	
			}

			robot->goal[0] = atan2(robot->goal[2]-tmpY, robot->goal[1]-tmpX);

			robot->path_point_cnt++;
			ROS_WARN_STREAM("Goal Change: theta: " << robot->goal[0] << "   x: " << robot->goal[1] << "   y: " << robot->goal[2] << "  cnt:  " << robot->path_point_cnt << "\n");
		}
	}
	
	double k1= 10000, k2 =30000, k3 = 30000;

	double qe[3];

	//From algorithm
	for(int i=0; i<3; i++)
	{
		qe[i] += A[i][0]*err[0] + A[i][1]*err[1] + A[i][2]*err[2];		
	}


	//Setting linear and angular velocity variable to current velocities fetched from gazebo's topic
	double wd = curr.twist.angular.z;
	double vd = curr.twist.linear.x;



	//New velocity according to the algorithm
	ret_cmd.linear.x = ( vd - ( k1*abs(vd)*(qe[1] + qe[2]*tan(qe[0])) ) )/cos(qe[0]);
	ret_cmd.linear.y = 0;
	ret_cmd.linear.z = 0;

	ret_cmd.angular.x = 0;
	ret_cmd.angular.y = 0;
	ret_cmd.angular.z = (wd - ( ( (k2*vd*qe[2]) + k3*abs(vd)*tan(qe[0]) ) * (pow(cos(qe[0]),2) ) ) );




	ROS_INFO_STREAM("\nwd: " << wd << "   vd: " << vd << "\nte: " << (180*err[0]/M_PI) << "   xe: " << err[1] << "   ye: " << err[2] << "\n" << "gt: " << (180*robot->goal[0]/M_PI) << "   gx:" << robot->goal[1] << "   gy: " << robot->goal[2] << "\n" << "ct: " << (180*euler.z/M_PI) << "   cx: " << curr.pose.position.x << "   cy: " << curr.pose.position.y << "\nCount: " << robot->path_point_cnt);


	//New twist command returned and will be published in fn:callback_pos().
	return ret_cmd;
}



void INOF_robot::callback_pos(const gazebo_msgs::ModelStates& msg)
{
	//From current data of robot(position and twist) generate new command(twist)
	geometry_msgs::Twist cmd = get_control_values(msg, this);
	pub.publish(cmd);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "app_inof_node");

	INOF_robot app("inof1");

	inof_robot::Pose gp;
	gp = *(ros::topic::waitForMessage<inof_robot::Pose>("/inof/gui_goal_points"));
	app.set_goal_point(gp);	
	
	//Path planner is started as service and srvmsg is sent as request with src and goal points
	//It will be returned with the path points as an array
	inof_robot::path_points srvmsg;
	srvmsg.request.pose_start.x = app.src.y/grid_fac;		// X-axis in gazebo is column for array
	srvmsg.request.pose_start.y = app.src.x/grid_fac;		// Vice versa
	srvmsg.request.pose_start.t = 0;
	srvmsg.request.pose_end.x = app.des.y/grid_fac;
	srvmsg.request.pose_end.y = app.des.x/grid_fac;
	srvmsg.request.pose_end.t = 0;
	app.servc = app.nh.serviceClient<inof_robot::path_points>("path_planning_srvc");
	app.servc.waitForExistence();
	

	app.pathPub = app.nh.advertise<inof_robot::PathPoints>("/inof/generated_path", 5);
	
	//Subscribing to robot's URDF model's command-topic
	app.pub = app.nh.advertise<geometry_msgs::Twist>("/inof_diff_drive_controller/cmd_vel", 10);
	while(app.pub.getNumSubscribers() == 0) {}

	//Calling path planner service
	if(app.servc.call(srvmsg))
	{
		//Publishing path for GUI
		app.pathPub.publish(srvmsg.response.path);
		
		//If path available
		if(srvmsg.response.path.path_len)
		{
			ROS_INFO("Path available");
			
			//Path points are returned with each point on grid. This function only keeps points where there is any turn.
			//Points which lies in a straight line is discarded except start and end of the line.
			app.path = filter_path_points(srvmsg);	 
			app.path_point_cnt = 0;
			ROS_INFO_STREAM("Total Path Points: " << srvmsg.response.path.path_len << std::endl << "Total Filtered Points: " << app.path.size());
			
			for(auto x:app.path)
				ROS_INFO_STREAM("Path points are: " << x.first << "," << x.second << "    (" << double(x.first)*0.15 << "," << double(x.second)*0.15 << ")");


			//Subscribes to the gazebo's topic->model_states to get values of position and twist of robot. With callback fn: callback_pos()
			app.sub_pos = app.nh.subscribe("/gazebo/model_states", 10, &INOF_robot::callback_pos, &app);
			ros::spin();

		}
		else
		{
			ROS_INFO("Destination out of reach");
		}
	}
}
