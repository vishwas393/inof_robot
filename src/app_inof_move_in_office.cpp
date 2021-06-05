#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <inof_robot/path_points.h>
#include <inof_robot/Pose.h>

std::vector<std::pair<float, float>> filter_path_points(inof_robot::path_points msg)
{
	std::vector<std::pair<float, float>> path;
	double c_theta = 0, p_theta = 0;
	for(int i=1; i<msg.response.path_len; i++)
	{
		double x = msg.response.points[i].x - msg.response.points[i-1].x;
		double y = msg.response.points[i].y - msg.response.points[i-1].y;
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



int main(int argc, char** argv)
{
	ros::init(argc, argv, "app_inof_node");
	ros::NodeHandle nh;

	ros::ServiceClient servc = nh.serviceClient<inof_robot::path_points>("path_planning_srvc");
	servc.waitForExistence();

	inof_robot::path_points srvmsg;
	srvmsg.request.pose_start.x = 235;
	srvmsg.request.pose_start.y = 15;
	srvmsg.request.pose_start.t = 0;
	srvmsg.request.pose_end.x = 350;
	srvmsg.request.pose_end.y = 15;
	srvmsg.request.pose_end.t = 0;

	if(servc.call(srvmsg))
	{
		if(srvmsg.response.path_len)
		{
			ROS_INFO("Path available");
			std::vector<std::pair<float, float>> f_path = filter_path_points(srvmsg);
			ROS_INFO_STREAM("Total Path Points: " << srvmsg.response.path_len << std::endl << "Total Filtered Points: " << f_path.size());

		}
		else
		{
			ROS_INFO("Destination out of reach");
		}
	}
}
