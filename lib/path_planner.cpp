#include <algorithm>
#include <vector>
#include "ros/ros.h"
#include "inof_robot/Pose.h"
#include "inof_robot/path_points.h"
#include "../include/inof_robot/a_star_search_algo.h"

bool plan_path(inof_robot::path_points::Request &req, inof_robot::path_points::Response &res)
{
	std::pair<int, int> src, dest;
	
	src  = std::make_pair(req.pose_start.x, req.pose_start.y);
	dest = std::make_pair(req.pose_end.x, req.pose_end.y);
		
	std::vector<node> path = path_planning_fn(src, dest);
	if(path.size() != 0)
	{
		std::cout << "Total nodes: " << path.size() << std::endl;
		std::vector<inof_robot::Pose> ret_arr;
		for(int i=0; i<path.size(); i++)
		{
			inof_robot::Pose p;
			p.x = path.at(i).coords.first;
			p.y = path.at(i).coords.second;
			p.t = 0;
			ret_arr.push_back(p);
		}
		res.points = ret_arr;
		res.path_avl = true;
	}
	else {
		res.path_avl = false;
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning_server");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("path_planning_srvc", plan_path);
	ROS_INFO("Path_Planning Service Started Successfully!");
	ros::spin();

	return 0;
}

