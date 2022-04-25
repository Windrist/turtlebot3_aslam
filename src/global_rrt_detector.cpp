#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"

#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


// Global variables
nav_msgs::OccupancyGrid mapData, costmapData;
visualization_msgs::Marker points, line;
float xdim, ydim, resolution;

rdm r; // for genrating random numbers


// Subscribers Callback Functions
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	mapData = *msg;
}

void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	costmapData = *msg;
	resolution = float(msg->info.resolution);
	xdim = float(msg->info.width);
	ydim = float(msg->info.height);
}

int main(int argc, char **argv) {

	unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
	MTRand_int32 irand(init, length); // 32-bit int generator
	// This is an example of initializing by an array
	// you may use MTRand(seed) with any 32bit integer
	// as a seed for a simpler initialization
	MTRand drand; // double in [0, 1) generator, already init

	// Generate the same numbers as in the original C test program
	ros::init(argc, argv, "global_rrt_frontier_detector");
	ros::NodeHandle nh;
	
	// Fetching all parameters
	float eta;
	std::string map_topic, costmap_topic, base_frame_topic;
	
	std::string ns;
	ns = ros::this_node::getName();

	ros::param::param<float>(ns + "/eta", eta, 1.0);
	ros::param::param<std::string>(ns + "/map_topic", map_topic, "/map");
	ros::param::param<std::string>(ns + "/costmap_topic", costmap_topic, "/move_base/global_costmap/costmap");
	ros::param::param<std::string>(ns + "/robot_frame", base_frame_topic, "/base_link"); 

	//---------------------------------------------------------------
	ros::Subscriber sub = nh.subscribe(map_topic, 100, mapCallBack);
	ros::Subscriber costmap_sub = nh.subscribe(costmap_topic, 100, costmapCallBack);

	ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);

	ros::Rate rate(100);
	
	// Wait until map is received, when a map is received, mapData.header.seq will not be < 1  
	while (mapData.header.seq < 1 or mapData.data.size() < 1) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	while (costmapData.data.size() < 1) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	// Visualizations points and lines..
	points.header.frame_id = mapData.header.frame_id;
	line.header.frame_id = mapData.header.frame_id;
	points.header.stamp = ros::Time(0);
	line.header.stamp = ros::Time(0);
		
	points.ns = line.ns = "markers";
	points.id = 0;
	line.id = 1;

	points.type = points.POINTS;
	line.type = line.LINE_LIST;

	// Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points.action = points.ADD;
	line.action = line.ADD;
	points.pose.orientation.w = 1.0;
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.03;
	line.scale.y = 0.03;
	points.scale.x = 0.3; 
	points.scale.y = 0.3; 

	line.color.r = 9.0 / 255.0;
	line.color.g = 91.0 / 255.0;
	line.color.b = 236.0 / 255.0;
	points.color.r = 255.0 / 255.0;
	points.color.g = 0.0 / 255.0;
	points.color.b = 0.0 / 255.0;
	points.color.a = 1.0;
	line.color.a = 1.0;
	points.lifetime = ros::Duration();
	line.lifetime = ros::Duration();

	geometry_msgs::Point p;

	float init_map_x = xdim * resolution;
	float init_map_y = ydim * resolution;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	try {
		listener.waitForTransform(map_topic, base_frame_topic, ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform(map_topic, base_frame_topic, ros::Time(0), transform);
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	
	std::vector<std::vector<float>> V;
	std::vector<float> xnew;
	xnew.push_back(transform.getOrigin().x());
	xnew.push_back(transform.getOrigin().y());
	V.push_back(xnew);

	float xr, yr;
	std::vector<float> x_rand, x_nearest, x_new, x_current;
	geometry_msgs::PointStamped exploration_goal;

	// Main Loop
	while (ros::ok()) {
		listener.lookupTransform(map_topic, base_frame_topic, ros::Time(0), transform);
		x_current.push_back(transform.getOrigin().x());
		x_current.push_back(transform.getOrigin().y());


		// Sample Free Point
		xr = (drand() - 0.5) * init_map_x + transform.getOrigin().x();
		yr = (drand() - 0.5) * init_map_y + transform.getOrigin().y();

		x_rand.clear();
		x_rand.push_back(xr);
		x_rand.push_back(yr);

		// Nearest
		x_nearest = Nearest(V, x_rand);

		// Steer
		x_new = Steer(x_nearest, x_rand, eta);

		// obstacleFree
			// 1: Free
			// -1: Unnkown (Frontier Region)
			// 0: Obstacle
		//
		char checking = obstacleFree(x_nearest, x_new, mapData);
		if (checking == -1) {
			exploration_goal.header.stamp = ros::Time(0);
			exploration_goal.header.frame_id = mapData.header.frame_id;
			exploration_goal.point.x = x_new[0];
			exploration_goal.point.y = x_new[1];
			exploration_goal.point.z = 0.0;

			points.points.push_back(exploration_goal.point);
			pub.publish(points);
			targetspub.publish(exploration_goal);

			points.points.clear();
		}
		if (checking == 1) {
			V.push_back(x_new);

			p.x = x_new[0]; 
			p.y = x_new[1]; 
			p.z = 0.0;
			line.points.push_back(p);
			p.x = x_nearest[0]; 
			p.y = x_nearest[1]; 
			p.z = 0.0;
			line.points.push_back(p);
		}
		else {
			init_map_x = xdim * resolution;
			init_map_y = ydim * resolution;
		}
		pub.publish(line);
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
