#ifndef GLOBAL_COLLISION_AVOIDANCE_H
#define GLOBAL_COLLISION_AVOIDANCE_H

#include <iostream>
#include <ctime>

// ROS libraries
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h> // for getting file path of package

// Boost libraries
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// moveit_msgs
#include <moveit_msgs/RobotTrajectory.h> //for the rosbag
#include <moveit/robot_trajectory/robot_trajectory.h> 
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// custom libraries
#include "node.h"
#include "collisionCheck.h"
#include "AStarSearchNodes.h"
#include "demo1/NeighborTrajectory.h"//the rosbag msg

// ---------------------------------------------------------------------
// Class definition
// ---------------------------------------------------------------------
class GlobalCollisionAvoidance
{
	private:
		typedef std::vector<std::vector<double> > DistanceMap;
		std::clock_t time;
		std::vector<Node> node_list_;
		std::vector<int> astar_path_;
		DistanceMap distances_;
		moveit::planning_interface::MoveGroup group_;
		moveit::planning_interface::MoveGroup::Plan motion_plan_;
		std::string trajectory_file;
		robot_trajectory::RobotTrajectory multi_trajectory_;
      CollisionCheck& collisionCheck_;
      int start = 0;
      int goal = 1;
		
		void goHome()
		{
			group_.setNamedTarget("up");  // home, resting, up
			group_.move();
		}

		robot_trajectory::RobotTrajectory newTrajectory()
		{
			return robot_trajectory::RobotTrajectory(group_.getCurrentState()->getRobotModel(), group_.getName());
		}		
		
		robot_state::RobotState getCurrentRobotState()
		{
			return *group_.getCurrentState();
		}
		
		/*
		* \brief When the robot starts up, it is left in a position that might not be the starting node. 
		* 			Before moving from the starting node to the goal node, we make sure that it starts off properly
		* 			at the starting node. 
		* \param from - the target node
		* \param go_to - the target node
		*/ 
		robot_trajectory::RobotTrajectory planFromTo(int from, int go_to)
		{
			//add the trajectory from the current state to the starting state
			robot_trajectory::RobotTrajectory from_to_trajectory = newTrajectory();
			
			robot_state::RobotState start_state(group_.getCurrentState()->getRobotModel());
			start_state.setJointGroupPositions(group_.getName(), node_list_[from].getJointPos());
			
			group_.setStartState(start_state);
			group_.setJointValueTarget(node_list_[go_to].getJointPos());
			
			moveit::planning_interface::MoveGroup::Plan from_to_plan;
			if(group_.plan(from_to_plan))
			{
				from_to_trajectory.setRobotTrajectoryMsg(getCurrentRobotState(), from_to_plan.trajectory_);
				return from_to_trajectory;	
			}	
		}
		
		
		/*
		* \brief Move to the target node. Best used when the robot starts up 
		* \param go_to - the target node
		*/ 
		void moveFromCurrentState(int go_to)
		{
			//add the trajectory from the current state to the starting state
			group_.setStartState(getCurrentRobotState());
			group_.setJointValueTarget(node_list_[go_to].getJointPos());
			
			moveit::planning_interface::MoveGroup::Plan move_plan;
			if(group_.plan(move_plan))
				group_.execute(move_plan);
		}
		
		
		/*
		* \brief After their connections are decided, with the nearest neighbors already known
		*			create a lookup table consisting of each neighbor and the robot_trajectory to get there.
		*/ 
		void setNeighborTrajectory()
		{
			int count=0;
			double percent;

			for(int a = 0; a < node_list_.size(); ++a)
			{
				std::vector<int> neighbors = node_list_[a].getNN();
				for(int b = 0; b < neighbors.size(); ++b)
				{
			  		node_list_[a].populate_traj_list(neighbors[b], planFromTo(a, neighbors[b]));
				}

			  	count++;
			  	percent = 100* count / double(node_list_.size());
				std::cout<< percent<<" percent complete ... "<<std::endl;
			}

			
		}
		
 		//checks the location where the bag file is intended to be
		bool checkForBagFile()
		{
			return boost::filesystem::exists(trajectory_file);
		}
		
		/**
			* \brief Access the trajectory bag and either read or write the neighbor trajectories
			* \param to_read - whether you would like to read, corresponds to the existance of the bag file
			*/
		void accessBag(bool to_read)
		{
			rosbag::Bag trajectory_bag;
			//READ
			if(to_read)
			{
				int from, to;
				robot_trajectory::RobotTrajectory read_traj = newTrajectory();
				trajectory_bag.open(trajectory_file, rosbag::bagmode::Read);
				rosbag::View view(trajectory_bag, rosbag::TopicQuery("trajectory"));
				
				std::cout << "Reading Bag... " << std::endl;
				time = std::clock();
				BOOST_FOREACH(const rosbag::MessageInstance traj_msg, view) 
				{
					demo1::NeighborTrajectory n_traj_read = *(traj_msg.instantiate<demo1::NeighborTrajectory>());
					int from_node = n_traj_read.to;
					int to_node = n_traj_read.from;
					read_traj.setRobotTrajectoryMsg(getCurrentRobotState(), n_traj_read.robot_trajectory);
					read_traj.reverse(); //the trajectories were written in the reverse order that we need to read
					node_list_[from_node].populate_traj_list(to_node, read_traj);
				}
				time = std::clock() - time;
				std::cout << "...Reading Completed in " << boost::to_string(((float)time)/CLOCKS_PER_SEC) << " seconds" << std::endl;
			}
			//WRITE
			else
			{
				std::cout << "Making Trajectories ... " << std::endl;
				setNeighborTrajectory();
				std::cout << "... Trajectory generation complete. " << std::endl;

				std::cout << " Writing into Bag... " << std::endl;
				trajectory_bag.open(trajectory_file, rosbag::bagmode::Write);
				time = std::clock();
				for(int q = 0; q < node_list_.size(); q++)
				{
					std::vector<int> checkNeighbor = node_list_[q].getNN();
					for(int w = 0; w < checkNeighbor.size(); w++)
					{
						demo1::NeighborTrajectory n_traj_write;
						n_traj_write.from = q;
						n_traj_write.to = checkNeighbor[w];
				
						robot_trajectory::RobotTrajectory node_traj = node_list_[q].get_trajectory_to_node(checkNeighbor[w]);
						node_traj.getRobotTrajectoryMsg(n_traj_write.robot_trajectory);
						trajectory_bag.write("trajectory", ros::Time::now(), n_traj_write);
					}
				}
				time = std::clock() - time;
				std::cout << "... Writing Completed in " << boost::to_string(((float)time)/CLOCKS_PER_SEC) << " seconds" << std::endl;
			}
			trajectory_bag.close();
		}
		
		
		/**
		* \brief Function that samples multiple nodes, populates the node_list and distances__map
		*/
		void jointSpaceSampler(int points, int connections)
		{
			//DOESNT USE THE LIMIT FOR NUMBER OF POINTS YET
			std::vector<double> base_joint_sample;
			std::cout << "Making Sample Points... " << std::endl;
			base_joint_sample = {0, -1.57, -3.14, 1.57};
			
			time = std::clock();
			int add = node_list_.size();
			for(int sample = 0; sample < base_joint_sample.size(); sample++)
			{
				node_list_.push_back(Node(base_joint_sample[sample], -2.35, -0.785, -1.57, 1.57, 0));
				node_list_[3*sample + add].setNodeID(3*sample + add);
		
				node_list_.push_back(Node(base_joint_sample[sample], -1.963, -0.785, -1.963, 1.57, 0));
				node_list_[3*sample + 1 + add].setNodeID(3*sample + 1 + add);
		
				node_list_.push_back(Node(base_joint_sample[sample], -1.57, -0.785, -2.35, 1.57, 0));
				node_list_[3*sample + 2 + add].setNodeID(3*sample + 2 + add);
			}
			time = std::clock() - time;
			std::cout << "...Done in " << boost::to_string(((float)time)/CLOCKS_PER_SEC) << " seconds" << std::endl;
			
			int size = node_list_.size();
			typedef std::pair<int, double> ID_Distance;
	
			//Initialize the distance map and fill it out while finding the nearest neighbors
			DistanceMap distances_map(size, std::vector<double>(size, 100));
			for(int i = 0; i < size; i++) //rows
			{
				std::vector<ID_Distance> id_distance(connections, std::make_pair(-1, 100));
				distances_map[i][i] = 0.0;
				for(int j = 0; j < size; j++) //columns
				{
					if(i != j)
					{	
						double node_dist = node_list_[i].distance(node_list_[j].getJointPos());
						for(int k = 0; k < connections; k++)
						{
							if(node_dist < id_distance[k].second)
							{
								for(int l = (connections - 1); l > k; --l)
									id_distance[l] = std::make_pair(id_distance[l-1].first, id_distance[l-1].second);
							
								id_distance[k] = std::make_pair(j, node_dist);
								break;
							}
						}
					}
				}
				//some repetitions => check for this
				for(int m = 0; m < connections; m++)
				{
					int check_node_id = id_distance[m].first;
					double check_node_dist = id_distance[m].second;
			
					if(!node_list_[i].checkNeighbor(node_list_[check_node_id].getNodeID()))
						node_list_[i].setNeighbor(check_node_id);
					if(!node_list_[check_node_id].checkNeighbor(node_list_[i].getNodeID()))
						node_list_[check_node_id].setNeighbor(i);	
		
					distances_map[i][check_node_id] = check_node_dist;
				}
			}
			distances_ = distances_map;
			
			/*for(int f = 0; f < size; f++)
			{
				std::vector<int> curNeighbors = node_list_[f].getNN();
				std::cout << f << ": ";
				for(int g = 0; g < curNeighbors.size(); g++)
					std::cout << curNeighbors[g] << " ";
				std::cout << std::endl;
			}*/
		}
		
			
		/*
		* \brief Once the neighboring trajectories are saved within each Node instance, use the 	
		*			shortest path calculated from the A* Search and append each RobotTrajectory to make 
		*			the robot's final path to be executed
		* \param startID - the beginning node
		* \param goalID - the goal node
		* \return vector of int IDs corresponding to the shortest path. This will later be used to check for collisions
		*/ 
		bool makeMotionPlan(int startID, int goalID, DistanceMap redirected_map)
		{
			AStarSearchNodes searchNodes = AStarSearchNodes(node_list_, redirected_map);
		    astar_path_ = searchNodes.findPath(startID, goalID);

		    bool path_found = true;

		    if(astar_path_.size() ==0 )
		    {
		    	path_found = false;
		    	return path_found;
		    }

		    multi_trajectory_ = newTrajectory();
		 	for(int spi = 0; spi < (astar_path_.size() - 1); spi++)
		 	{
		 		//get shortest path node ids, then get those corresponding RobotTrajectory objects
		 		Node checkNode = node_list_[astar_path_[spi]];
		 		robot_trajectory::RobotTrajectory append_traj = checkNode.get_trajectory_to_node(astar_path_[spi + 1]);
				//multi_trajectory_.append(appendTraj, appendTraj.getWaypointDurationFromStart(appendTraj.getWayPointCount()));
				for(std::size_t point = 1; point < append_traj.getWayPointCount(); point++)
				{
					robot_state::RobotState append_state = append_traj.getWayPoint(point);
					multi_trajectory_.insertWayPoint(multi_trajectory_.getWayPointCount(), append_state, append_traj.getWayPointDurationFromPrevious(point));
				}
			}

			return path_found;	
		}
		
		/*
		* \brief Using the shortest path, check each appended trajectory for a collision. If one is found
		*			then make that path unreachable
		* \return true if there is a collision, and vice versa.
		*/ 
		bool checkGlobalCollision(DistanceMap &redirected_map)
		{
			for(int x = 0; x < (astar_path_.size() - 1); x++)
			{
				int from = astar_path_[x];
				int to = astar_path_[x + 1];
				Node check_node = node_list_[from];
				robot_trajectory::RobotTrajectory check_traj = check_node.get_trajectory_to_node(to);
				
				#pragma omp parallel for
				for(std::size_t point = 0; point < check_traj.getWayPointCount(); point++)
				{
					if(collisionCheck_.checkCollision(check_traj.getWayPoint(point)))
					{
						redirected_map[from][to] = 100;
						redirected_map[to][from] = 100;
						std::cout << "Collision between " << from << " => " << to << std::endl;
						
						return true;
					}
				}
			}
			return false;
		}
	
		//for the new_points, the order matters
		//the first point should be the start, the second should be a goal
		//		the rest of the new points are the intermediates
		void addNewPoints(std::vector<std::vector<double> > new_points)
		{
			for(int i = 0; i < new_points.size(); i++) //loop through new points
			{
				bool eq = false;
				Node new_node = Node(new_points[i]);
				for(int n = 0; n < node_list_.size(); n++)
				{
					if(node_list_[n].equals(new_node.getJointPos()))
					{
						if(i == 0)
							start = n;
						else if(i == 1)
							goal = n;
						else
						{	
							new_node.setNodeID(node_list_.size());
							node_list_.push_back(new_node);
						}
						eq = true;
					}
				}
				//the new node is not recognized and should be added
				if(!eq)
				{
					new_node.setNodeID(node_list_.size());
					node_list_.push_back(new_node);
				}
			}
		}
			
    public:
	   /*
	   * INITIALIZE ALL THE GLOBAL VARIABLES
	   * \param gr - in order for this to execute and share information with other ros-based
	   *  		     classes, they must share the same MoveGroup instance of the robot
		* \param collisionCheck - the collisionCheck object that checks the environment for obstacles
	   */
		GlobalCollisionAvoidance(moveit::planning_interface::MoveGroup& gr, CollisionCheck& collisionCheck):
		    group_(gr), collisionCheck_(collisionCheck),
		    multi_trajectory_(group_.getCurrentState()->getRobotModel(), group_.getName())
		{
			// We can print the name of the reference frame for this robot.
			ROS_INFO("Reference frame: %s", group_.getPlanningFrame().c_str());

			// We can also print the name of the end-effector link for this group.
			ROS_INFO("Reference frame: %s", group_.getEndEffectorLink().c_str());

			ros::WallDuration sleep_time(2.0);
			 
			// where to save trajectory files
			const std::string package_path_(ros::package::getPath( "demo1" ) + "/");
			trajectory_file = package_path_ + "src/trajectory_bag.bag";
		}
		
	 	// \brief Set the collision objects, create the nodes and connect them
		void buildEnvironment(int num_points, int degree, std::vector<std::vector<double> > initial_points)
		{
			jointSpaceSampler(num_points, degree);
			addNewPoints(initial_points); //the user may input the new start and goal states
			accessBag(checkForBagFile());	
		}
		
		// \brief Check for collisions. If there is one, correct the path and check until you get one
		//robot_trajectory::RobotTrajectory execute(int start, int goal)
		void globalPlan(moveit::planning_interface::MoveGroup::Plan &ready_to_execute)
		{
			DistanceMap redirected(distances_);
			bool path_found = makeMotionPlan(start, goal, redirected);
			//BOOST SENDS ERROR IF NO PATH FOUND
			while(checkGlobalCollision(redirected))
			{
				//there was a collision and the connections were altered temporarily
				path_found = makeMotionPlan(start, goal, redirected);

				if(path_found == false)
				{
					ROS_WARN("No path found, waiting ... ");
					sleep(5.0);
				}
			}
			//plan is guaranteed to be safe, execute the plan
			typedef trajectory_processing::IterativeParabolicTimeParameterization iptp;
			iptp continuousTrajectory = iptp(100, .3);
			if(continuousTrajectory.computeTimeStamps(multi_trajectory_, 1.0))
			{
				multi_trajectory_.getRobotTrajectoryMsg(ready_to_execute.trajectory_);
				//multi_trajectory_.getRobotTrajectoryMsg(motion_plan_.trajectory_);
			}
			std::swap(start, goal);
		}
		
		void reverse()
		{
			multi_trajectory_.reverse();
		}
				
		std::vector<Node> getNodeList()
		{
			return node_list_;
		}
		
		DistanceMap getDistancesMap()
		{
			return distances_;
		}
		
		std::vector<int> getAStarPath()
		{
			return astar_path_;
		}
};

#endif /* GLOBAL_COLLISION_AVOIDANCE_H */
