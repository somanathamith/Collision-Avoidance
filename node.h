#ifndef NODE_H
#define NODE_H

#include <vector>

class Node
{
	protected:
		int nodeId;
		double cost;
		std::vector<double> joint_pos;
		std::vector<int> nnList;
		
		// list of trajetories from (parent) node id to node ids of all neighbors
		std::vector<std::pair<int, robot_trajectory::RobotTrajectory> > traj_list;
		
 	public:
		Node() { }

		Node(std::vector<double> jointList) : joint_pos(jointList) { }
		
		Node(double j1, double j2, double j3,double j4, double j5, double j6) 
		{ 
			joint_pos = {j1, j2, j3, j4, j5, j6};			
		}

	    void setNeighbor(int newNeighborId)
		{
			nnList.push_back(newNeighborId);
		}
		
		bool checkNeighbor(int id) 
		{
			std::vector<int>::iterator findInt = find(nnList.begin(), nnList.end(), id);
			if(findInt != nnList.end())
				return true;
			return false;
		}
		
		std::vector<int> getNN() 
		{
			return nnList;
		}

		std::vector<double> getJointPos() 
		{
			return joint_pos;
		}

		void setNodeID(int newNodeId) 
		{
			nodeId = newNodeId;			
		}

		int getNodeID() 
		{
			return nodeId;
		}
		
		//Can change based on how the distance between two joints is classified
		//At this point, take the square of the difference in each joint angle and sum them
		double distance(std::vector<double> otherNodePos) 
		{
			double dist = 0;
			for(int j = 0; j < joint_pos.size(); j++)
			{
				dist += std::pow((joint_pos[j] - otherNodePos[j]), 2);
			}
			return dist; //THERE IS AN ERROR HERE IN DISTANCE CALCULATION, missing square root
		}
		
		//Function to populate the nodeID-trajectory look up list
		void populate_traj_list(int newPathId, robot_trajectory::RobotTrajectory newTraj) 
		{
			traj_list.push_back(std::make_pair(newPathId, newTraj));
		}

		// return the trajectory from parent node to node with given lookupId
		robot_trajectory::RobotTrajectory get_trajectory_to_node(int lookupId) 
		{
			//if id does not exist, it will return an empty pose			
			for(int look = 0; look < traj_list.size(); look++)
			 {
				if(traj_list[look].first == lookupId)
					return traj_list[look].second;
			}
		}
		
		bool equals(std::vector<double> compare_node)
		{
			for(int c = 0; c < compare_node.size(); c++)
			{
				if(joint_pos[c] != compare_node[c])
					return false;
			}
			return true;
		}
		
};
#endif /* NODE_H */
