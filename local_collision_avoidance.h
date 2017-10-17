#ifndef LOCAL_COLLISION_AVOIDANCE_H
#define LOCAL_COLLISION_AVOIDANCE_H

#include <iostream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "workspace.h"

static const int SPLICING_DEPTH = 1;

// ---------------------------------------------------------------------
// Class definition
// ---------------------------------------------------------------------

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JTAC;
class LocalCollisionAvoidance
{
public:
    LocalCollisionAvoidance(moveit::planning_interface::MoveGroup& group,
        CollisionCheck& collisionCheck, Workspace& workspace)
        : group_(group), collisionCheck_(collisionCheck), workspace_(workspace)
    {
        //! Initialize the action client and wait for action server to come up
        traj_client_ = new JTAC("follow_joint_trajectory");
        int trial = 5;
        while (!traj_client_->waitForServer(ros::Duration(1.0))) {
            ROS_INFO("Waiting for the joint_trajectory_action server");
            if (trial-- < 0) {
                fake_mode = true;
                break;
            }
        }
    }

    ~LocalCollisionAvoidance()
    {  
        //! Clean up the action client
        delete traj_client_;
    }
    
    void prepareToExecuteTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
    {
        // initialize time index for monitoring trajectory execution
        time_index_.clear();
        for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) 
        {
            time_index_.push_back(trajectory.joint_trajectory.header.stamp +
                trajectory.joint_trajectory.points[i].time_from_start);
        }
    }

    void executePlan(const moveit::planning_interface::MoveGroup::Plan& org_plan)
    {
        moveit::planning_interface::MoveGroup::Plan plan = org_plan;
        plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(0.0);   // 1.0?

        // display
        clearTrajectoryView();
        workspace_.displayOriginalTrajectory(org_plan.trajectory_);
        workspace_.displayTrajectory(plan.trajectory_);

        // execution
        if (fake_mode) {
            std::cout << "Execute trajectory in fake mode." << std::endl; 
            group_.execute(plan);
        }
        else {
            prepareToExecuteTrajectory(plan.trajectory_); // what does this do?
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = plan.trajectory_.joint_trajectory;
            traj_client_->sendGoal(goal);

            bool inCollision = false;
            while (!traj_client_->getState().isDone() && ros::ok())
            { 
                int path_segment = getCurrentExpectedTrajectoryIndex();
                
                if (!isRemainingPathValid(plan, path_segment)) {
                    std::cout << "Update trajectory." << std::endl;
                    inCollision = getCollisionFreeTrajectory(plan.trajectory_, path_segment);
                    workspace_.displayTrajectory(plan.trajectory_);

                    prepareToExecuteTrajectory(plan.trajectory_);
                    goal.trajectory = plan.trajectory_.joint_trajectory;
                    traj_client_->sendGoal(goal);
                }
                usleep(50000);
            }
        }
    }

    void executePlan_modified(moveit::planning_interface::MoveGroup::Plan& org_plan)
    {
        multiplyWayPointsForTrajectory(org_plan.trajectory_,2);

        moveit::planning_interface::MoveGroup::Plan plan = org_plan;
        plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(0.0);   // 1.0?

        clearTrajectoryView();
        workspace_.displayOriginalTrajectory(org_plan.trajectory_);
        workspace_.displayTrajectory(plan.trajectory_);

        // execution
        if (fake_mode) {
            std::cout << "Execute trajectory in fake mode." << std::endl; 
            group_.execute(plan);
        }
        else {
            prepareToExecuteTrajectory(plan.trajectory_); // what does this do?

            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = plan.trajectory_.joint_trajectory;
            traj_client_->sendGoal(goal);

            bool found;
            while (!traj_client_->getState().isDone() && ros::ok())
            {
                int path_segment = getCurrentExpectedTrajectoryIndex();
                                
                if (!isPathClear(plan, path_segment)) {
                    std::cout << "Update trajectory. " << path_segment << std::endl;
                    found = getCollisionFreeTrajectory_modified(plan.trajectory_, path_segment);

                    while(!found)
                    {
                        sleep(5);
                        found = getCollisionFreeTrajectory_modified(plan.trajectory_, path_segment);
                    }

                    workspace_.displayTrajectory(plan.trajectory_);

                    prepareToExecuteTrajectory(plan.trajectory_);
                    goal.trajectory = plan.trajectory_.joint_trajectory;
                    
                    traj_client_->sendGoal(goal);
                }
            }
        }
    }
    
private:

    // ######################################################
    // -----------Collision avoidance------------------------
    // ######################################################

    bool getCollisionFreeConfigurationInBetween(std::vector<double>& a, std::vector<double>& b, 
        std::vector<double>& new_point_bf, std::vector<double>& new_point, std::vector<double>& new_point_af)
    {
        double max_dist = distance(a,b)*0.5;
        double temp;
        
        std::vector<double> mid_point = {0,0,0,0,0,0};
        new_point_bf = {0,0,0,0,0,0};
        new_point = {0,0,0,0,0,0};
        new_point_af = {0,0,0,0,0,0};
                
        for (int i=0; i<a.size(); i++)
        {
            mid_point[i] = (b[i]+a[i])*0.5;    
        }

        do
        {
            collisionCheck_.getCollisionFreeConfiguration(mid_point, new_point, max_dist);
            temp = distance(a, new_point) / distance(b, new_point);
        }
        while (temp<0.8 || temp>1.2);

        for (int i=0; i<a.size(); i++)
        {
            new_point_bf[i] = (new_point[i] + a[i]) * 0.75 - 0.5 * b[i]; //new_point[i] + a[i] - b[i];
        }
        
        for (int i=0; i<a.size(); i++)
        {
            new_point_af[i] = (new_point[i] + b[i]) * 0.75 - 0.5 * a[i]; //new_point[i] + b[i] - a[i];
        }
        
        return true;
    }
    
     bool getCollisionFreeConfigurationInBetween(std::vector<double>& a, std::vector<double>& b, 
          std::vector<double>& new_point)
    {
        double max_dist = distance(a,b)*0.5;
        double temp;
        
        std::vector<double> mid_point = {0,0,0,0,0,0};
        new_point = {0,0,0,0,0,0};
                       
        for (int i=0; i<a.size(); i++)
        {
            mid_point[i] = (b[i]+a[i])*0.5;    
        }

        do
        {
           collisionCheck_.getCollisionFreeConfiguration(mid_point,new_point,max_dist);
           temp = distance(a,new_point)/distance(b,new_point);
        }
        while (temp<0.8 || temp>1.2);

        return true;
    }
    
    //-----------------------------------------------------------------------
    // Core function which computes collision free trajectory
    //-----------------------------------------------------------------------
    bool getCollisionFreeTrajectory(moveit_msgs::RobotTrajectory& trajectory, int path_segment = 0)
    {
        std::cout << "getCollisionFreeTrajectory" << std::endl;
        int np = trajectory.joint_trajectory.points.size();
        std::vector<trajectory_msgs::JointTrajectoryPoint> points;
        for (int i = std::max(path_segment - 1, 0); i < np; i++)
        {
            points.push_back(trajectory.joint_trajectory.points[i]);
        }
        trajectory.joint_trajectory.points = points;
        std::cout << "trajectory = " << trajectory << std::endl;
        
        int start_id = -1;
        int end_id = -1;
        bool inCollision;
        for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
        {
            // check collision for each waypoint on the trajectory
            inCollision = collisionCheck_.checkCollision(trajectory.joint_trajectory.points[i].positions);
            if (inCollision && start_id < 0) 
            {
                start_id = i;
            }
            if (inCollision) 
            {
                end_id = i;
            }
        }
        
        // No collision found
        if (start_id==-1 && end_id==-1)
        {
            return false;
        }
    
        // Obstacle blocking start or goal position
        if (start_id==0 || end_id==trajectory.joint_trajectory.points.size()-1)
        {
            ROS_WARN("Obstacle blocking start or goal positions, can't find collision free trajectory");
            return true;
        }

        // Else modify trajectory
        moveit_msgs::RobotTrajectory new_traj;
        std::vector< std::vector<double> > new_config_list, config_list_copy;
        std::vector<double> new_config, new_config_af, new_config_bf;
        int splicing_depth = 0;
           
        new_config_list.push_back(trajectory.joint_trajectory.points[start_id-1].positions);
        new_config_list.push_back(trajectory.joint_trajectory.points[end_id+1].positions);

        while (splicing_depth<SPLICING_DEPTH)    
        {
            config_list_copy = new_config_list;
            new_config_list.clear();
            int i = 0;
            for(i = 0; i < config_list_copy.size()-1; i++)
            {
                getCollisionFreeConfigurationInBetween(config_list_copy[i], config_list_copy[i+1], new_config_bf, new_config, new_config_af);
                   
                //new_config_list.push_back(config_list_copy[i]);
                new_config_list.push_back(new_config_bf);
                new_config_list.push_back(new_config);
                new_config_list.push_back(new_config_af);
            }

            //new_config_list.push_back(config_list_copy[i]);
            splicing_depth++; 
        }   
        
        // Needed because a weird shape obstacle might block only few points between start_id and end_id
        for (int i=0; i<start_id-1; i++)
        {
           new_traj.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]); 
        }
        for (int i=0; i < new_config_list.size(); i++)
        {
           trajectory.joint_trajectory.points[i].positions = new_config_list[i]; 
           new_traj.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);
        }    
        for (int i=end_id+2; i<trajectory.joint_trajectory.points.size(); i++)
        {
           new_traj.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]); 
        }          
        
        // Update current trajectory to new collision free trajectory
        trajectory.joint_trajectory.points.clear();
        trajectory.joint_trajectory.points = new_traj.joint_trajectory.points;

        // time parametrization
        time_parametrize(trajectory);

        //trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        trajectory.joint_trajectory.header.stamp = ros::Time::now();
    }
    //-----------------------------------------------------------------------

    void getMidConfiguration(std::vector<double>& a, std::vector<double>& b, std::vector<double>& mid_point)
    {
        mid_point = {0, 0, 0, 0, 0, 0};
        for (int i=0; i<a.size(); i++)
        {
            mid_point[i] = (a[i] + b[i])*0.5;    
        }
    }
    
    void multiplyWayPointsForTrajectory(moveit_msgs::RobotTrajectory& trajectory, int multiplicativeFactor)
    {
        moveit_msgs::RobotTrajectory new_traj;
        std::vector<double> mid_point; 
        int factor = 1;

        while (factor<multiplicativeFactor)    
        {
            new_traj = trajectory;
            trajectory.joint_trajectory.points.clear();

            int i=0;
            for (i=0; i < new_traj.joint_trajectory.points.size()-1; i++)
            {
                getMidConfiguration(new_traj.joint_trajectory.points[i].positions, new_traj.joint_trajectory.points[i+1].positions, mid_point);
                trajectory.joint_trajectory.points.push_back(new_traj.joint_trajectory.points[i]); 
                new_traj.joint_trajectory.points[i].positions = mid_point;
                trajectory.joint_trajectory.points.push_back(new_traj.joint_trajectory.points[i]); 
            }

            trajectory.joint_trajectory.points.push_back(new_traj.joint_trajectory.points[i]); 
            factor++; 
        } 

        // time parametrization
        time_parametrize(trajectory);
        trajectory.joint_trajectory.header.stamp = ros::Time::now();
    }

    int getCurrentExpectedTrajectoryIndex() const
    {
        if (time_index_.empty()) {
            return -1;
        }
        ros::Time tNow = ros::Time::now();
        std::vector<ros::Time>::const_iterator it = 
            std::lower_bound(time_index_.begin(), time_index_.end(), tNow);
        
        int pos = it - time_index_.begin() - 1;
         
        return pos;
    }

    bool isRemainingPathValid(const moveit::planning_interface::MoveGroup::Plan& plan, int path_segment)
    {
        if (path_segment >= 0)
        {
            const moveit_msgs::RobotTrajectory &t = plan.trajectory_;
            std::size_t wpc = t.joint_trajectory.points.size();
            for (std::size_t i = std::max(path_segment - 1, 0) ; i < wpc; ++i)
            {
                if (collisionCheck_.checkCollision(t.joint_trajectory.points[i].positions)) {
                    return false;
                }
            }
        }
        return true;
    }
    
    bool isPathClear(const moveit::planning_interface::MoveGroup::Plan& plan, int current_id)
    {
        if (current_id >= 0)
        {
            const moveit_msgs::RobotTrajectory &t = plan.trajectory_;
            int look_ahead = std::min(current_id + 4, int(plan.trajectory_.joint_trajectory.points.size()));
            for (std::size_t i = current_id; i < look_ahead; ++i)
            {
                if (collisionCheck_.checkCollision(t.joint_trajectory.points[i].positions)) 
                {
                    return false;
                }
            }
        }
        return true;
    }
  
    //-----------------------------------------------------------------------
    // MODIIED function that computes collision free trajectory
    //-----------------------------------------------------------------------
    bool getCollisionFreeTrajectory_modified(moveit_msgs::RobotTrajectory& trajectory, int current_id)
    {
        std::cout<<"current_id in modified trajectory function: "<<current_id<<std::endl;
        
        // Check start and end ids of trajectory blocked by obstacle
        int start_id = -1;
        int end_id = -1;
        bool inCollision;
        for (int i = current_id; i < trajectory.joint_trajectory.points.size(); ++i)
        {
            // check collision for each waypoint on the trajectory
            inCollision = collisionCheck_.checkCollision(trajectory.joint_trajectory.points[i].positions);
            if (inCollision && start_id < 0) {
                start_id = i;
            }
            if (inCollision) {
                end_id = i;
            }
        }
        
        // No collision found
        if (start_id==-1 && end_id==-1)
        {
            return true;
        }
    
        // Obstacle blocking start or goal position
        if (start_id==0 || end_id==trajectory.joint_trajectory.points.size()-1)
        {
            ROS_WARN("Obstacle blocking goal positions, can't find collision free trajectory");

            std::vector<trajectory_msgs::JointTrajectoryPoint> rpoints;
            for (int i = current_id; i < trajectory.joint_trajectory.points.size(); i++)
            {
                rpoints.push_back(trajectory.joint_trajectory.points[i]);
            }

            trajectory.joint_trajectory.points = rpoints;
            return false;
        }

        // Else modify trajectory
        moveit_msgs::RobotTrajectory new_traj1,new_traj2;
        std::vector< std::vector<double> > new_config_list;
        std::vector<double> start_config, end_config, new_config, new_config_af, new_config_bf;
        
        start_config = trajectory.joint_trajectory.points[start_id-1].positions;
        end_config = trajectory.joint_trajectory.points[end_id+1].positions; 
        getCollisionFreeConfigurationInBetween(start_config, end_config, new_config_bf, new_config, new_config_af);
          
        new_config_list.push_back(new_config_bf);
        new_config_list.push_back(new_config);
        new_config_list.push_back(new_config_af);
        
        std::cout<<"current : "<<current_id<<" start : "<<start_id<<" end : "<<end_id<<std::endl;
        
        // Needed because a weird shape obstacle might block only few points between start_id and end_id
        double totalT = distance(trajectory.joint_trajectory.points[current_id].positions,
            trajectory.joint_trajectory.points[start_id-1].positions)/ distance(trajectory.joint_trajectory.points[current_id].velocities,
            {0,0,0,0,0,0}) ;

        double d = distance(trajectory.joint_trajectory.points[current_id].positions, trajectory.joint_trajectory.points[start_id-1].positions);
        double max_acc_dec = 1.0;
        double current_velocity = distance(trajectory.joint_trajectory.points[current_id].velocities, {0,0,0,0,0,0});
        double needed_time_for_deceleration = current_velocity / max_acc_dec;
        int numSteps = start_id - current_id - 1;    
        //double dt = totalT/numSteps;

        std::cout << "numSteps = " << numSteps << std::endl;

        //new_traj1.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[current_id]);
        double dt = 0;
        for (int i=0; i<numSteps; i++)
        {
           //std::cout<<trajectory.joint_trajectory.points[current_id + i + 1]<<std::endl; 
           new_traj1.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[current_id + i+1]);
           dt += needed_time_for_deceleration*(2*i+1)/(numSteps*numSteps);
           new_traj1.joint_trajectory.points[i].time_from_start = ros::Duration(dt);
           new_traj1.joint_trajectory.points[i].accelerations = {0,0,0,0,0,0};  
           new_traj1.joint_trajectory.points[i].velocities = {0,0,0,0,0,0};
        }
        
        new_traj2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[start_id-1]);
        for (int i=0; i < new_config_list.size(); i++)
        {
           trajectory.joint_trajectory.points[i].positions = new_config_list[i]; 
           new_traj2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);
        }
        for (int i=end_id+1; i<trajectory.joint_trajectory.points.size(); i++)
        {
           new_traj2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]); 
        }
                
        // save points in part1 of trajectory
        std::vector<trajectory_msgs::JointTrajectoryPoint> points;
        int np1 = new_traj1.joint_trajectory.points.size();
        for (int i = 0; i < np1; i++)
        {
            points.push_back(new_traj1.joint_trajectory.points[i]);
        }
        ros::Duration time_traj2_start = ros::Duration(0);
        if (np1 > 0) {
            time_traj2_start = new_traj1.joint_trajectory.points[np1-1].time_from_start;
        }

        // Update current trajectory to new collision free trajectory (add two trajectories)
        trajectory.joint_trajectory.points.clear();
        trajectory.joint_trajectory.points = new_traj2.joint_trajectory.points;

        // time parametrization
        time_parametrize(trajectory);
        
        // combine part1 and part2
        for (int i = 1; i < trajectory.joint_trajectory.points.size(); i++)
        {
            trajectory.joint_trajectory.points[i].time_from_start += time_traj2_start;
            points.push_back(trajectory.joint_trajectory.points[i]);
        }
        trajectory.joint_trajectory.points = points;
        trajectory.joint_trajectory.header.stamp = ros::Time::now();

        return true;
    }

    //-----------------------------------------------------------------------
    void clearTrajectoryView() 
    {
        moveit::planning_interface::MoveGroup::Plan empty_plan;
        workspace_.displayTrajectory(empty_plan.trajectory_);
        workspace_.displayOriginalTrajectory(empty_plan.trajectory_);
    }    

    void time_parametrize(moveit_msgs::RobotTrajectory& trajectory)
    {
        // time parametrization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.01);

        // convert moveit trajectory message to robot trajectory object
        robot_trajectory::RobotTrajectory rt(group_.getCurrentState()->getRobotModel(), group_.getName());
        rt.setRobotTrajectoryMsg(*group_.getCurrentState(), trajectory);

        // get time parametization of current robot trajectory 
        iptp.computeTimeStamps(rt);

        // convert robot trajectory to moveit trajectory message
        rt.getRobotTrajectoryMsg(trajectory);
    }
    
private:
    moveit::planning_interface::MoveGroup& group_;
    CollisionCheck& collisionCheck_;
    Workspace& workspace_;
        
    // Action client for the joint trajectory action 
    // used to trigger the arm movement action
    JTAC* traj_client_;
    bool fake_mode = false;
    std::vector<ros::Time> time_index_;
};

#endif /* LOCAL_COLLISION_AVOIDANCE_H */
