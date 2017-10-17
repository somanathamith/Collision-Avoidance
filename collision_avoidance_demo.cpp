#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "common.h"
#include "workspace.h"
#include "collisionCheck.h"
#include "local_collision_avoidance.h"
#include "global_collision_avoidance.h"

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string GROUP_NAME="manipulator";

// ---------------------------------------------------------------------
// Class definition
// ---------------------------------------------------------------------
class CollisionAvoidance
{

public:
    CollisionAvoidance(moveit::planning_interface::MoveGroup& group, 
        CollisionCheck& collisionCheck, Workspace& workspace)
    : group_(group), workspace_(workspace), collisionCheck_(collisionCheck), 
      localAvoidance_(group, collisionCheck, workspace),
      globalAvoidance_(group, collisionCheck)
    {
        // Setup planning scene (add collision objects)
        workspace_.removeCollisionObjects();
        workspace_.addTable();
        workspace_.addObstacle();
        
        int num_points = 12;
        int degrees = 8;
        std::vector<std::vector<double> > init_points;
        init_points = {pick_configuration, place_configuration};
        
        globalAvoidance_.buildEnvironment(num_points, degrees, init_points);
    }

    ~CollisionAvoidance()
    {
    }

    void execute()
    {
        pick_and_place_demo();
    }
    
private:
    
    // go to home position
    void gohome()
    {
        ROS_INFO("gohome");
        group_.setNamedTarget("up");  // home, resting, up
        group_.move();
    }

    // ######################################################
    // --------------Planning--------------------------------
    // ######################################################

    bool plan_home_to_pick(moveit::planning_interface::MoveGroup::Plan& home2pick_plan)
    {
        ROS_INFO("Planning home to pick motion");

        gohome();
        group_.setJointValueTarget(pick_configuration);

        return group_.plan(home2pick_plan);
    }

    bool plan_pick_to_place(moveit::planning_interface::MoveGroup::Plan& pick_plan)
    {
        ROS_INFO("Planning pick to place motion");
        
        robot_state::RobotState pick_state(*group_.getCurrentState());
        pick_state.setVariablePositions(pick_configuration);
        group_.setStartState(pick_state);
        group_.setJointValueTarget(place_configuration);
        
        return group_.plan(pick_plan);
    }

    bool plan_place_to_pick(moveit::planning_interface::MoveGroup::Plan& place_plan)
    {
        ROS_INFO("Planning place to pick motion");
        
        robot_state::RobotState place_state(*group_.getCurrentState());
        place_state.setVariablePositions(place_configuration);
        group_.setStartState(place_state);
        group_.setJointValueTarget(pick_configuration);
        
        return group_.plan(place_plan);

    }

    bool pick_and_place()
    {
        moveit::planning_interface::MoveGroup::Plan pick_plan, place_plan, home2pick_plan;

        plan_home_to_pick(home2pick_plan);
        group_.execute(home2pick_plan);

        plan_pick_to_place(pick_plan);
        plan_place_to_pick(place_plan);

        while(ros::ok())
        {
            group_.execute(place_plan); 
            workspace_.displayTrajectory(place_plan.trajectory_);
            sleep(5.0);

            group_.execute(pick_plan); 
            workspace_.displayTrajectory(pick_plan.trajectory_);
            sleep(5.0);
        }
    }

    void pick_and_place_demo()
    {
        moveit::planning_interface::MoveGroup::Plan pick2place, place2pick, home2pick;
        moveit::planning_interface::MoveGroup::Plan org_pick2place, org_place2pick;

        plan_home_to_pick(home2pick);
        group_.execute(home2pick);

        while(ros::ok())
        {
            globalAvoidance_.globalPlan(pick2place);
            localAvoidance_.executePlan_modified(pick2place);
            workspace_.displayTrajectory(pick2place.trajectory_);
            //-----------------------------------------------    
            
            globalAvoidance_.globalPlan(place2pick);
            localAvoidance_.executePlan_modified(place2pick);
            workspace_.displayTrajectory(org_place2pick.trajectory_);
        
        }
    }

private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroup& group_;
    Workspace& workspace_;
    CollisionCheck& collisionCheck_;
    LocalCollisionAvoidance localAvoidance_;
	 GlobalCollisionAvoidance globalAvoidance_;
    // This type of assignment is possible only if C++11 complier is installed/used
    // Make sure that C++11 is enables in cmake file
    std::vector<double> pick_configuration =  {0.0, -2.35, -0.785, -1.57, 1.57, 0.0};
    std::vector<double> place_configuration = {-3.14, -2.35, -0.785, -1.57, 1.57, 0.0};
};

//###################################################################
// Main function
//###################################################################
int main(int argc, char **argv)
{
    ros::init (argc, argv, "collision_avoidance");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Planning scene monitor for updating world information
    moveit::planning_interface::PlanningSceneInterface psi;
    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf));

    moveit::planning_interface::MoveGroup group(GROUP_NAME);
    //group.setPlannerId("RRTstarkConfigDefault");
    //group.setPlannerId("PRMstarkConfigDefault");
    //group.setPlanningTime(5.0);

    if (psm->getPlanningScene())
    {
        psm->startSceneMonitor();
        psm->startWorldGeometryMonitor();
        psm->startStateMonitor();
        psm->publishDebugInformation(true);
        
        workspace = new Workspace(psi, psm, group);
        CollisionCheck collisionCheck(psm, group);
        CollisionAvoidance collision_avoidance(group, collisionCheck, *workspace);
        collision_avoidance.execute();
        ros::shutdown();
    }
    else
    {
        ROS_ERROR("Planning scene not configured");
    }

    return 0;
}


