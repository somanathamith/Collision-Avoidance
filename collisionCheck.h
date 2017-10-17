#ifndef COLLISIONCHECK_H
#define COLLISIONCHECK_H

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>

class CollisionCheck
{
public:
    CollisionCheck(planning_scene_monitor::PlanningSceneMonitorPtr &psm,
        moveit::planning_interface::MoveGroup& group)
        : psm_(psm), group_(group)
    {
        planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        while(planning_scene_diff_publisher.getNumSubscribers() < 1)
            {
                ros::WallDuration sleep_t(0.5);
                sleep_t.sleep();    
            }
    }

    // ######################################################
    // -----------Collision avoidance------------------------
    // ######################################################

    // collision check for current state
    bool checkCollision()
    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        robot_state::RobotState& current_state = ps->getCurrentStateNonConst();
        const robot_model::JointModelGroup* joint_model_group = 
            current_state.getJointModelGroup(group_.getName());
        std::vector<double> joint_values;
        current_state.copyJointGroupPositions(joint_model_group, joint_values);
        return checkCollision(joint_values);
    }
    
    // collision check for joint configuration
    bool checkCollision(std::vector<double> joint_values)
    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();
        acm.setEntry("table", "base_link", true);
        
        bool collision = false;
        robot_state::RobotState& current_state = ps->getCurrentStateNonConst();
        current_state.update();
        const robot_model::JointModelGroup* joint_model_group = 
            current_state.getJointModelGroup(group_.getName());
        current_state.setJointGroupPositions(joint_model_group, joint_values);

        collision_detection::CollisionRequest creq;
        collision_detection::CollisionResult cres;
        creq.contacts = true;
        creq.max_contacts = 1000;
        cres.clear();
        ps->checkCollision(creq, cres, current_state, acm);
        //ps->checkCollisionUnpadded(creq, cres, current_state, acm);

                bool debug = false; // change to true to print collision check in verbose mode
        collision = cres.collision;
        if (debug) {
            if (collision) { 
                ps->printKnownObjects(std::cout);
                std::cout << "state = " << joint_values;
                ROS_INFO_STREAM("Current state is "
                    << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
                ROS_INFO_STREAM("Current state is "
                    << (cres.collision ? "in" : "not in")<< " collision");
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = cres.contacts.begin(); it != cres.contacts.end(); ++it)
                {
                    ROS_INFO("Contact between: %s and %s", 
                        it->first.first.c_str(), it->first.second.c_str());
                }
            }
        } 
        return collision;
    }
    
    // collision check with an attached object to the robot
    bool checkCollision_with_attached_object()
    {
        // Get current robot state for collision check
        planning_scene::PlanningScenePtr ps = psm_->getPlanningScene();
        robot_state::RobotState& current_state = ps->getCurrentStateNonConst();
        const robot_model::JointModelGroup* joint_model_group = 
            current_state.getJointModelGroup(group_.getName());
        std::vector<double> joint_values;
        current_state.copyJointGroupPositions(joint_model_group, joint_values);

        return checkCollision_with_attached_object(joint_values);

    }    
    
    // collision check with an attached object to the robot
    bool checkCollision_with_attached_object(std::vector<double> joint_values)
    {
        // Lock planning scene for collision check
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);

        moveit_msgs::PlanningScene planning_scene;

        planning_scene.is_diff = true;
        
        // Define attached object
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "gripper_link";
        attached_object.object.header.frame_id = "gripper_link";
        attached_object.object.id = "sphere";
        
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE; 
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = 0.1;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);
        attached_object.object.operation = attached_object.object.ADD;

        // Attach object to robot and the environment
        planning_scene.robot_state.attached_collision_objects.push_back(attached_object); // to the robot
        //planning_scene.world.collision_objects.push_back(attached_object.object); // to the environment
        planning_scene_diff_publisher.publish(planning_scene);

        // Allow collisions between attahed object and nearby parts of robot
        collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();
        acm.setEntry("table", "base_link", true);
        acm.setEntry("sphere", "finger_link", true);
        acm.setEntry("sphere", "gripper_base_link", true);
        acm.setEntry("sphere", "gripper_link", true);
        acm.setEntry("sphere", "base_link", true);
        
        // Check collision
        bool collision = false;
        robot_state::RobotState& current_state = ps->getCurrentStateNonConst();
        current_state.update();
        const robot_model::JointModelGroup* joint_model_group = 
            current_state.getJointModelGroup(group_.getName());
        current_state.setJointGroupPositions(joint_model_group, joint_values);

        //std::cout << "checkCollision2.2" << std::endl;
        collision_detection::CollisionRequest creq;
        collision_detection::CollisionResult cres;
        creq.contacts = true;
        creq.max_contacts = 1000;
        cres.clear();
        ps->checkCollision(creq, cres, current_state, acm);
        //ps->checkCollisionUnpadded(creq, cres, current_state, acm);

                bool debug = false; // change to true to print collision check in verbose mode
        collision = cres.collision;
        if (debug) {
            if (collision) { 
                ps->printKnownObjects(std::cout);
                std::cout << "state = " << joint_values;
                ROS_INFO_STREAM("Current state is "
                    << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
                ROS_INFO_STREAM("Current state is "
                    << (cres.collision ? "in" : "not in")<< " collision");
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = cres.contacts.begin(); it != cres.contacts.end(); ++it)
                {
                    ROS_INFO("Contact between: %s and %s", 
                        it->first.first.c_str(), it->first.second.c_str());
                }
            }
        } 

        // get collision free configuration
        std::vector<double> new_config;
        double max_dist = 1.0;
        
        if(collision)
        {
           ROS_WARN("In collision");
           getCollisionFreeConfiguration(new_config, max_dist); 
           std::cout<<"new config: "<<new_config;
           group_.setJointValueTarget(new_config);
           group_.move();
        }
        

        // Detach the object from the robot and the environment
        attached_object.object.operation = attached_object.object.REMOVE;

        planning_scene.robot_state.attached_collision_objects.clear();
        //planning_scene.world.collision_objects.clear(); 
        //planning_scene.robot_state.attached_collision_objects.push_back(attached_object);

       planning_scene_diff_publisher.publish(planning_scene);

        return collision;
    }
    
   
    // collision check for current robot state
    bool checkCollision(robot_state::RobotState current_state)
    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();
        acm.setEntry("table", "base_link", true);

		  //const robot_model::JointModelGroup* joint_model_group = 
        //   current_state.getJointModelGroup(group_.getName());

        bool collision = false;
        collision_detection::CollisionRequest creq;
        collision_detection::CollisionResult cres;
        creq.contacts = true;
        creq.max_contacts = 1000;
        cres.clear();
        ps->checkCollision(creq, cres, current_state, acm);
        //ps->checkCollisionUnpadded(creq, cres, current_state, acm);

        bool debug = true;
        collision = cres.collision;
        
        return collision;
    }

/*
    poincloudoctomap_updator
    {
        update planning scene (octomap)
    }

    somefunction()
    {
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);

        moveit_msgs::CollisionObject addCollisionObjectMsg;
        publish removeCollisionObjectMsg or call ps.usePlanningSceneMsg(removeCollisionObjectMsg) directly to update planningScene 

        if (checkCollision(ps))
        {
            getCollisionFreeConfiguration(ps)            
        }

        moveit_msgs::CollisionObject removeCollisionObjectMsg;
        publish removeCollisionObjectMsg or call ps.usePlanningSceneMsg(removeCollisionObjectMsg) directly to update planningScene 
    }

    bool checkConfiguration(planning_scene_monitor::LockedPlanningSceneRW& ps)
    {
        get some_configuration;

        ps.checkCollision(some_configuration);
    }

    bool getCollisionFreeConfiguration(planning_scene_monitor::LockedPlanningSceneRW& ps)
    {
        get some_configuration;

        ps.checkCollision(some_configuration);

        for (...)
        {
            sampling new point
            checkCollision(ps);
        }
    }
*/
    
    bool getCollisionFreeConfiguration(std::vector<double>& new_c)
    {
        // get the specified start state
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        //planning_scene::PlanningScenePtr ps = psm_->getPlanningScene();
        robot_state::RobotState current_state = ps->getCurrentState();

        std::vector<double> c;
        current_state.copyJointGroupPositions(group_.getName(), c);
        return getCollisionFreeConfiguration(c, new_c);
    }
    
    bool getCollisionFreeConfiguration(std::vector<double>& new_c, double max_dist)
    {
        // get the specified start state
        //planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        planning_scene::PlanningScenePtr ps = psm_->getPlanningScene();
        robot_state::RobotState current_state = ps->getCurrentState();

        std::vector<double> c;
        current_state.copyJointGroupPositions(group_.getName(), c);
        return getCollisionFreeConfiguration(c, new_c, max_dist);
    }

    bool getCollisionFreeConfiguration(const std::vector<double>& c, std::vector<double>& new_c)
    {
        bool debug = false;
        bool diff = false;
        double max_dt_offset_ = 0.5;
        double jiggle_fraction_ = JIGGLE_FRACTION;
        int sampling_attempts_ = 100;

        // get the specified start state
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        robot_model::RobotModelConstPtr rm = ps->getRobotModel();
        robot_state::RobotState org_state = ps->getCurrentState();
        const moveit::core::JointModelGroup* jmg = rm->getJointModelGroup("manipulator");
        org_state.setJointGroupPositions(jmg, c);

        // Check collision
        collision_detection::CollisionRequest creq;
        creq.group_name = group_.getName();
        collision_detection::CollisionResult cres;
        //ps->checkCollision(creq, cres, org_state);
        ps->checkCollisionUnpadded(creq, cres, org_state);
        
        if (cres.collision)
        {
            // Rerun in verbose mode
            collision_detection::CollisionRequest vcreq = creq;
            collision_detection::CollisionResult vcres;
            vcreq.verbose = false; // change to true to check collision in verbose mode
            //ps->checkCollision(vcreq, vcres, org_state);
            ps->checkCollisionUnpadded(vcreq, vcres, org_state);

            if (debug) {
                if (creq.group_name.empty()) {
                    ROS_INFO("State appears to be in collision");
                }
                else {
                    ROS_INFO_STREAM("State appears to be in collision with respect to group " << creq.group_name);
                }
            }

            robot_state::RobotStatePtr prefix_state(new robot_state::RobotState(org_state));
            random_numbers::RandomNumberGenerator &rng = prefix_state->getRandomNumberGenerator();
      
            const std::vector<const robot_model::JointModel*> &jmodels =
                ps->getRobotModel()->hasJointModelGroup(group_.getName()) ?
                ps->getRobotModel()->getJointModelGroup(group_.getName())->getJointModels() :
                ps->getRobotModel()->getJointModels();
      
            // Sampling collision free state
            bool found = false;
            for (int c = 0; !found && c < sampling_attempts_; ++c)
            {
                for (std::size_t i = 0 ; !found && i < jmodels.size() ; ++i)
                {
                    std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
                    const double *original_values = prefix_state->getJointPositions(jmodels[i]);
                    jmodels[i]->getVariableRandomPositionsNearBy(
                        rng, &sampled_variable_values[0], original_values, 
                        jmodels[i]->getMaximumExtent() * jiggle_fraction_);
                    org_state.setJointPositions(jmodels[i], sampled_variable_values);
                    collision_detection::CollisionResult cres;
                    ps->checkCollisionUnpadded(creq, cres, org_state);
                    if (!cres.collision)
                    {
                        found = true;
                        org_state.copyJointGroupPositions(group_.getName(), new_c);
                        if (debug) {
                            ROS_INFO("Found a valid state near the start state at distance %lf after %d attempts", 
                                prefix_state->distance(org_state), c);
                        }
                    }
                }
            }

            if (found)
            {
                diff = true;
            }
            else
            {
                ROS_WARN("Unable to find a valid state nearby the state"
                    " (using jiggle fraction of %lf and %u sampling attempts)."
                    " Passing the original planning request to the planner.",
                    jiggle_fraction_, sampling_attempts_);
            }
        }
        else
        {
            if (debug) {
                if (creq.group_name.empty()) {
                    ROS_DEBUG("State is valid");
                }
                else {
                    ROS_DEBUG_STREAM("State is valid with respect to group " << creq.group_name);
                }
            }
        }
        return diff;
    }

    bool getCollisionFreeConfiguration(const std::vector<double>& c, std::vector<double>& new_c, double max_dist)
    {
        bool debug = false;
        bool diff = false;
        double max_dt_offset_ = 0.5;
        double jiggle_fraction_ = JIGGLE_FRACTION;
        int sampling_attempts_ = 100;

        // get the specified start state
        planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        robot_model::RobotModelConstPtr rm = ps->getRobotModel();
        robot_state::RobotState org_state = ps->getCurrentState();
        const moveit::core::JointModelGroup* jmg = rm->getJointModelGroup("manipulator");
        org_state.setJointGroupPositions(jmg, c);

        // Check collision
        collision_detection::CollisionRequest creq;
        creq.group_name = group_.getName();
        collision_detection::CollisionResult cres;
        //ps->checkCollision(creq, cres, org_state);
        ps->checkCollisionUnpadded(creq, cres, org_state);
        
        if (cres.collision)
        {
            // Rerun in verbose mode
            collision_detection::CollisionRequest vcreq = creq;
            collision_detection::CollisionResult vcres;
            vcreq.verbose = true;
            //ps->checkCollision(vcreq, vcres, org_state);
            ps->checkCollisionUnpadded(vcreq, vcres, org_state);

            if (debug) {
                if (creq.group_name.empty()) {
                    ROS_INFO("State appears to be in collision");
                }
                else {
                    ROS_INFO_STREAM("State appears to be in collision with respect to group " << creq.group_name);
                }
            }

            robot_state::RobotStatePtr prefix_state(new robot_state::RobotState(org_state));
            random_numbers::RandomNumberGenerator &rng = prefix_state->getRandomNumberGenerator();
      
            const std::vector<const robot_model::JointModel*> &jmodels =
                ps->getRobotModel()->hasJointModelGroup(group_.getName()) ?
                ps->getRobotModel()->getJointModelGroup(group_.getName())->getJointModels() :
                ps->getRobotModel()->getJointModels();
      
            // Sampling collision free state
            bool found = false;
            for (int c = 0 ; !found && c < sampling_attempts_ ; ++c)
            {
                for (std::size_t i = 0 ; !found && i < jmodels.size() ; ++i)
                {
                    std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
                    const double *original_values = prefix_state->getJointPositions(jmodels[i]);
                    
                    // ---------------------------------------------
                    //std::cout<<"Sampling new point at max distance: "<<max_dist<<std::endl;
                    jmodels[i]->getVariableRandomPositionsNearBy(rng, &sampled_variable_values[0], original_values, max_dist); 
                    // ---------------------------------------------
                                        
                    org_state.setJointPositions(jmodels[i], sampled_variable_values);
                    collision_detection::CollisionResult cres;
                    //ps->checkCollision(creq, cres, org_state);
                    ps->checkCollisionUnpadded(creq, cres, org_state);
                    if (!cres.collision)
                    {
                        found = true;
                        org_state.copyJointGroupPositions(group_.getName(), new_c);
                        if (debug) {
                            ROS_INFO("Found a valid state near the start state at distance %lf after %d attempts", 
                                prefix_state->distance(org_state), c);
                        }
                    }
                }
            }

            if (found)
            {
                diff = true;
            }
            else
            {
                ROS_WARN("Unable to find a valid state nearby the state"
                    " (using jiggle fraction of %lf and %u sampling attempts)."
                    " Passing the original planning request to the planner.",
                    jiggle_fraction_, sampling_attempts_);
            }
        }
        else
        {
            if (debug) {
                if (creq.group_name.empty()) {
                    ROS_DEBUG("State is valid");
                }
                else {
                    ROS_DEBUG_STREAM("State is valid with respect to group " << creq.group_name);
                }
            }
        }
        return diff;
    }

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
           getCollisionFreeConfiguration(mid_point,new_point,max_dist);
           temp = distance(a,new_point)/distance(b,new_point);
        }
        while (temp<0.8 || temp>1.2);

        for (int i=0; i<a.size(); i++)
        {
           new_point_bf[i] = (new_point[i] + a[i])*0.75 - 0.5*b[i]; //new_point[i] + a[i] - b[i];
        }
        //std::cout<<"Point before: "<<std::endl<<new_point_bf<<std::endl; 
        //new_point_bf = (new_point + a - b) - (new_point-a)*(b-a);
        //std::cout<<new_point_bf<<std::endl; 

        for (int i=0; i<a.size(); i++)
        {
            new_point_af[i] = (new_point[i] + b[i])*0.75 - 0.5*a[i]; //new_point[i] + b[i] - a[i];
        }

        //std::cout<<"Point after: "<<std::endl<<new_point_af<<std::endl; 
        //new_point_af = (new_point + b - a) - (new_point-a)*(b-a);
        //std::cout<<new_point_af<<std::endl;

        return true;
    }
    
            
private:
    ros::NodeHandle nh_;
    planning_scene_monitor::PlanningSceneMonitorPtr &psm_;
    moveit::planning_interface::MoveGroup& group_;
    ros::Publisher planning_scene_diff_publisher;
};

#endif /* COLLISIONCHECK_H */

