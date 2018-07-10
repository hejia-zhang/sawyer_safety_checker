//
// Created by hejia on 6/28/18.
//

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/PlanningScene.h>

#include "sawyer_safety_checker/CollisionChecker.h"
#include "sawyer_safety_checker/CollisionPredictor.h"

// Base planning scene.
planning_scene::PlanningScenePtr pPlanningSceneParent;
// Base planning scene + additional objects actually used to check the collision.
planning_scene::PlanningScenePtr pPlanningScene;
// Allowed self collision
collision_detection::AllowedCollisionMatrix acm;

void disableSelfCol() {
    robot_state::RobotState copied_state = pPlanningScene->getCurrentState();

    collision_detection::CollisionRequest colReq;
    colReq.contacts = true;
    colReq.max_contacts = 1000;
    collision_detection::CollisionResult colRes;
    pPlanningScene->checkCollision(colReq, colRes, copied_state);

    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = colRes.contacts.begin();
        it != colRes.contacts.end();
        ++it)
    {
        acm.setEntry(it->first.first, it->first.second, true);
    }
}

void onJointStatesMsg(const sensor_msgs::JointState& msg) {
    // Update current state:
    robot_state::RobotState& currentState = pPlanningSceneParent->getCurrentStateNonConst();
    robot_state::jointStateToRobotState(msg, currentState);
}

bool predictCollision(sawyer_safety_checker::CollisionPredictor::Request &req,
                      sawyer_safety_checker::CollisionPredictor::Response &res) {
    /*
     * sensor_msgs::JointState ref_joint_states
     * ---
     * std_msgs/Bool collision_state
     */
    robot_state::RobotState state(pPlanningScene->getCurrentState());
    robot_state::jointStateToRobotState(req.ref_joint_states, state);
//    ROS_INFO(state.getStateTreeString("robot_description").c_str());

    collision_detection::CollisionRequest colReq;
    colReq.contacts = true;
    colReq.max_contacts = 1000;
    collision_detection::CollisionResult colRes;
    pPlanningScene->checkCollision(colReq, colRes, state, acm);

    if (colRes.collision) {
        ROS_INFO("Contact Count: %d", colRes.contact_count);
        for (auto contact : colRes.contacts) {
            std::string contact_info = contact.first.first + " " + contact.first.second;
            ROS_INFO(contact_info.c_str());
        }
        res.collision_state.data = true;
    }

    return true;
}

bool checkCollision(sawyer_safety_checker::CollisionChecker::Request &req,
                    sawyer_safety_checker::CollisionChecker::Response &res) {
    /*
     * std_msgs/Empty req
     * ---
     * std_msgs/Bool collision_state
     */
    robot_state::RobotState state(pPlanningScene->getCurrentState());

    collision_detection::CollisionRequest colReq;
    colReq.contacts = true;
    colReq.max_contacts = 1000;
    collision_detection::CollisionResult colRes;
    pPlanningScene->checkCollision(colReq, colRes, state, acm);

    if (colRes.collision) {
        ROS_INFO("Contact Count: %d", colRes.contact_count);
        for (auto contact : colRes.contacts) {
            std::string contact_info = contact.first.first + " " + contact.first.second;
            ROS_INFO(contact_info.c_str());
        }
        res.collision_state.data = true;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sawyer_safety_checker_srv_node");

    ros::NodeHandle n;

    // Load robot model from ros parameter server.
    robot_model_loader::RobotModelLoader robotModelLoader = robot_model_loader::RobotModelLoader();
    robot_model::RobotModelPtr kinematicModel = robotModelLoader.getModel();
    pPlanningSceneParent = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematicModel));
    pPlanningScene = pPlanningSceneParent->diff();

//    ros::Publisher pubColObj = n.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
//    sleep(1.0);
//    moveit_msgs::CollisionObject co;
//    co.id = "table";
//    co.header.frame_id = "table";
//    co.operation = co.ADD;
//    co.primitives.resize(1);
//    co.primitives[0].dimensions.resize(3);
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.09;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.9;
//    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
//    co.primitive_poses.resize(1);
//    co.primitive_poses[0].position.x = 0.655;
//    co.primitive_poses[0].position.y = 0;
//    co.primitive_poses[0].position.z = -0.05;
//    co.primitive_poses[0].orientation.x = 0;
//    co.primitive_poses[0].orientation.y = 0;
//    co.primitive_poses[0].orientation.z = 0;
//    co.primitive_poses[0].orientation.w = 1.0;
//    pubColObj.publish(co);
//    sleep(1);
//    ros::spinOnce();

    // Disable self-collision detection
    acm = pPlanningSceneParent->getAllowedCollisionMatrix();
//    disableSelfCol();

    // Start collision predicting service.
    ros::ServiceServer srvCollisionPredict = n.advertiseService("predict_collision", &predictCollision);
    // Start collision checking service.
    ros::ServiceServer srvCollisionCheck = n.advertiseService("check_collision", &checkCollision);

    // Joint state updater.
    ros::Subscriber subJointStates = n.subscribe("/robot/joint_states", 1, &onJointStatesMsg);

    ros::spin();

    return 0;
}

