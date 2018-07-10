//
// Created by hejia on 6/28/18.
//

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/PlanningScene.h>
#include <shape_msgs/SolidPrimitive.h>

#include "sawyer_safety_checker/CollisionChecker.h"
#include "sawyer_safety_checker/CollisionPredictor.h"

// Base planning scene.
planning_scene::PlanningScenePtr pPlanningSceneParent;
// Base planning scene + additional objects actually used to check the collision.
planning_scene::PlanningScenePtr pPlanningScene;

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

    collision_detection::CollisionRequest colReq;
    collision_detection::CollisionResult colRes;
    pPlanningScene->checkCollision(colReq, colRes, state);

    if (colRes.collision) {
        res.collision_state.data = false;
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
    collision_detection::CollisionResult colRes;
    pPlanningScene->checkCollision(colReq, colRes, state);

    if (colRes.collision) {
        res.collision_state.data = false;
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

    // Start collision predicting service.
    ros::ServiceServer srvCollisionPredict = n.advertiseService("predict_collision", &predictCollision);
    // Start collision checking service.
    ros::ServiceServer srvCollisionCheck = n.advertiseService("check_collision", &checkCollision);

    // Joint state updater.
    ros::Subscriber subJointStates = n.subscribe("/robot/joint_states", 1, &onJointStatesMsg);

    ros::spin();

    return 0;
}

