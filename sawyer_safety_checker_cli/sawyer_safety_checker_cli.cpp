//
// Created by hejia on 7/9/18.
//

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/PlanningScene.h>
#include <shape_msgs/SolidPrimitive.h>

#include "sawyer_safety_checker/CollisionChecker.h"
#include "sawyer_safety_checker/CollisionPredictor.h"

// Store current robot joint states
sensor_msgs::JointState jointStates;

void onJointStatesMsg(const sensor_msgs::JointState& msg) {
    // Update jointStates
    jointStates = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sawyer_safety_checker_cli_node");

    ros::NodeHandle n;

    // Joint state updater.
    ros::Subscriber subJointStates = n.subscribe("/robot/joint_states", 1, &onJointStatesMsg);

    // Test add collision objects into planning scene
    ros::Publisher pubAddColObj = n.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
    sleep(1.0);
    moveit_msgs::CollisionObject co;
    co.id = "table";
    co.header.frame_id = "sawyer";
    co.operation = co.ADD;
    co.primitives.resize(1);
    co.primitives[0].dimensions.resize(3);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.09;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.9;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
    co.primitive_poses[0].position.x = 0.655;
    co.primitive_poses[0].position.y = 0;
    co.primitive_poses[0].position.z = -0.05;
    co.primitive_poses[0].orientation.x = 0;
    co.primitive_poses[0].orientation.y = 0;
    co.primitive_poses[0].orientation.z = 0;
    co.primitive_poses[0].orientation.w = 1.0;
    pubAddColObj.publish(co);

    sleep(1);

    // Start collision predicting client.
    ros::ServiceClient cliCollisionPredictor = n.serviceClient<sawyer_safety_checker::CollisionPredictor>("predict_collision");
    // Start collision checking client.
    ros::ServiceClient cliCollisionChecker = n.serviceClient<sawyer_safety_checker::CollisionChecker>("check_collision");

    sawyer_safety_checker::CollisionChecker srvChecker;
    sawyer_safety_checker::CollisionPredictor srvPredictor;

    // Test Collision Checker
    std_msgs::Empty req;
    srvChecker.request.req = req;
    if (cliCollisionChecker.call(srvChecker)) {
        ROS_INFO("Collision State: %d", srvChecker.response.collision_state);
    } else {
        ROS_ERROR("Failed to call service CollisionChecker");
        return 1;
    }

    // Test Collision Predictor
    srvPredictor.request.ref_joint_states = jointStates;
    if (cliCollisionPredictor.call(srvPredictor)) {
        ROS_INFO("Collision State: %d", srvChecker.response.collision_state);
    } else {
        ROS_ERROR("Failed to call service CollisionPredictor");
        return 1;
    }

    return 0;
}