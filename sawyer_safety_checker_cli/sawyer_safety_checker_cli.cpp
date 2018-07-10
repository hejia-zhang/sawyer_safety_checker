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
    sleep(1);
    ros::spinOnce();

    // Test add collision objects into planning scene
    ros::Publisher pubColObj = n.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
    sleep(1.0);
    moveit_msgs::CollisionObject co;
    co.id = "table";
    co.header.frame_id = "table";
    co.operation = co.ADD;
    co.primitives.resize(1);
    co.primitives[0].dimensions.resize(3);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 100;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 100;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = 0.655;
    co.primitive_poses[0].position.y = 0;
    co.primitive_poses[0].position.z = -0.05;
    co.primitive_poses[0].orientation.x = 0;
    co.primitive_poses[0].orientation.y = 0;
    co.primitive_poses[0].orientation.z = 0;
    co.primitive_poses[0].orientation.w = 1.0;
    pubColObj.publish(co);
    sleep(1);
    ros::spinOnce();

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
    std::map<std::string, double> initial_joints = std::map<std::string, double>();
    initial_joints["right_j0"] = -0.140923828125;
    initial_joints["right_j1"] = -1.2789248046875;
    initial_joints["right_j2"] = -3.043166015625;
    initial_joints["right_j3"] = -2.139623046875;
    initial_joints["right_j4"] = -0.047607421875;
    initial_joints["right_j5"] = -0.7052822265625;
    initial_joints["right_j6"] = -1.4102060546875;

    srvPredictor.request.ref_joint_states = jointStates;
    for (int i = 0; i < srvPredictor.request.ref_joint_states.name.size(); i++) {
        if (srvPredictor.request.ref_joint_states.name[i] == "right_j0") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j0"];
        } else if (srvPredictor.request.ref_joint_states.name[i] == "right_j1") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j1"];
        } else if (srvPredictor.request.ref_joint_states.name[i] == "right_j2") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j2"];
        } else if (srvPredictor.request.ref_joint_states.name[i] == "right_j3") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j3"];
        } else if (srvPredictor.request.ref_joint_states.name[i] == "right_j4") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j4"];
        } else if (srvPredictor.request.ref_joint_states.name[i] == "right_j5") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j5"];
        } else if (srvPredictor.request.ref_joint_states.name[i] == "right_j6") {
            srvPredictor.request.ref_joint_states.position[i] = initial_joints["right_j6"];
        }
    }

    for (int i = 0; i < 20; i++){
        if (cliCollisionPredictor.call(srvPredictor)) {
            ROS_INFO("Table height: %f", co.primitive_poses[0].position.z);
            ROS_INFO("Collision State: %d", srvChecker.response.collision_state);
        } else {
            ROS_ERROR("Failed to call service CollisionPredictor");
            return 1;
        }
        co.primitive_poses[0].position.z += 0.05;
        co.operation = co.MOVE;
        pubColObj.publish(co);
        sleep(1);
        ros::spinOnce();
    }

    co.operation = co.REMOVE;
    pubColObj.publish(co);
    sleep(1);
    ros::spinOnce();

    return 0;
}