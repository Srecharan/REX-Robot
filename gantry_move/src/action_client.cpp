#include "gantry_action_client.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>  // Include for the Float64 message type
#include <std_srvs/Trigger.h>  // Include for the Trigger service

typedef actionlib::SimpleActionClient<gantry_move::gantry_moveAction> Client;

class move_robot {
public:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object1, collision_object2;
    moveit_msgs::ObjectColor collision_object_color1, collision_object_color2;
    moveit_msgs::PlanningScene planning_scene;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::ObjectColor> ObjectColors;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::vector<double> joint_group_positions;
    std::vector<double> grasp_positions;
    std::vector<double> pre_grasp_positions;  // Vector to store the pre-grasp positions
    std::vector<double> grasp_diff;  // Vector to store the grasp difference
    moveit::core::RobotStatePtr current_state;
    moveit::core::RobotStatePtr ee_state;
    const robot_state::JointModelGroup* joint_model_group;
    const robot_state::JointModelGroup* ee_model_group;

    geometry_msgs::Pose box_pose1, box_pose2;

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

    // Add this to your class
    ros::Publisher gantry_position_pub;
    ros::ServiceClient trigger_camera_client;  // Add a service client for triggering the camera

    move_robot() : ac_("gantry_move_server", true), grasp_point_received(false), pre_grasp_point_received(false), new_grasp_point(false), yaw_angle_received(false), grasp_diff_received(false) {
        ros::param::set("/trigger_flash", "false");
        ROS_WARN("Waiting for action server to start.");

        cout << "..... ROBOT_MODE: " << ROBOT_MODE << endl;

        pub_linear_calib = nh.advertise<std_msgs::String>("/do_calib", 1);
        planning_scene_pub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

        sub_go_center = nh.subscribe("/go_center", 5, &move_robot::go_to_center_, this);
        sub_go_test = nh.subscribe("/go_test", 5, &move_robot::go_to_test_, this);
        sub_go_rpy = nh.subscribe("/go_rpy", 5, &move_robot::go_rpy_, this);  // New subscriber for RPY control
        sub_grasp_point = nh.subscribe("grasp_point_topic", 1, &move_robot::grasp_point_callback_, this);
        sub_pre_grasp_point = nh.subscribe("pre_grasp_point_topic", 1, &move_robot::pre_grasp_point_callback_, this);  // Subscriber for pre-grasp point
        sub_grasp_diff = nh.subscribe("grasp_diff_topic", 1, &move_robot::grasp_diff_callback_, this);  // Subscriber for grasp difference
        sub_grasp_angle = nh.subscribe("grasp_angle_topic", 1, &move_robot::grasp_angle_callback_, this);  // Subscriber for yaw angle

        // In the constructor of your class (move_robot) add this
        gantry_position_pub = nh.advertise<std_msgs::String>("/gantry_position_topic", 1);

        // Initialize the service client for triggering the camera
        trigger_camera_client = nh.serviceClient<std_srvs::Trigger>("/trigger_firefly_camera");

        ac_.waitForServer(); // waits forever;

        move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        moveit_visual_tools::MoveItVisualTools visual_tools("world");

        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;

        init_setup_();
    }

    void init_setup_() {
        ROS_INFO("In the init_setup_ function .......");

        moveit::planning_interface::MoveGroupInterface &move_group = *move_group_ptr;
        moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

        move_group.setStartStateToCurrentState();
        usleep(50000);

        visual_tools.deleteAllMarkers();
        visual_tools.trigger();
        usleep(5000);

        visual_tools.loadRemoteControl();
        text_pose.translation().z() = 2.0;
        text_pose.translation().y() = 0.5;
        visual_tools.publishText(text_pose, "Initial Setup Complete....",
                                 rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();

        visual_tools.publishText(text_pose, "Visualizing target(s)...", rvt::WHITE,
                                 rvt::XLARGE);
    }

    void grasp_point_callback_(const std_msgs::String::ConstPtr& msg) {
        std::istringstream iss(msg->data);
        double x, y, z;
        iss >> x >> y >> z;

        // Ensure vectors are initialized properly
        if (grasp_positions.size() < 3) grasp_positions.resize(3);
        if (joint_group_positions.size() < 3) joint_group_positions.resize(3);

        // Assign the received values to joint_group_positions
        grasp_positions = {x, y, z};

        ROS_INFO("Received grasping point coordinates X=%.6f, Y=%.6f, Z=%.6f",
                 grasp_positions[0],
                 grasp_positions[1],
                 grasp_positions[2]);

        grasp_point_received = true;
        new_grasp_point = true; // Indicate that a new grasp point has been received
    }

    void pre_grasp_point_callback_(const std_msgs::String::ConstPtr& msg) {
        std::istringstream iss(msg->data);
        double x, y, z;
        iss >> x >> y >> z;

        // Ensure vectors are initialized properly
        if (pre_grasp_positions.size() < 3) pre_grasp_positions.resize(3);
        pre_grasp_positions = {x, y, z};

        ROS_INFO("Received pre-grasping point coordinates X=%.6f, Y=%.6f, Z=%.6f",
                 pre_grasp_positions[0],
                 pre_grasp_positions[1],
                 pre_grasp_positions[2]);

        pre_grasp_point_received = true;
    }

    void grasp_diff_callback_(const std_msgs::String::ConstPtr& msg) {
        std::istringstream iss(msg->data);
        double x, y, z;
        iss >> x >> y >> z;

        // Ensure vectors are initialized properly
        if (grasp_diff.size() < 3) grasp_diff.resize(3);
        grasp_diff = {x, y, z};

        ROS_INFO("Received grasping point difference: X=%.6f, Y=%.6f, Z=%.6f",
                 grasp_diff[0],
                 grasp_diff[1],
                 grasp_diff[2]);

        grasp_diff_received = true;
    }

    void grasp_angle_callback_(const std_msgs::Float64::ConstPtr& msg) {
        yaw = msg->data;  // Update yaw with the received angle in radians
        ROS_INFO("Received grasping yaw angle: %.6f radians", yaw);
        yaw_angle_received = true;  // Mark that the yaw angle has been received
    }

    void go_to_center_(const std_msgs::StringConstPtr& msg) {
        ROS_INFO("Inside the go_to_center subscriber....");

        ros::param::set("/trigger_flash", "false");
        moveit::planning_interface::MoveGroupInterface &move_group = *move_group_ptr;
        moveit_visual_tools::MoveItVisualTools visual_tools("world");

        move_group.setStartStateToCurrentState();
        ros::Duration(1.0).sleep();

        joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        // ----------------------------------
        // Step 1: Move to the "loading_phase" position
        // ----------------------------------
        ROS_INFO("Moving to the 'loading_phase' position...");
        move_group.setNamedTarget("loading_phase");
        move_group.setMaxVelocityScalingFactor(0.2);  // Reduce speed to 0.2
        move_group.setMaxAccelerationScalingFactor(0.2);  // Reduce acceleration to 0.

        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to 'loading_phase'.");
            return;
        }
        ros::Duration(7.0).sleep();

        // ----------------------------------
        // Step 2: Move to the "mid_center" position
        // ----------------------------------
        ROS_INFO("Moving to the 'mid_center' position...");
        move_group.setNamedTarget("mid_center");
        move_group.setMaxVelocityScalingFactor(0.2);  // Maintain speed at 0.2
        move_group.setMaxAccelerationScalingFactor(0.2);

        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to 'mid_center'.");
            return;
        }

        // ----------------------------------
        // Step 3: Move to the specified coordinates
        // ----------------------------------
        ROS_INFO("Moving to specified coordinates...");

        current_state = move_group.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions[0] = 0.678;  // Update these values to goto the position where image is captured
        joint_group_positions[1] = 0.100;  // Modify Y as per the requirement
        joint_group_positions[2] = 0.272;

        move_group.setJointValueTarget(joint_group_positions);
        move_group.setPlanningTime(5);
        move_group.allowReplanning(true);
        move_group.setNumPlanningAttempts(5);

        visual_tools.deleteAllMarkers();

        visual_tools.publishText(text_pose, "Homing....Click Next....", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();

        move_group.setStartStateToCurrentState();
        usleep(50000);
        move_group.setMaxVelocityScalingFactor(0.2);  // Maintain speed at 0.2
        move_group.setMaxAccelerationScalingFactor(0.2);

        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to the specified coordinates.");
            return;
        }

        // Log the actual joint positions after moving to the center
        current_state = move_group.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        ROS_INFO("Actual Joint Positions after centering: X=%.6f, Y=%.6f, Z=%.6f",
                 joint_group_positions[0],
                 joint_group_positions[1],
                 joint_group_positions[2]);

        // Publish the gantry position
        std_msgs::String gantry_position_msg;
        std::stringstream ss;
        ss << joint_group_positions[0] << " " << joint_group_positions[1] << " " << joint_group_positions[2];
        gantry_position_msg.data = ss.str();
        gantry_position_pub.publish(gantry_position_msg);
        ROS_INFO("Published gantry position: %s", gantry_position_msg.data.c_str());

        // Trigger the camera to capture an image
        std_srvs::Trigger srv;
        if (trigger_camera_client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Camera triggered successfully: %s", srv.response.message.c_str());
            } else {
                ROS_ERROR("Failed to trigger the camera: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed to call service to trigger the camera.");
        }

        ros::param::set("/centered", 1);
        ROS_INFO("Completed centering....");
    }
    void go_to_test_(const std_msgs::StringConstPtr& msg) {
        ROS_INFO("Inside the go_to_test subscriber....");

        ros::param::set("/trigger_flash", "false");

        moveit::planning_interface::MoveGroupInterface &move_group = *move_group_ptr;
        moveit_visual_tools::MoveItVisualTools visual_tools("world");

        // Ensure the current state is up-to-date
        move_group.setStartStateToCurrentState();
        ros::Duration(1.0).sleep();

        joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        // Retrieve current joint positions
        current_state = move_group.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // Log current positions
        ROS_INFO("Current Joint Positions: X=%.6f, Y=%.6f, Z=%.6f",
                 joint_group_positions[0],
                 joint_group_positions[1],
                 joint_group_positions[2]);

        // Check if grasp positions, pre-grasp positions, grasp difference, and yaw angle are received
        if (!grasp_point_received || !pre_grasp_point_received || !grasp_diff_received || !yaw_angle_received) {
            ROS_WARN("Grasp point, pre-grasp point, grasp difference, or yaw angle not received yet. Aborting go_to_test_ operation.");
            return;
        }

        // ----------------------------------
        // Step 1: Update the Yaw axis in the internal state
        // ----------------------------------
        double yaw_angle = yaw;  // Assuming `yaw` is already set from the leaf_grasp_node

        // Get the current pose of the end-effector
        geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

        // Apply the yaw rotation to the current orientation
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(current_pose.orientation, q_orig);

        q_rot.setRPY(0, 0, yaw_angle);
        q_new = q_rot * q_orig;
        q_new.normalize();

        current_pose.orientation = tf2::toMsg(q_new);

        // Update the internal joint states with the new yaw value without moving the hardware
        current_state->setJointGroupPositions(joint_model_group, joint_group_positions);
        joint_group_positions[3] = yaw_angle;  // Assuming the yaw joint is at index 3

        ROS_INFO("Updated internal yaw angle to %.6f radians without moving hardware", yaw_angle);

        // ----------------------------------
        // Step 2: Move to the pre-grasp position
        // ----------------------------------
        std::vector<double> pre_grasp_target_positions = joint_group_positions;
        pre_grasp_target_positions[0] += pre_grasp_positions[0];
        pre_grasp_target_positions[1] += pre_grasp_positions[1];
        pre_grasp_target_positions[2] += pre_grasp_positions[2];

        move_group.setJointValueTarget(pre_grasp_target_positions);
        move_group.setPlanningTime(10);
        move_group.setNumPlanningAttempts(10);
        move_group.allowReplanning(true);
        move_group.setMaxVelocityScalingFactor(0.2);
        move_group.setMaxAccelerationScalingFactor(0.2);

        ROS_INFO("Moving to pre-grasp position...");
        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to pre-grasp position.");
            return;
        }

        ros::Duration(2.0).sleep();

        // ----------------------------------
        // Step 3: Move to the final grasp position
        // ----------------------------------
        std::vector<double> final_grasp_target_positions = pre_grasp_target_positions;
        final_grasp_target_positions[0] += grasp_diff[0];
        final_grasp_target_positions[1] += grasp_diff[1];
        final_grasp_target_positions[2] += grasp_diff[2];

        move_group.setJointValueTarget(final_grasp_target_positions);
        move_group.setPlanningTime(10);
        move_group.setNumPlanningAttempts(10);
        move_group.allowReplanning(true);

        ROS_INFO("Moving to final grasp position...");
        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to final grasp position.");
            return;
        }

        // ----------------------------------
        // Step 4: Trigger the grasp command by publishing to /go_grasp
        // ----------------------------------
        ROS_INFO("Triggering the /go_grasp command...");
        ros::Publisher grasp_pub = nh.advertise<std_msgs::String>("/go_grasp", 1);
        std_msgs::String grasp_msg;
        grasp_msg.data = "grasp";
        grasp_pub.publish(grasp_msg);

        ros::Duration(20.0).sleep();   // Wait for the grasp to be executed

        // ----------------------------------
        // Step 5: Move back to the pre-grasp position
        // ----------------------------------
        ROS_INFO("Moving back to the pre-grasp position...");
        move_group.setJointValueTarget(pre_grasp_target_positions);
        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move back to the pre-grasp position.");
            return;
        }

        // ----------------------------------
        // Step 6: Move to the "home" location
        // ----------------------------------
        ROS_INFO("Moving to 'home' location...");
        move_group.setNamedTarget("home");
        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to 'home' location.");
            return;
        }

        // ----------------------------------
        // Step 7: Move to the "front_right" location
        // ----------------------------------
        ROS_INFO("Moving to 'front_right' location...");
        move_group.setNamedTarget("front_right");
        if (move_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to 'front_right' location.");
            return;
        }

        ROS_INFO("Completed the go_to_test_ operation.");
    }

    void go_rpy_(const std_msgs::StringConstPtr& msg) {  // Function retained but not used in go_to_test_
        ROS_INFO("Inside the go_rpy subscriber....");

        std::istringstream iss(msg->data);
        double roll_input, pitch_input, yaw_input;
        iss >> roll_input >> pitch_input >> yaw_input;

        ROS_INFO("Received RPY values: Roll=%.6f, Pitch=%.6f, Yaw=%.6f",
                 roll_input, pitch_input, yaw_input);

        moveit::planning_interface::MoveGroupInterface &move_group = *move_group_ptr;

        move_group.setStartStateToCurrentState();
        ros::Duration(1.0).sleep();

        joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        current_state = move_group.getCurrentState();

        // Get the current pose of the end-effector
        geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

        // Apply the desired RPY values to the current orientation
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(current_pose.orientation, q_orig);

        ROS_INFO("Original Orientation: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
                 current_pose.orientation.x, current_pose.orientation.y,
                 current_pose.orientation.z, current_pose.orientation.w);

        // Create rotation from the input values
        q_rot.setRPY(roll_input, pitch_input, yaw_input);

        // Combine with the original orientation
        q_new = q_rot * q_orig;
        q_new.normalize();

        current_pose.orientation = tf2::toMsg(q_new);

        ROS_INFO("New Orientation: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
                 current_pose.orientation.x, current_pose.orientation.y,
                 current_pose.orientation.z, current_pose.orientation.w);

        move_group.setPoseTarget(current_pose);

        move_group.setPlanningTime(15);  // Increase planning time
        move_group.allowReplanning(true);
        move_group.setNumPlanningAttempts(15);

        move_group.setMaxVelocityScalingFactor(0.1);  // Reduce speed to 10% of the maximum
        move_group.setMaxAccelerationScalingFactor(0.1);  // Reduce acceleration to 10% of the maximum

        moveit::planning_interface::MoveItErrorCode success = move_group.move();

        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("RPY motion successful.");
        } else {
            ROS_ERROR("RPY motion failed. Error code: %d", (int)success.val);
            ROS_WARN("The motion may have failed due to joint limits or other constraints.");
        }
    }

    void get_eepose(geometry_msgs::PoseStamped& pose_) {
        moveit::planning_interface::MoveGroupInterface &move_group = *move_group_ptr;
        moveit_visual_tools::MoveItVisualTools visual_tools("world");

        ee_model_group = move_group.getCurrentState()->getJointModelGroup(EE_GROUP);

        pose_ = move_group.getCurrentPose("ee_link");
    }

    tf2_ros::Buffer* tf2_buffer;

protected:
    double roll, pitch, yaw;  // yaw now stores the angle from the leaf_grasp_node
    geometry_msgs::PoseStamped ee_pose_;
    Client ac_;
    ros::NodeHandle nh;
    ros::Subscriber sub_go_to_point, sub_sweep, sub_soya, sub_get_images,
        sub_experiment_01, sub_go_home, sub_sliding_calib, sub_go_center,
        sub_go_test, sub_clear_octomap, sub_custom_octomap, sub_grasp_point, sub_pre_grasp_point, sub_grasp_diff, sub_grasp_angle, sub_go_rpy;  // Added sub_grasp_angle, sub_pre_grasp_point, sub_grasp_diff, and sub_go_rpy
    ros::Publisher pub__, pub_linear_calib, planning_scene_pub;

private:
    bool grasp_point_received;
    bool pre_grasp_point_received;
    bool grasp_diff_received;
    bool new_grasp_point;
    bool yaw_angle_received;  // Track if the yaw angle has been received
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr;
};

ros::Publisher joints_pub_;

int main(int argc, char** argv) {
    ros::init(argc, argv, "gantry_move_client");
    ros::param::set("/trigger_flash", "false");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh_;
    joints_pub_ = nh_.advertise<sensor_msgs::JointState>("/theia/joint_states", 10);

    move_robot mR;
    ros::waitForShutdown();
    return 0;
}
