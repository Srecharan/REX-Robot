//#include <cstdlib>
#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
//#include <stdint-gcc.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include "gantry_driver/Axis.h"
#include "gantry_driver/Supervisor.h"
#include "gantry_driver/inc/inc-pub/pubSysCls.h"
#include "gantry_driver/server.h"

#pragma clang diagnostic ignored                                               \
    "-Wzero-as-null-pointer-constant" // this is a QT thing

using namespace std;
using namespace sFnd;

void MN_DECL AttentionDetected(const mnAttnReqReg& detected) {
  // Make a local, non-const copy for printing purposes
  mnAttnReqReg myAttns = detected;
  // Create a buffer to hold the attentionReg information
  char attnStringBuf[512];
  // Load the buffer with the string representation of the attention information
  myAttns.AttentionReg.StateStr(attnStringBuf, 512);
}

class multi_servo_driver {

public:
  multi_servo_driver(std::string name)
      : as_(nh_, "follow_joint_trajectory", false),
        action_name_("follow_joint_trajectory") {

    portCount = 0;
    init_done = false;

    start_pos_ = 0.0;
    calib_pose = 0.0;
    nodeTypesGood = false;
    accessLvlsGood = false;

    ros::param::set("/GANTRY_CALIBRATED_X", false);
    ros::param::set("/GANTRY_CALIBRATED_Y", false);
    ros::param::set("/GANTRY_CALIBRATED_Z", false);
    ros::param::set("/GANTRY_CALIBRATED_YAW", false);

    init_done = false;

    init();

    SensorTrigger = nh_.advertise<std_msgs::String>("commands_to_syncbox", 10);
    pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    sub__ = nh_.subscribe("/do_calib_n", 1, &multi_servo_driver::calib, this);
    sub___ = nh_.subscribe("/ping", 1, &multi_servo_driver::js_pub, this);
    // sub_torque =
    //    nh_.subscribe("/torque", 1, &multi_servo_driver::toerqueOFF, this);

    as_.registerGoalCallback(boost::bind(&multi_servo_driver::goalCB, this));
    as_.registerPreemptCallback(
        boost::bind(&multi_servo_driver::preemptCB, this));

    as_.start();
  }

  /*  BACKUP of the JS publisher */
  void js_pub(const std_msgs::StringConstPtr& msg) {

    if (!init_done) {

      ROS_WARN("init() check failed... reinitializing....");
      init();
      ROS_INFO(".... init done...");
    }

    sensor_msgs::JointState joint_msg;

    double cur_pos = 0.0;
    double cur_vel = 0.0;
    double _positions = 0.0;
    double _velocities = 0.0;
    unsigned int jount_counter = 0;
    std::vector<geometry_msgs::TransformStamped> gantry_tf_vec;
    geometry_msgs::TransformStamped gantry_tf;

    // for (size_t portcount = 0; portcount < 2; portcount++) {
    size_t portcount = 0;                   // based on 4/13/2022
    IPort& myPort = myMgr.Ports(portcount); // 0 is for serial 1 for USB

    // cout << "Node count for sanity check...:" << myPort.NodeCount() << endl;

    for (size_t i = 0; i < myPort.NodeCount(); i++) {

      INode& theNode = myPort.Nodes(i);

      if (theNode.Info.SerialNumber.Value() == GANTRY_X1_SERIAL) {
        ROS_INFO_ONCE("Caught redundant x axis serial: %d",
                      theNode.Info.SerialNumber.Value());
        continue;
      }

      else {
        cur_pos = theNode.Motion.PosnMeasured.ValueDouble::Value(true);
        cur_vel = theNode.Motion.VelMeasured.ValueDouble::Value(true);

        if (theNode.Info.SerialNumber.Value() == GANTRY_Z_SERIAL) {

          _positions = (calib_pose + cur_pos) * METERS_TO_COUNTS_Z; // meter
          _velocities = cur_vel * METERS_TO_COUNTS_Z;               // m/s
        }

        else if (theNode.Info.SerialNumber.Value() == GANTRY_YAW_SERIAL) {

          _positions = (calib_pose + cur_pos) * METERS_TO_COUNTS_YAW; // rad
          _velocities = cur_vel * METERS_TO_COUNTS_YAW;               // rad/s
        }

        else {
          _positions = (calib_pose + cur_pos) * METERS_TO_COUNTS_XY; // meter
          _velocities = cur_vel * METERS_TO_COUNTS_XY;               // m/s
        }

        joint_msg.name.push_back(joint_names[jount_counter]);

        if (joint_names[jount_counter] == "gantryY") {

          // joint_msg.header.stamp = ros::Time::now();
          joint_msg.position.push_back(1.0 * _positions);  // m/s
          joint_msg.velocity.push_back(1.0 * _velocities); // m/s
          joint_msg.effort.push_back(0.0); // typically torque; not reqired here

          gantry_tf.header.frame_id = "gantryX_link";
          gantry_tf.child_frame_id = "gantryY_link";
          gantry_tf.transform.translation.x = 0.07700000;
          gantry_tf.transform.translation.y = 0.00000000 + _positions;
          gantry_tf.transform.translation.z = 1.49793000;

          gantry_tf.transform.rotation.x = 0.00000000;
          gantry_tf.transform.rotation.y = 0.00000000;
          gantry_tf.transform.rotation.z = 0.00000000;
          gantry_tf.transform.rotation.w = 1.00000000;
          // gantry_tf.header.stamp = ros::Time::now();

          gantry_tf_vec.push_back(gantry_tf);
        } else if (joint_names[jount_counter] == "gantryZ") {

          // joint_msg.header.stamp = ros::Time::now();
          joint_msg.position.push_back(-1.0 * _positions);  // m/s
          joint_msg.velocity.push_back(-1.0 * _velocities); // m/s
          joint_msg.effort.push_back(0.0); // typically torque; not reqired here

          gantry_tf.header.frame_id = "gantryY_link";
          gantry_tf.child_frame_id = "gantryZ_link";
          gantry_tf.transform.translation.x = 0.09843000;
          gantry_tf.transform.translation.y = 0.00000000;
          gantry_tf.transform.translation.z = -0.5334000 + _positions;

          gantry_tf.transform.rotation.x = 0.00000000;
          gantry_tf.transform.rotation.y = 0.00000000;
          gantry_tf.transform.rotation.z = 0.00000000;
          gantry_tf.transform.rotation.w = 1.00000000;
          // gantry_tf.header.stamp = ros::Time::now();

          gantry_tf_vec.push_back(gantry_tf);
        } else if (joint_names[jount_counter] == "gantryYAW") { // YAW

          // joint_msg.header.stamp = ros::Time::now();
          joint_msg.position.push_back(1 * _positions);  // radian // was -1
          joint_msg.velocity.push_back(1 * _velocities); // radian/s   // was -1
          joint_msg.effort.push_back(0.0); // typically torque; not reqired here

          gantry_tf.header.frame_id = "gantryZ_link";
          gantry_tf.child_frame_id = "gantryYAW_link";
          gantry_tf.transform.translation.x = 0.07500000;
          gantry_tf.transform.translation.y = 0.00000000;
          gantry_tf.transform.translation.z = 0.10000000;

          tf::Quaternion Q;

          Q.setRPY(0.0, 0.0, -1 * _positions);

          // cout<<"Quaternion(x,y,z,w): "<<Q.x()<<", "<<Q.y()<<", "<<Q.z()<<",
          // "<<Q.w()<<endl;
          // is ccw positive? and it should be in radian
          // don't forget this is in quaternion

          gantry_tf.transform.rotation.x = Q.x();
          gantry_tf.transform.rotation.y = Q.y();
          gantry_tf.transform.rotation.z = Q.z();
          gantry_tf.transform.rotation.w = Q.w();
          // gantry_tf.header.stamp = ros::Time::now();

          gantry_tf_vec.push_back(gantry_tf);

          /*           //   This moves tot he dynamixel driver

          // Experimental  TF for th ee_link here

          gantry_tf.header.frame_id = "gantryYAW_link";
          gantry_tf.child_frame_id = "ee_link";
          gantry_tf.transform.translation.x = 0.00000000;
          gantry_tf.transform.translation.y = 0.00000000;
          gantry_tf.transform.translation.z = -0.2400000;

          gantry_tf.transform.rotation.x = 0.00000000;
          gantry_tf.transform.rotation.y = 0.00000000;
          gantry_tf.transform.rotation.z = 0.00000000;
          gantry_tf.transform.rotation.w = 1.00000000;

          gantry_tf_vec.push_back(gantry_tf);
          */

        } else if (joint_names[jount_counter] == "gantryX") { // X

          // joint_msg.header.stamp = ros::Time::now();
          joint_msg.position.push_back(_positions);  // m/s
          joint_msg.velocity.push_back(_velocities); // m/s
          joint_msg.effort.push_back(0.0); // typically torque; not reqired here

          gantry_tf.header.frame_id = "base_link";
          gantry_tf.child_frame_id = "gantryX_link";
          gantry_tf.transform.translation.x = 0.00000000 + _positions;
          gantry_tf.transform.translation.y = 0.00000000;
          gantry_tf.transform.translation.z = 0.14288000;

          gantry_tf.transform.rotation.x = 0.00000000;
          gantry_tf.transform.rotation.y = 0.00000000;
          gantry_tf.transform.rotation.z = 0.00000000;
          gantry_tf.transform.rotation.w = 1.00000000;
          // gantry_tf.header.stamp = ros::Time::now();

          gantry_tf_vec.push_back(gantry_tf);
        } else {

          ROS_ERROR("INVALID JOINT with name %s: \n",
                    joint_names[jount_counter].c_str());
        }
        jount_counter++;
      }
    }
    //}
    joint_msg.header.stamp = ros::Time::now();
    gantry_tf.header.stamp = ros::Time::now();
    pub_.publish(joint_msg);
    broadcaster.sendTransform(gantry_tf_vec);
  }

  int init() {

    init_done = false;
    // SysManager::FindComHubPorts(comHubPorts);
    comHubPorts.push_back("/dev/ttyS7");
    // comHubPorts.push_back("/dev/ttyXRUSB0");
    ROS_INFO_ONCE("Found: %ld SC Hub(s)\n", comHubPorts.size());

    for (portCount = 0;
         portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
         portCount++) {

      myMgr.ComHubPort(
          portCount,
          comHubPorts[portCount]
              .c_str()); // define the first SC,
                         // update 4/13/2022. Connect SC HUBs in series
                         // then only one connection port would be necessary
    }

    if (portCount <= 0) {
      ROS_ERROR("Unable to locate SC hub port.....");
      return -1;
    }

    // Create a list of axes - one per node
    vector<Axis*> listOfAxes;

    // Assume that the nodes are not of the not right type and that also the app
    // has no full control
    nodeTypesGood = false;
    accessLvlsGood = false;

    //--- serial numbers ----
    GANTRY_X1_SERIAL = 57702051;
    GANTRY_X2_SERIAL = 57702033;
    GANTRY_Y_SERIAL = 57900033;
    GANTRY_Z_SERIAL = 57700742;
    GANTRY_YAW_SERIAL = 67604431;

    // default values -- will be overridden later
    ros::param::set("/GANTRY_X1_NODE", 0);
    ros::param::set("/GANTRY_X2_NODE", 1);
    ros::param::set("/GANTRY_Y_NODE", 2);
    ros::param::set("/GANTRY_Z_NODE", 3);
    ros::param::set("/GANTRY_YAW_NODE", 4);

    ros::param::set("/GANTRY_CALIBRATED_X", false);
    ros::param::set("/GANTRY_CALIBRATED_Y", false);
    ros::param::set("/GANTRY_CALIBRATED_Z", false);
    ros::param::set("/GANTRY_CALIBRATED_YAW", false);

    joint_names.resize(0);
    // resetting the joint names in case the next cycle something dies

    joint_names.push_back(std::string("gantryX"));
    joint_names.push_back(std::string("gantryY"));
    joint_names.push_back(std::string("gantryYAW"));
    // for now in alphabetical order 4/18/2022
    joint_names.push_back(std::string("gantryZ"));

    myMgr.PortsOpen(1); // portCount Open the port

    // Note 2022. Start another FOR loop for numer of ports
    // SC-HUB 1 has serial com for 4 motors for 3 axis
    // SC-HUB 2 has USB com for 1 yaw axis motor
    // update: 4/7/2022: use schub in serial dsisy chain so both can be
    // controlled with one serial port

    // int NodeCounter = 0;

    int _node_count = 0;
    // for (unsigned int PortCount = 0; PortCount < portCount; PortCount++) {

    size_t PortCount = 0;

    IPort& myPort = myMgr.Ports(PortCount); // 0 for now
    cout << " Port: " << comHubPorts[PortCount]
         << " NetNo.: " << myPort.NetNumber()
         << " State: " << myPort.OpenState() << " Nodes: " << myPort.NodeCount()
         << endl;

    /*
    if (myPort.NodeCount() < NUM_OF_MOTORS) { // we have 2 motors in the x
    axis ROS_ERROR("ERROR... missing a node.... manuall check before running
    the " "robot !!!"); return -1;
    }
    */

    myPort.Adv.Attn.Enable(true);
    // The attentions will be handled by the individual nodes, but register
    // a handler at the port level, just for illustrative purposes.
    myPort.Adv.Attn.AttnHandler(AttentionDetected);

    for (size_t i = 0; i < myPort.NodeCount(); i++) {

      INode& theNode = myPort.Nodes(i);
      // INode* a;
      // a = &myPort.Nodes(1);

      theNode.EnableReq(false); // Ensure Node is disabled before loading
                                // config file We don't have to
      myMgr.Delay(200);
      theNode.Status.AlertsClear();

      // if (theNode.Info.NodeType() == IInfo::CLEARPATH_SC_ADV) {
      //  cout << "My Node is an Advanced ClearPath-SC" << endl;
      // return(2);
      //}

      // Make sure we are talking to a ClearPath SC
      if (theNode.Info.NodeType() != IInfo::CLEARPATH_SC_ADV) {
        ROS_ERROR("---> ERROR: Uh-oh! Node %d is not a ClearPath-SC Advanced "
                  "Motor\n",
                  static_cast<unsigned int>(i));
        nodeTypesGood = false;
        return -1;
      }

      else
        nodeTypesGood = true;

      if (!theNode.Setup.AccessLevelIsFull()) {
        printf("---> ERROR: Oh snap! Access level is not good for node %u\n",
               static_cast<unsigned int>(i));
        accessLvlsGood = false; // false
      }

      else
        accessLvlsGood = true;

      // cout <<i<< ". Node[" << i << "]\t:"<<theNode.Info.NodeType() << endl;
      // cout <<i<< ". userID\t: " << theNode.Info.UserID.Value() << endl;
      // cout <<i<< ". FW version\t: " << theNode.Info.FirmwareVersion.Value()
      // << endl;
      // cout << i << ". Serial \t: " << theNode.Info.SerialNumber.Value() <<
      // endl;

      if (theNode.Info.SerialNumber.Value() == GANTRY_X1_SERIAL) {
        cout << "check serial: " << theNode.Info.SerialNumber.Value() << ", "
             << GANTRY_X1_SERIAL << endl;
        ros::param::set(
            "/GANTRY_X1_NODE",
            _node_count); // ros::param::set("/GANTRY_X1_NODE", int(i));
      }

      if (theNode.Info.SerialNumber.Value() == GANTRY_X2_SERIAL) {
        cout << "check serial: " << theNode.Info.SerialNumber.Value() << ", "
             << GANTRY_X2_SERIAL << endl;
        ros::param::set("/GANTRY_X2_NODE", _node_count);
      }

      if (theNode.Info.SerialNumber.Value() == GANTRY_Y_SERIAL) {
        cout << "check serial: " << theNode.Info.SerialNumber.Value() << ", "
             << GANTRY_Y_SERIAL << endl;
        ros::param::set("/GANTRY_Y_NODE", _node_count);
      }

      if (theNode.Info.SerialNumber.Value() == GANTRY_Z_SERIAL) {
        cout << "check serial: " << theNode.Info.SerialNumber.Value() << ", "
             << GANTRY_Z_SERIAL << endl;
        ros::param::set("/GANTRY_Z_NODE", _node_count);
      }

      if (theNode.Info.SerialNumber.Value() == GANTRY_YAW_SERIAL) {
        cout << "check serial: " << theNode.Info.SerialNumber.Value() << ", "
             << GANTRY_YAW_SERIAL << endl;
        ros::param::set("/GANTRY_YAW_NODE", _node_count);
      }

      theNode.Status.AlertsClear();   // Clear Alerts on node
      theNode.Motion.NodeStopClear(); // Clear Nodestops on Node

      theNode.AccUnit(INode::RPM_PER_SEC); // units for Accel RPM/SEC
      theNode.VelUnit(INode::RPM);         // units for Velocity to RPM
      theNode.Motion.JrkLimit =
          JERK_LIMIT_44ms; // 3 = 10 ms, 5 = 25ms, 6 = 44 ms

      if (theNode.Info.SerialNumber.Value() == GANTRY_Z_SERIAL) {
        theNode.EnableReq(true);
      }

      else {
      theNode.EnableReq(
          false); // Note: Set this to false because it might intefere
                  // with the homing fucntion For homing, currently motor
                  // X1 is not used because two motors can't be synced and
                  // homed at the same time
                  // except for the Z motor where the torque must on all the time...
      }

      myMgr.Delay(100);
      ROS_WARN("Node %d enabled......", _node_count);
      _node_count++;
      cout << "-----------------------------------" << endl;
    }
    init_done = true;

    ROS_INFO("Port(s) opened:.. ..Init() complete...");
    //}

    return 1;
  }

  void calib(const std_msgs::StringConstPtr& msg) {
    /*
     * This function is supposed to calibrate the YAW-Z-Y-X in sequence and
     * reset the joint states to 0,0,0.
     */
    std::string data = msg->data;

    bool val_yaw = false;
    bool val_x = false;
    bool val_y = false;
    bool val_z = false;

    val_yaw = calib_axisN("YAW");

    if (!val_yaw) {
      ROS_ERROR("Calibration of YAW axis failed...");
    } else {
      val_z = calib_axisN("Z");
      if (!val_z) {
        ROS_ERROR("Calibration of Z axis failed...");
      } else {
        val_y = calib_axisN("Y");
        if (!val_y) {
          ROS_ERROR("Calibration of Y axis failed...");
        } else {
          val_x = calib_axisN("X");
          if (!val_x) {
            ROS_ERROR("Calibration of X axis failed...");
          }
        }
      }
    }

    if (!val_yaw || !val_x || !val_y || !val_z) {
      ROS_ERROR("Calibration failed.....Manual intervention required....");
    } else {
      ROS_INFO("Home action complete in all axis... enabling the X1 motor... ");

      IPort& myPort =
          myMgr.Ports(0); // 0 is always the serial ... as set in init()
      // for some reason it is not possible to make the IPort
      // public or initialize in the constructor, depricated thought :-),
      // 4/13/2022 Note: In the future, set ros param for this

      int node_x1 = 0; // usually mounts to 0

      ros::param::get("/GANTRY_X1_NODE", node_x1);

      // cout << "/GANTRY_X1_NODE: " << node_x1 << endl;
      // INode &theNode = myPort.Nodes(static_cast<unsigned int>(node_x1));
      INode& theNode = myPort.Nodes(static_cast<unsigned int>(node_x1));

      theNode.Status.AlertsClear();   // Clear Alerts on node
      theNode.Motion.NodeStopClear(); // Clear Nodestops on Node
      theNode.EnableReq(true);        // Enable node true
      sleep(1);
    }
  }

  bool calib_axisN(std::string data) {

    int node_no = 0;
    size_t portNo =
        0; // set port no. for the right SC HUB, not necessary 4/13/2022

    std::string ros_param_name("NOT_SET");

    if (!strcmp(data.c_str(), "X")) {
      ros::param::set("/GANTRY_CALIBRATED_X", false);
      ros::param::get("/GANTRY_X2_NODE", node_no);
      // portNo = 0;
    }

    if (!strcmp(data.c_str(), "Y")) {
      ros::param::set("/GANTRY_CALIBRATED_Y", false);
      ros::param::get("/GANTRY_Y_NODE", node_no);
      // portNo = 0;
    }

    if (!strcmp(data.c_str(), "Z")) {
      ros::param::set("/GANTRY_CALIBRATED_Z", false);
      ros::param::get("/GANTRY_Z_NODE", node_no);
      // portNo = 0;
    }

    if (!strcmp(data.c_str(), "YAW")) {
      ros::param::set("/GANTRY_CALIBRATED_YAW", false);
      ros::param::get("/GANTRY_YAW_NODE", node_no);
      // node_no = 0; // reset because there is only one node, depricated
      // 4/13/2022 portNo = 0; // previously 1
    }

    ROS_WARN("Calibrating the %s Axis....", data.c_str());

    // cout << "node_no: " << node_no << endl;
    IPort& myPort =
        myMgr.Ports(portNo); // for some reason it is not possible to make the
                             // IPort public or initialize in the constructor
                             // Note: In the future, set ros param for this

    INode& theNode = myPort.Nodes(static_cast<unsigned int>(node_no));

    theNode.Status.AlertsClear();   // Clear Alerts on node
    theNode.Motion.NodeStopClear(); // Clear Nodestops on Node
    theNode.EnableReq(true);        // Enable node

    sleep(1); // this is required for the enabling function to work properly
              // 1 sec is okay but 2 is just to be sure

    double timeout = myMgr.TimeStampMsec() + TIME_TILL_TIMEOUT;

    if (theNode.Motion.Homing.HomingValid()) {
      if (theNode.Motion.Homing.WasHomed()) {
        ROS_WARN("Node %d has already been homed, current position is: \t%8.0f",
                 node_no, theNode.Motion.PosnMeasured.Value());
        ROS_INFO("Rehoming Node...");
      } else {
        ROS_INFO("Node [%d] has not been homed.  Homing Node now...", node_no);
      }

      theNode.Motion.Homing.Initiate(); // Now we will home the Node

      timeout = myMgr.TimeStampMsec() +
                TIME_TILL_TIMEOUT; // define a timeout in case the node is
                                   // unable to enable
                                   // Basic mode - Poll until disabled

      current_pos_motor_ = theNode.Motion.PosnMeasured.ValueDouble::Value(true);

      while (!theNode.Motion.Homing.WasHomed()) {
        if (myMgr.TimeStampMsec() > timeout) {
          ROS_WARN(
              "Node did not complete homing:  \n\t -Ensure Homing settings "
              "have been defined through ClearView. \n\t -Check for "
              "alerts/Shutdowns \n\t -Ensure timeout is longer than the "
              "longest possible homing move.");
        }
      }

      theNode.Motion.Homing
          .SignalComplete(); // reset the Node's "sense of home" soft limits
                             // (unchanged) are now active again
      sleep(1);

      theNode.Motion.PosnMeasured.Refresh();

      //--- making sure it is absolute zero even after homing

      if (!strcmp(data.c_str(), "Y")) {
        theNode.Motion.AddToPosition(
            80000); // because we changed the origin to center
      }

      else {
        double posn = theNode.Motion.PosnMeasured.Value();
        theNode.Motion.AddToPosition(-posn);
      }

      // theNode.EnableReq(false); // comment this line enable torque
      ROS_INFO("Calibration of %s axis .... done", data.c_str());
      ros::param::set(std::string("/GANTRY_CALIBRATED_" + data), true);

      // theNode.EnableReq(false); // delete this later
      sleep(1);

      return true;
    }

    else {
      ROS_WARN("Node[%d] has not had homing setup through ClearView.  The node "
               "will not be homed..",
               node_no);
      sleep(1);
      return false;
    }

    // return false;
  }

  void preemptCB() {
    ROS_INFO("%s:preempted:", action_name_.c_str());
    as_.setPreempted();
  }

  void goalCB() {
    goal_ = as_.acceptNewGoal();
    move();
  }

  void move() {

    if (!as_.isActive()) {
      ROS_WARN_ONCE(
          "\rGOAL NOT RECEIVED YET !!!! ... GET out of this function ...");
      return;
    }

    if (!init_done) {
      ROS_WARN("init() not completed... re-routing to init()...");
      init();
    }

    ROS_INFO("GOAL RECEIVED .................");

    size_t traj_count = goal_->trajectory.points.size();
    size_t joint_count = goal_->trajectory.joint_names.size();

    // cout << "joint_count: " << joint_count << endl;

    j_pos.clear();
    j_vel.clear();
    j_acc.clear();

    std::vector<vector<double>> vel_;
    std::vector<vector<double>> acc_;
    std::vector<vector<double>> pos_;

    for (size_t k = 0; k < joint_count; k++) {
      j_pos.clear();
      j_vel.clear();
      j_acc.clear();

      if (!std::strcmp(goal_->trajectory.joint_names[k].c_str(), "gantryX") ||
          !std::strcmp(goal_->trajectory.joint_names[k].c_str(), "gantryY") ||
          !std::strcmp(goal_->trajectory.joint_names[k].c_str(), "gantryZ") ||
          !std::strcmp(goal_->trajectory.joint_names[k].c_str(), "gantryYAW")) {

      for (size_t i = 0; i < traj_count; i++) {
        for (size_t j = 0; j < joint_count; j++) {
          if (k == j) {
            /*
            cout.precision(6);
            cout << "Joint_name: [" << j << "] "
                 << goal_->trajectory.joint_names[j]
                 << " \tPos: " << goal_->trajectory.points[i].positions[j]
                 << std::fixed
                 << " \tVel: " << goal_->trajectory.points[i].velocities[j]
                 << std::fixed
                 << " \tAcc: " << goal_->trajectory.points[i].accelerations[j]
                 << std::fixed << endl;
            */

            j_pos.push_back(goal_->trajectory.points[i].positions[j]);
            j_vel.push_back(goal_->trajectory.points[i].velocities[j]);
            j_acc.push_back(goal_->trajectory.points[i].accelerations[j]);
          }
        }
      }
      vel_.push_back(j_vel);
      pos_.push_back(j_pos);
      acc_.push_back(j_acc);
    }
      else
        cout<<"Skipping joint... "<<goal_->trajectory.joint_names[k].c_str()<<endl;
    }

    vector<double> vel_limit; // in m/s
    vector<double> acc_limit; // in m/s
    vector<double> pos_start; // in meters
    vector<double> pos_end;   // in meters

    //=========== computer median velocity and max/ min acceleration =========

    get_mean_vel_max_acc(pos_, vel_, acc_, vel_limit, acc_limit, pos_start,
                         pos_end);

    // for (size_t p = 0; p<vel_limit.size(); p++) {
    //  cout<<"vel["<<p<<"]: "<<vel_limit[p]<<endl;
    //  cout<<"acc["<<p<<"]: "<<acc_limit[p]<<endl;
    //  cout<<"p_s["<<p<<"]: "<<pos_start[p]<<endl;
    //  cout<<"p_e["<<p<<"]: "<<pos_end[p]<<endl;
    //  cout<<"---------------------------------"<<endl;
    //}

    //================ Code to move the servo ==================
    vector<Axis*> listOfAxes;

    // NOW as in the init(), calib(), and others
    // need to change the manual port to serial or USB

    // for (size_t portno = 0; portno < 2; portno++) {
    size_t portno = 0;

    IPort& myPort =
        myMgr.Ports(portno); // now chaning to different sc hub for now

    double _vel_limit = 0;
    double _acc_limit = 0;
    float _distance_travel = 0.0f;
    double _vel_limit_x1_bak = 0.0;
    double _acc_limit_x1_bak = 0.0;
    float _distance_x1_bak = 0.0f;

    //======== REMINDER THAT THERE ARE 4 ACTUATORS BUT 3 JOINTS ===========

    unsigned _joint_count = 0;

    for (unsigned iNode = 0; iNode < myPort.NodeCount(); iNode++) {

      INode& theNode = myPort.Nodes(iNode);

      _distance_travel =
          static_cast<float>((pos_end[_joint_count] - pos_start[_joint_count]));

      if (nodeTypesGood && accessLvlsGood) {
        // ROS_INFO("Create an axis for node: %d, %d", iNode,
        //         theNode.Info.SerialNumber.Value());

        listOfAxes.push_back(new Axis(&theNode));

        // Set the move distance based on where it is in the network

        // ============================XXXXXXX=======================
        if (theNode.Info.SerialNumber.Value() == GANTRY_X2_SERIAL) {
          // cout << "[" << iNode << "] move gantryX2 by: " << _distance_travel
          //     << " meters" << endl;

          listOfAxes.at(iNode)->SetMoveRevs_meters(_distance_travel); // X1

          _vel_limit = 0;
          _acc_limit = 0;

          _vel_limit =
              floor(fabs(vel_limit[_joint_count] *
                         MPS_TO_RPM_XY)); // RPM = 2*(60*v)/(2*pi*r) which is
                                          // nearly equal to 1500
          _acc_limit = floor(fabs(acc_limit[_joint_count] * MPS_TO_RPM_XY));

          if (_vel_limit >= VEL_LIMIT_SMALL)
            _vel_limit = VEL_LIMIT_SMALL;

          if (_acc_limit >= ACC_LIMIT_SMALL)
            _acc_limit = ACC_LIMIT_SMALL;

          if (_vel_limit <= 1)
            _vel_limit = 5;
          if (_acc_limit <= 1)
            _acc_limit = 5;

          // cout<<"velocity limit: "<< _vel_limit<<endl;
          // cout<<"acceleration limit: "<< _acc_limit<<endl;

          cout << "vel limit: " << _vel_limit << " acc limit: " << _acc_limit
               << endl;

          theNode.Motion.VelLimit = _vel_limit;
          theNode.Motion.AccLimit = _acc_limit;
          theNode.Motion.Adv.TriggerGroup(1);

          _vel_limit_x1_bak = _vel_limit;
          _acc_limit_x1_bak = _acc_limit;
          _distance_x1_bak = _distance_travel;

          _joint_count++;
        }
        /* Note: X1 & X2 are synced but move in opposite direction */
        if (theNode.Info.SerialNumber.Value() == GANTRY_X1_SERIAL) {
          // cout << "[" << iNode
          //     << "] move gantryX1 by: " << -1.0f * _distance_x1_bak
          //     << " meters" << endl;
          theNode.Motion.VelLimit =
              _vel_limit_x1_bak; // over write the velocity limit
          theNode.Motion.AccLimit = _acc_limit_x1_bak;

          listOfAxes.at(iNode)->SetMoveRevs_meters(-1.0f *
                                                   _distance_x1_bak); // X2
          theNode.Motion.Adv.TriggerGroup(1);

          // cout << "vel limit: " << _vel_limit_x1_bak
          //     << " acc limit: " << _vel_limit_x1_bak << endl;
        }
        //=====================YYYYYYYYYYYYYY=======================
        if (theNode.Info.SerialNumber.Value() == GANTRY_Y_SERIAL) {
          // cout << "[" << iNode
          //     << "] move gantryY by: " << -1.0f * _distance_travel << "
          //     meters"
          //     << endl;

          _vel_limit = 0;
          _acc_limit = 0;

          _vel_limit =
              floor(fabs(vel_limit[_joint_count] *
                         MPS_TO_RPM_XY)); // RPM = 2*(60*v)/(2*pi*r) which is
                                          // nearly equal to 1500
          _acc_limit = floor(fabs(acc_limit[_joint_count] * MPS_TO_RPM_XY));

          if (_vel_limit >= VEL_LIMIT_LARGE)
            _vel_limit = VEL_LIMIT_LARGE;

          if (_acc_limit >= ACC_LIMIT_LARGE)
            _acc_limit = ACC_LIMIT_LARGE;

          if (_vel_limit <= 1) // Note: very very important thing, acc and vel
                               // should not be zero
            _vel_limit = 5;    // else the motor/ thread will take forever to
                               // exit = shit hitting the fan
          if (_acc_limit <= 1)
            _acc_limit = 5;

          // cout << "velocity limit: " << _vel_limit << endl;
          // cout << "acceleration limit: " << _acc_limit << endl;

          theNode.Motion.VelLimit = _vel_limit;
          theNode.Motion.AccLimit = _acc_limit;

          listOfAxes.at(iNode)->SetMoveRevs_meters(
              1.0f * _distance_travel);       // Y previously -1
          theNode.Motion.Adv.TriggerGroup(1); // 2
          _joint_count++;
        }
        //=====================ZZZZZZZZZZZZZZ=======================
        if (theNode.Info.SerialNumber.Value() == GANTRY_Z_SERIAL) {
          // cout << "[" << iNode
          //     << "] move gantryZ by: " << -1.0f * _distance_travel<< "
          //     meters" << endl;

          _vel_limit =
              floor(fabs(vel_limit[_joint_count] *
                         MPS_TO_RPM_Z)); // RPM = 2*(60*v)/(2*pi*r) which is
                                         // nearly equal to 1500
          _acc_limit = floor(fabs(acc_limit[_joint_count] * MPS_TO_RPM_Z));

          if (_vel_limit >= VEL_LIMIT_SMALL)
            _vel_limit = VEL_LIMIT_SMALL;

          if (_acc_limit >= ACC_LIMIT_SMALL)
            _acc_limit = ACC_LIMIT_SMALL;

          if (_vel_limit <= 1)
            _vel_limit = 5;
          if (_acc_limit <= 1)
            _acc_limit = 5;

          // cout << "vel limit: " << _vel_limit << " acc limit: " << _acc_limit
          //     << endl;

          theNode.Motion.VelLimit = _vel_limit;
          theNode.Motion.AccLimit = _acc_limit;
          listOfAxes.at(iNode)->SetMoveRevs_meters(-1.0f * _distance_travel, 1);
          theNode.Motion.Adv.TriggerGroup(1); // 2
          _joint_count++;
        }
        //=====================YAWWWWWWWWWWWW=======================
        if (theNode.Info.SerialNumber.Value() == GANTRY_YAW_SERIAL) {

          cout << "[" << iNode << "] move gantryYAW by: " << _distance_travel
               << " radians..." << fabs(vel_limit[_joint_count]) << " rad/sec"
               << endl;

          cout << "[" << iNode << "] move gantryYAW by: "
               << _distance_travel * float(180 / (3.141592654)) << " degrees..."
               << endl;
          cout << "[" << iNode << "] move gantryYAW by: "
               << _distance_travel * float(RADIAN_TO_COUNTS_YAW) << " steps..."
               << endl;
          // YAW degrees to be compensated for the gear ratio of 5
          _vel_limit = floor(
              fabs(vel_limit[_joint_count] * RADIANS_PER_SEC_TO_RPM_YAW * 20));
          _acc_limit = floor(
              fabs(acc_limit[_joint_count] * RADIANS_PER_SEC_TO_RPM_YAW * 20));

          if (_vel_limit >= VEL_LIMIT_SMALL)
            _vel_limit = VEL_LIMIT_SMALL;

          if (_acc_limit >= ACC_LIMIT_SMALL)
            _acc_limit = ACC_LIMIT_SMALL;

          if (_vel_limit <= 1)
            _vel_limit = 5;
          if (_acc_limit <= 1)
            _acc_limit = 5;

          // cout << "velocity limit: " << _vel_limit << endl;
          // cout << "acceleration limit: " << _acc_limit << endl;

          cout << "vel limit: " << _vel_limit << " acc limit: " << _acc_limit
               << endl;

          theNode.Motion.VelLimit =
              _vel_limit; // this needs to be converted to RPM
          theNode.Motion.AccLimit =
              _acc_limit; // this needs to be converted to RPM/s

          listOfAxes.at(iNode)->SetMoveRevs_meters(1 * _distance_travel, 2); // 7/8/24 was -1 * _distance_travel
          theNode.Motion.Adv.TriggerGroup(1);
          _joint_count++;
        }
        // cout<<"++++++++++++++++++++++++++++++++++"<<endl;
      }
    }

    //============ now create thred to move in sync ===========
    if (nodeTypesGood && accessLvlsGood) {

      ROS_INFO("Creating supervisor thread...");

      Supervisor theSuper(listOfAxes, myMgr);

      // cout << "axis list size: " << listOfAxes.size() << endl;
      theSuper.CreateThread();

      // usleep(200000);

      ros::param::set("/move_done", false);
      bool done = false;

      ros::Rate r(3);
      while (ros::ok()) {

        ros::param::get("/move_done", done);

        if (done == true) {
          break;
        }

        cout << "\rWaiting for move to complete ....... |";
        cout << "\rWaiting for move to complete ....... /";
        cout << "\rWaiting for move to complete ....... -";
        cout << "\rWaiting for move to complete ....... \\";

        r.sleep();
      }

      theSuper.Quit();
      theSuper.Terminate();

      result_.result.error_code = result_.result.SUCCESSFUL;
      as_.setSucceeded(result_.result);
    }

    else {
      if (!nodeTypesGood) {
        ROS_WARN("FAILURE: Please attach only ClearPath-SC Advanced nodes.");
      } else if (!accessLvlsGood) {
        ROS_WARN("FAILURE: Please get full access on all your nodes.");
      }
    }

    // Delete the list of axes that were created
    for (size_t iAxis = 0; iAxis < listOfAxes.size(); iAxis++) {
      delete listOfAxes.at(iAxis);
    }

    // Close down the ports
    // myMgr.PortsClose();

    torqueON();

    cout << "=============================================" << endl;
  }


  //  msg is either True or False to enable or disable torque

  void torqueON() {

    //std::string data = msg->data;
    size_t portno = 0;
    //cout << "Received msg: " << data << endl;

    IPort& myPort = myMgr.Ports(portno);

    for (size_t i = 0; i < myPort.NodeCount(); i++) {

      INode& theNode = myPort.Nodes(i);
      theNode.Status.AlertsClear();
      theNode.EnableReq(true); // Ensure Node is disabled before loading
                                       // config file We don't have to
      myMgr.Delay(200);

    }
  }

  int get_mean_vel_max_acc(vdd pos_, vdd vel_, vdd acc_, vd& v_lim, vd& a_lim,
                           vd& p_start, vd& p_end) {
    /*
     * Here the input is [X]{v1, v2, .... vn}[Y]{v1, v2, .... vn} [Z]{v1, v2,
     * .... vn} {a1, a2, .... an}   {a1, a2, .... an}    {a1, a2, .... an}
     * Output: [v_mean a_min a_max
     *          v_mean a_min a_max]
     */

    if (vel_.size() != acc_.size() || vel_.size() != pos_.size() ||
        acc_.size() != pos_.size())
      return -1;

    for (size_t i = 0; i < vel_.size(); i++) {

      if (vel_.size() != acc_.size() || vel_.size() != pos_.size() ||
          acc_.size() != pos_.size())
        return -1;

      std::sort(acc_[i].begin(), acc_[i].end());

      double max_acc = *max_element(acc_[i].begin(), acc_[i].end());
      // double min_acc = *min_element(acc_[i].begin(), acc_[i].end());

      if (fabs(max_acc) <= 0.001)
        max_acc = 0.1;

      // ========== compute the median of the traj vel ===========
      // Here the velocity is increasing so the center value will suffice
      // the accleration seems like is max at end and decelleration is equal to
      // the peak acceleration

      size_t mean_vel_index = static_cast<size_t>(vel_[i].size() / 2);
      // the mean velocity can be replaced with buffer drop velocity profile.
      // 4/19/2022

      p_start.push_back(pos_[i][0]);
      p_end.push_back(pos_[i][pos_[i].size() - 1]);
      v_lim.push_back(vel_[i][mean_vel_index]);
      a_lim.push_back(max_acc);
    }
    return 0;
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
  tf::TransformBroadcaster broadcaster;

  ros::Subscriber sub_, sub__, sub___, sub_torque;
  double velocity_, acceleration_, position_; // goal_;
  double velocity_pub, acceleration_pub, position_pub;
  double velocity__, acceleration__; // based on return type
  double start_pos_, end_pos_, calib_pose;
  double current_pos_, current_vel_, current_pos_motor_;

  actionlib::SimpleActionServer<
      control_msgs::FollowJointTrajectoryAction>::GoalConstPtr goal_;

  control_msgs::FollowJointTrajectoryActionFeedback feedback_;
  control_msgs::FollowJointTrajectoryActionResult result_;
  std::string joint_name_;
  std::thread* rt_publish_thread_;
  ros::Publisher joint_pub;

  std::vector<double> j_vel;
  std::vector<double> j_pos;
  std::vector<double> j_acc;

  std::vector<string> joint_names;

  SysManager myMgr;

  std::vector<std::string> comHubPorts;
  size_t portCount;
  std_msgs::String cameraTrigGommand;
  ros::Publisher SensorTrigger, pub_;
  bool init_done;
  Uint32 GANTRY_X1_SERIAL;
  Uint32 GANTRY_X2_SERIAL;
  Uint32 GANTRY_Y_SERIAL;
  Uint32 GANTRY_Z_SERIAL;
  Uint32 GANTRY_YAW_SERIAL;

  bool nodeTypesGood, accessLvlsGood;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "gantry_driver");
  multi_servo_driver servo(ros::this_node::getName());

  // Note: it is the async spinner that makes it possible
  // to get joint state when the motor is moving.
  // with just the regular spin(), no joint state when the robot is moving

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
