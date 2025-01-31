#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>
//#include <dynamixel_sdk/dynamixel_sdk.h>
#include <fcntl.h>
#include <termios.h>
#include <string>

#include "gantry_move/gantry_moveAction.h"

/*
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
//#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define EXT_POSITION_CONTROL_MODE       4                   // Value for extended position control mode (operating mode)
#define DXL_OPEN_LOCATION               1000//2048                // this i soprn position
#define DXL_CLOSE_LOCATION              -5600//-5000

// ===================== MX-64AT & MX-106R =================
#define ADDR_OPERATING_MODE             11
#define ADDR_TORQUE_ENABLE              64
#define ADDR_DIRECTION_CHANGE           10
#define ADDR_PRESENT_POSITION           132
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_CURRENT            126

#define ESC_ASCII_VALUE                 0x1b

int dxl_goal_position[2] = { DXL_CLOSE_LOCATION, DXL_OPEN_LOCATION };
std::string DEVICENAME = "/dev/ttyUSB2";
*/
bool cut_inst = true;
using namespace std;

class dynamixel_motor {

  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<gantry_move::gantry_moveAction> as_;
    std::string action_name_;
    gantry_move::gantry_moveFeedback feedback_;
    gantry_move::gantry_moveResult results_;

    //dynamixel::PortHandler* portHandler;
    //dynamixel::PacketHandler* packetHandler;
    int dxl_comm_result;             // Communication result
    uint8_t dxl_error;                          // Dynamixel error
    uint16_t dxl_model_number;
    int32_t dxl_present_position;
    int16_t dxl_present_current;

  public:

    dynamixel_motor(std::string name): as_(nh_, name, boost::bind(&dynamixel_motor::executeCB, this,
                                                                      _1), false), action_name_(name) {
        as_.start();
        ROS_INFO("Action server initialized....");
        //dxl_comm_result = COMM_TX_FAIL;             // Communication result
        //dxl_error = 0;                          // Dynamixel error
        //dxl_model_number = 0;
        //dxl_present_position = 0;
        //dxl_present_current = 0;

        //ros::param::get("/udev_id/dynamixel", DEVICENAME);

        //cout << "DEVICENAME: " << DEVICENAME << endl;

        ros::param::get("/cut_inst", cut_inst);
        //cout << "cut_inst: " << cut_inst << endl;
        //dynamixel_init();
    }

    void executeCB(const gantry_move::gantry_moveGoalConstPtr& goal) {

        bool success = true;

        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_WARN("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
            //break;
        }

        cout << "goal received: " << goal->cut_command << endl;
        int res = dynamixel_init();

        //sleep(2);

        feedback_.cut_feedback = res;
        as_.publishFeedback(feedback_);

        if (res == 1) {
            results_.cut_result = feedback_.cut_feedback;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(results_);
        }
    }

    int dynamixel_init() {


        //remove this for actual cut

        int ret_val = -1;


        /*
        if (cut_inst == true) {

            portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME.c_str());
            packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

            if (portHandler->openPort()) {
                cout << "Succeeded to open the port!" << endl;
            }

            else {
                cout << "Failed to open the port!\n" << endl;
                cout << "Press any key to terminate...\n" << endl;
                cin.get();
            }

            dxl_comm_result = packetHandler->ping(portHandler, DXL_ID, &dxl_model_number, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
            }

            else if (dxl_error != 0) {
                cout << packetHandler->getRxPacketError(dxl_error) << endl;
            }

            else {
                cout << "[ID: " << 1 << "]. ping Succeeded. Dynamixel model number: " << dxl_model_number << endl;
            }

            // Set operating mode to extended position control mode
            //cout<<"Set operating mode to extended position control mode "<<endl;

            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE,
                                                            EXT_POSITION_CONTROL_MODE, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                cout << "SUCCESS! " << packetHandler->getTxRxResult(dxl_comm_result) << endl;
            }

            else if (dxl_error != 0) {
                cout << "ERROR!!! " << packetHandler->getRxPacketError(dxl_error) << endl;
            } else {
                cout << "Operating mode changed to extended position control mode. \n" << endl;
            }

            // Enable Dynamixel Torque
            //cout<<"Enable Dynamixel Torque...."<<endl;
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                            TORQUE_ENABLE, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;;
            } else if (dxl_error != 0) {
                cout << "ERROR!!!" << packetHandler->getRxPacketError(dxl_error) << endl;
            } else {
                cout << "Torque enabled........ \n" << endl;
            }

            int index = 0;

            for (int i = 0; i < 2; i++) {

                //printf("Press any key to continue! (or press ESC to quit!)\n");

                //char str[100];
                //cin>>str;
                //string s = str;

                //if (s.compare("s") == 0)
                //  break;

                // Write goal position
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,
                                                                dxl_goal_position[index], &dxl_error);

                if (dxl_comm_result != COMM_SUCCESS) {
                    cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;;
                } else if (dxl_error != 0) {
                    cout << "ERROR!!!" << packetHandler->getRxPacketError(dxl_error) << endl;
                } else {
                    cout << "couldn't write goal position values........ \n" << endl;
                }

                do {
                    // Read present position
                    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION,
                                                                   (uint32_t*)&dxl_present_position, &dxl_error );

                    if (dxl_comm_result != COMM_SUCCESS) {
                        cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;;
                    } else if (dxl_error != 0) {
                        cout << "ERROR!!!" << packetHandler->getRxPacketError(dxl_error) << endl;
                    }

                    // Read present current
                    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT,
                                                                   (uint16_t*)&dxl_present_current, &dxl_error );

                    if (dxl_comm_result != COMM_SUCCESS) {
                        cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;;
                    } else if (dxl_error != 0) {
                        cout << "ERROR!!!" << packetHandler->getRxPacketError(dxl_error) << endl;
                    }

                    //cout<<" Present Current:"<< dxl_present_current<<endl;

                    //cout<<"[ID:"<<DXL_ID  <<"] GoalPos: "<< dxl_goal_position[index]<<" PresPos:"<< dxl_present_position<<endl;

                } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

                // Change goal position
                if (index == 0) {
                    index = 1;
                } else {
                    index = 0;
                }
            }

            // Diable Dynamixel Torque
            //cout<<"Diabling Dynamixel Torque...."<<endl;
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                            TORQUE_DISABLE, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;;
            } else if (dxl_error != 0) {
                cout << "ERROR!!!" << packetHandler->getRxPacketError(dxl_error) << endl;
            } else {
                cout << "Torque disabled........ \n" << endl;
            }

            // Close port
            portHandler->closePort();
            ret_val = 1;
        }

        else {

            ROS_WARN_ONCE("I'll cut once the motor is enabled .......");
            sleep(1);
            ret_val = 1;
        }

        */
        return ret_val; // in the future return 1 or 0 based of peak detection
    }

};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "gantry_move_server");
    //dynamixel_motor dm;
    dynamixel_motor cut_action("gantry_move_server");
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    ros::spin();
    return 0;
}
