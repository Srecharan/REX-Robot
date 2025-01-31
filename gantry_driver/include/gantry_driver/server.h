#ifndef SERVER_H
#define SERVER_H
#include <iostream>

using namespace std;

typedef vector<vector<double>> vdd;
typedef vector<double> vd;

#define MOVE_DISTANCE_CNTS -144000 // this is equivalent to 0.225 m
#define CHANGE_NUMBER_SPACE_XY 250 // The change to the numberspace after homing (cnts)
#define CHANGE_NUMBER_SPACE_Z 1600
#define TIME_TILL_TIMEOUT 100000 // The timeout used for homing(ms)
#define NUM_OF_MOTORS 5 // 4 id 3 axis, 5 for yaw control
#define JERK_LIMIT_10ms 3   //// 3 = 10 ms, 5 = 25ms, 6 = 44 ms
#define JERK_LIMIT_25ms 5
#define JERK_LIMIT_44ms 6

#define VEL_LIMIT_LARGE 1800 //RPM   limit
#define ACC_LIMIT_LARGE 4000 //RPM/s limit

#define VEL_LIMIT_SMALL 2300 //RPM   limit
#define ACC_LIMIT_SMALL 4000 //RPM/s limit

#define MPS_TO_RPM_Z  4724.4094488189
#define MPS_TO_RPM_XY 1499.9287812006

#define RPM_YAW_VEL 1500
#define RPM_YAW_ACC 500

/*
#define COUNTS_TO_METERS_XY 160000
#define METERS_TO_COUNTS_XY 0.00000625
#define COUNTS_TO_METERS_Z  503930.6591413022 //320000
#define METERS_TO_COUNTS_Z  0.0000019844 //0.000003125
#define RADIAN_TO_COUNTS_YAW 5092.958178941  // (6400 * 5)/(2*pi)
#define RADIAN_TO_COUNTS_YAW_DEG 291805.008889953
#define METERS_TO_COUNTS_YAW 0.00019635 // (1/5092.958178941) rad
#define METERS_TO_COUNTS_YAW_DEG 0.01125 // (1/5092.958178941) * (180/pi) degrees
#define RADIANS_PER_SEC_TO_RPM_YAW 9.549297 // multiply this by the gear ration of 5
#define RPM_TO_RADIANS_PER_SEC 0.10472 // multiply this by the gear ration of 5
*/

#define COUNTS_TO_METERS_XY 160000
#define METERS_TO_COUNTS_XY 0.00000625
#define COUNTS_TO_METERS_Z  503930.6591413022 //320000
#define METERS_TO_COUNTS_Z  0.0000019844 //0.000003125
#define RADIAN_TO_COUNTS_YAW 20371.832715763  // (6400 * 20)/(2*pi)
#define METERS_TO_COUNTS_YAW 0.000049087 // (1/20371.83271576) rad
#define RADIANS_PER_SEC_TO_RPM_YAW 9.549297 // multiply this by the gear ration of 5
#define RPM_TO_RADIANS_PER_SEC 0.10472 // multiply this by the gear ration of 5

#endif // SERVER_H
