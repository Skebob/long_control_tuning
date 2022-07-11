#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "automotive_platform_msgs/SpeedMode.h"

#include "pacmod_msgs/PositionWithSpeed.h"
#include "pacmod_msgs/PacmodCmd.h"


#include "sensor_msgs/Imu.h"

int enabled_;

int speed;

ros::Publisher throttlePub;
ros::Publisher brakePub;
ros::Publisher gearPub;
ros::Publisher enablePub;
ros::Subscriber curAccelSub;
ros::Publisher desAccelPub;

// this may need to be tuned, I just guessed on the value
// IF (-CONSTANT_ACCEL_BOUNDRY < [measured acceleration] < CONSTANT_ACCEL_BOUNDRY) THEN car is at zero acceleration (constant velocity)
#define CONSTANT_ACCEL_BOUNDRY 0.005 // [-0.005, 0.005]

float accel_cur;
float accel_des = 0;

void curAccelCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	accel_cur = (msg->linear_acceleration).x;	
}

int isConstantVelocity(){
    return ((accel_cur < CONSTANT_ACCEL_BOUNDRY) && (accel_cur > -1*CONSTANT_ACCEL_BOUNDRY));
}

void publishThrottleCmd(float value){
    pacmod_msgs::PacmodCmd throttleCmd;
    throttleCmd.header.stamp = ros::Time::now();
    throttleCmd.header.frame_id ="accel_tuning";
    throttleCmd.enable = true;

    throttleCmd.f64_cmd = value;

    throttlePub.publish(throttleCmd);
    ROS_INFO("DEBUG: %f", value);
}

// gives enable message to pacmod
// lifts break of pacmod
// set gear to forward in pacmod
void CarInit(void){
    std_msgs::Bool enableCmd;
    enableCmd.data = true;

    enablePub.publish(enableCmd);

    pacmod_msgs::PacmodCmd brakeCmd;
    brakeCmd.enable = false;
    brakeCmd.f64_cmd = 0;
    brakeCmd.ui16_cmd = 0;

    brakePub.publish(brakeCmd);

    pacmod_msgs::PacmodCmd shiftCmd;
    shiftCmd.enable = true;
    shiftCmd.f64_cmd = 0;
    shiftCmd.ui16_cmd = 3;

    gearPub.publish(shiftCmd);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "longitudinal_tuning");
    ros::NodeHandle n;

    throttlePub = n.advertise<pacmod_msgs::PacmodCmd>("as_rx/accel_cmd", 1);
    brakePub = n.advertise<pacmod_msgs::PacmodCmd>("as_rx/brake_cmd", 1);
    gearPub = n.advertise<pacmod_msgs::PacmodCmd>("as_rx/shift_cmd", 1);
    enablePub = n.advertise<std_msgs::Bool>("as_rx/enable", 1);

    desAccelPub = n.advertise<std_msgs::Float64>("desAccel", 1);

    curAccelSub = n.subscribe<sensor_msgs::Imu>("imu/data", 1, curAccelCallback);

    // this is the throttle position that will be held until the car reaches constant velocity
    float ped_val = 0.4;
    // this is the acceleration command passed to the low level longitude controller after the car reaches constant velocity
    float accel_val = 0.3;

    int forked = 0;

    std_msgs::Float64 desaccel_msg;
    ros::Time startTime = ros::Time::now();
    ros::Rate loop_rate(30);

    CarInit();

    while(ros::ok()){
        if(forked == 0){
            // get current time
            ros::Time now = ros::Time::now();
            float time = now.toSec() - startTime.toSec();

            // check if the car is at a constant velocity after 5 seconds
            if(isConstantVelocity() && (time > 5)){
                // car has reached a constant velocity
                forked = 1;
                // pass off control to longitude controller
                // roslaunch echo testecho.launch
                if(fork() == 0){
                    // CHILD
                    // launch low level longitude  controller
                    execlp("roslaunch", "roslaunch", "echo", "testecho.launch", NULL);
                    ROS_INFO("Error: control should not reach here");
                } else {
                    // PARENT
                    //ros::Duration(0.5).sleep(); // sleep for half a second
                    desaccel_msg.data = accel_val;
                } 
            } else {
                // car has not reached a constant velocity 
                publishThrottleCmd(ped_val);
            }
        } else {
            desAccelPub.publish(desaccel_msg);
        }
        
        ros::spinOnce();
    }

    return 0;
}