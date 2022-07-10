#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "automotive_platform_msgs/SpeedMode.h"

#include "pacmod_msgs/PositionWithSpeed.h"
#include "pacmod_msgs/PacmodCmd.h"
#include "std_msgs/Bool.h"

#include "sensor_msgs/Imu.h"

int enabled_;


ros::Publisher throttlePub;
ros::Subscriber curAccelSub;
ros::Publisher desAccelPub;

// this may need to be tuned, I just guessed on the value
#define CONSTANT__ACCEL_BOUNDRY 0.005

float accel_cur;
float accel_des = 0;

void curAccelCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	accel_cur = (msg->linear_acceleration).x;	
}

int isConstantVelocity(){
    return ((accel_cur < CONSTANT__ACCEL_BOUNDRY) && (accel_cur > -1*CONSTANT__ACCEL_BOUNDRY));
}

void publishThrottleCmd(float value){
    pacmod_msgs::PacmodCmd throttleCmd;
    throttleCmd.header.stamp = ros::Time::now();
    throttleCmd.header.frame_id ="accel_tuning";
    throttleCmd.enable = true;

    throttleCmd.f64_cmd = value;

    throttlePub.publish(throttleCmd);
}

// puts throttle at 40% for half a second, then holds throttle at 20%
void startCar(void){
    publishThrottleCmd(0.4);

    ros::Duration(0.5).sleep(); // sleep for half a second

    publishThrottleCmd(0.2);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "longitudinal_tuning");
    ros::NodeHandle n;

    throttlePub = n.advertise<pacmod_msgs::PacmodCmd>("as_rx/accel_cmd", 1);
    desAccelPub = n.advertise<std_msgs::Float64>("desAccel", 1);

    curAccelSub = n.subscribe<sensor_msgs::Imu>("imu/data", 1, curAccelCallback);

    // start the car by giving it a big throttle for a short amount of time, then bring throttle to lesser position
    startCar();

    while(ros::ok()){
        if(isConstantVelocity()){
            // pass off control to longitude controller
            // roslaunch echo echo.launch
            if(fork() == 0){
                // TODO exec long controller
                execlp("roslaunch", "roslaunch", "echo", "testecho.launch", NULL);
            } else {
                ros::Duration(0.5).sleep(); // sleep for half a second
                // give small desired acceleration
                std_msgs::Float64 desAccelMsg;
		        desAccelMsg.data = 0.1; // THIS MAY NEED TO CHANGE
		        desAccelPub.publish(desAccelMsg);
            } 
        } 
        
    }

    return 0;
}