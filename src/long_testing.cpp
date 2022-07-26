#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "automotive_platform_msgs/SpeedMode.h"

#include "pacmod_msgs/PositionWithSpeed.h"
#include "pacmod_msgs/PacmodCmd.h"


#include "sensor_msgs/Imu.h"

// THIS IS THE OPTION TO CHOOSE WHAT TEST TO ENABLE
// test1 car still, then given constant desired acceleration value
// test2 car still, then given series of increasing desired acceleration values
// test3 car still, then given series of increasing desired velocity values
// test4
// test5
#define test2

int enabled_;

int speed;

ros::Publisher throttlePub;
ros::Publisher brakePub;
ros::Publisher gearPub;
ros::Publisher enablePub;

ros::Subscriber curAccelSub;
ros::Subscriber curVelSub; 
ros::Subscriber curGSub;

ros::Publisher desAccelPub;
ros::Publisher desVelPub;

// this may need to be tuned, I just guessed on the value
// IF (-CONSTANT_ACCEL_BOUNDRY < [measured acceleration] < CONSTANT_ACCEL_BOUNDRY) THEN car is at zero acceleration (constant velocity)
#define CONSTANT_ACCEL_BOUNDRY 0.1 // [-0.1, 0.1]
#define NUMACCELVALS 6
#define NUMVELVALS 6

float accel_cur;
float accel_des = 0;

float vel_cur;
float G_val;

void curVelCallback(const std_msgs::Float64::ConstPtr& msg) {
	vel_cur = msg->data;
}

void curGCallback(const std_msgs::Float64::ConstPtr& msg){
    G_val = msg->data;
}

void curAccelCallback(const std_msgs::Float64::ConstPtr& msg) {
	accel_cur = (msg->data);	
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
    desVelPub = n.advertise<automotive_platform_msgs::SpeedMode>("ssc/arbitrated_speed_commands", 1);

    curAccelSub = n.subscribe<std_msgs::Float64>("xAccelFilter/accel_x", 1, curAccelCallback);
    curGSub = n.subscribe<std_msgs::Float64>("/G", 1, curGCallback);
    curVelSub = n.subscribe<std_msgs::Float64>("as_tx/vehicle_speed", 1, curVelCallback);

    std_msgs::Float64 desaccel_msg;
    automotive_platform_msgs::SpeedMode desvel_msg;
    
    ros::Time startTime = ros::Time::now();
    ros::Rate loop_rate(30);

    CarInit();

#ifdef test1
    // this is the throttle position that will be held until the car reaches constant velocity
    float ped_val = 0.0;
    // this is the acceleration command passed to the low level longitude controller after the car reaches constant velocity
    float accel_val = 0.3;
    int forked = 0;
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
#endif

#ifdef test2
    // this is the throttle position that will be held until the car reaches constant velocity
    float ped_val = 0.0;
    float accel_vals[NUMACCELVALS] = {0.0, 0.2, 0.4, 0.6, 0.8, 1};
    float prevtime;
    int idx_accel = 0;
    int interval = 3;
    int forked = 0;

    while(ros::ok()){
        // get current time
        ros::Time now = ros::Time::now();
        float time = now.toSec() - startTime.toSec();
        if(forked == 0){
            if(isConstantVelocity() && (time > 5)){
                forked = 1;
                
                if(fork() == 0){
                    // CHILD
                    // launch low level longitude  controller
                    execlp("roslaunch", "roslaunch", "echo", "testecho.launch", NULL);
                    ROS_INFO("Error: control should not reach here");
                } else {
                    // PARENT
                    prevtime = time;
                }
            } else {
                publishThrottleCmd(ped_val);    
            }
        } else {
            // begin giving increasing constant accel values
            float delta_time = time - prevtime;
            desaccel_msg.data = accel_vals[idx_accel];
            
            if(delta_time > interval){
                prevtime = time;
                idx_accel = (idx_accel + 1) % NUMACCELVALS;
            }
            
            desAccelPub.publish(desaccel_msg);
        }
    }
#endif

#ifdef test3
    // this is the throttle position that will be held until the car reaches constant velocity
    float ped_val = 0.0;
    float vel_vals[NUMVELVALS] = {0.0, 4.0, 8.0, 12.0, 16.0, 20.0};
    float prevtime;
    int idx_vel = 0;
    int interval = 5;
    int forked = 0;

    while(ros::ok()){
        // get current time
        ros::Time now = ros::Time::now();
        float time = now.toSec() - startTime.toSec();
        if(forked == 0){
            if(isConstantVelocity() && (time > 5)){
                forked = 1;
                
                if(fork() == 0){
                    // CHILD
                    // launch low level longitude  controller
                    execlp("roslaunch", "roslaunch", "echo", "echo.launch", NULL);
                    ROS_INFO("Error: control should not reach here, ya dun fucked up");
                } else {
                    // PARENT
                    prevtime = time;
                }
            } else {
                publishThrottleCmd(ped_val);    
            }
        } else {
            // begin giving increasing constant accel values
            float delta_time = time - prevtime;
            desvel_msg.speed = vel_vals[idx_vel];
            
            if(delta_time > interval){
                prevtime = time;
                idx_vel= (idx_vel + 1) % NUMVELVALS;
            }
            
            desVelPub.publish(desvel_msg);
        }
    }
#endif

#ifdef test4
    float accel_val = 0.3;
    float vel_HighCutoff = 15.0;
    float vel_LowCutoff = 10.0;
    int forked = 0;
    while(ros::ok()){
        if(forked == 0){
            // get current time
            ros::Time now = ros::Time::now();
            float time = now.toSec() - startTime.toSec();

            // check if the car is at a velocity after 5 seconds
            if((time > 5) && (vel_cur > vel_HighCutoff)){
                // car has reached cutoff velocity
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
                    desaccel_msg.data = -G_val;
                } 
            } else {
                desaccel_msg.data = accel_val;
                desAccelPub.publish(desaccel_msg);
            }
        } else {
            if(vel_cur < vel_LowCutoff){
                desaccel_msg.data = -accel_val;
            }

            desAccelPub.publish(desaccel_msg);
        }
        
        ros::spinOnce();
    }
#endif

#ifdef test5
    float accel_val = 0.3;
    float vel_Cutoff = 15.0;
    int forked = 0;
    while(ros::ok()){
        if(forked == 0){
            // get current time
            ros::Time now = ros::Time::now();
            float time = now.toSec() - startTime.toSec();

            // check if the car is at a velocity after 5 seconds
            if((time > 5) && (vel_cur > vel_Cutoff)){
                // car has reached cutoff velocity
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
                    desaccel_msg.data = -accel_val;
                } 
            } else {
                desaccel_msg.data = accel_val;
                desAccelPub.publish(desaccel_msg);
            }
        } else {
            desaccel_msg.data = -accel_val;

            desAccelPub.publish(desaccel_msg);
        }
        
        ros::spinOnce();
    }
#endif

    return 0;
}