/**
 * @file mavros_test_node.cpp
 *
 * Copyright (c) 2020 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * For a license to use on non-ModalAI hardware, please contact
 * license@modalai.com
 */

 // modified by @imuguruza

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <math.h>

#define FLIGHT_ALTITUDE 1.5f
#define RATE            20  // loop rate hz
#define RADIUS          10 // radius of figure 8 in meters
#define CYCLE_S         60   // time to complete one figure 8 cycle in seconds
#define STEPS           (CYCLE_S*RATE)

#define PI  3.14159265358979323846264338327950


mavros_msgs::State current_state;
geometry_msgs::PoseStamped read_local_pos;
mavros_msgs::PositionTarget path[STEPS];
mavros_msgs::PositionTarget position_home;



void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_state_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    read_local_pos = *msg;
    //ROS_INFO("Drone position is %f %f %f", read_local_pos.pose.position.x,
    //        read_local_pos.pose.position.y, read_local_pos.pose.position.z);
}


int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "cmd_hold");
    ros::NodeHandle nh;

    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State>
                                        ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped>
                                        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool>
                                        ("mavros/cmd/arming");
    ros::ServiceClient land_client      = nh.serviceClient<mavros_msgs::CommandTOL>
                                        ("mavros/cmd/land");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>
                                        ("mavros/set_mode");
    ros::Publisher target_local_pub     = nh.advertise<mavros_msgs::PositionTarget>
                                        ("mavros/setpoint_raw/local", 10);

    ros::Subscriber local_pose_sub      = nh.subscribe<geometry_msgs::PoseStamped>
                                        ("mavros/local_position/pose", 10, pos_state_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);


    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCT...");
    }



    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ROS_INFO("waiting for offboard mode");
    // wait for the system to be armed and in offboard mode
    // while(ros::ok()){
    //     //target_local_pub.publish(position_home);
    //     ros::spinOnce();
    //     rate.sleep();
    //     if(current_state.mode == "OFFBOARD" && current_state.armed)
    //       break;
    //     if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
    //       if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    //             ROS_INFO("Offboard enabled");
    //       last_request = ros::Time::now();
    //     } else {
    //       if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
    //         if( arming_client.call(arm_cmd) && arm_cmd.response.success)
    //             ROS_INFO("Vehicle armed");
    //       last_request = ros::Time::now();
    //       }
    //     }
    // }


    //UNCOMMENT THESE lines
    //========================
    //THIS DOES NOT WORK AS EXPECTED, DOES NOT LOITER
    // offb_set_mode.request.custom_mode = "AUTO.LOITER";
    // if( set_mode_client.call(offb_set_mode) &&
    //     offb_set_mode.response.mode_sent){
    //     ROS_INFO("Loiter enabled");
    // }

    // OR THESE ONES
    //========================
    //WORKS OK
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
        land_cmd.response.success)){
        //local_pos_pub.publish(pose);
        ROS_INFO("tring to land");
      }


    return 0;
}


/* HOW TO ARM , LAND AND CHANGE MODE

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    if( set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
    }

    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

        ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
*/
