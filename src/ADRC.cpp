#include <adrc/ADRC.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>


ADRC_CONTROLLER ADRC_X;
ADRC_CONTROLLER ADRC_Y;
ADRC_CONTROLLER ADRC_Z;

nav_msgs::Odometry odom_info;
nav_msgs::Odometry exp_info;
mavros_msgs::PositionTarget raw_local_info;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odomCallBack(const nav_msgs::Odometry& msg)
{
    
    odom_info.twist.twist.linear.x = msg.twist.twist.linear.x;
    odom_info.twist.twist.linear.y = msg.twist.twist.linear.y;
    odom_info.twist.twist.linear.z = msg.twist.twist.linear.z;

    odom_info.pose.pose.position.x = msg.pose.pose.position.x;
    odom_info.pose.pose.position.y = msg.pose.pose.position.y;
    odom_info.pose.pose.position.z = msg.pose.pose.position.z;
    
    return;
}

void expCallBack(const nav_msgs::Odometry& msg)
{
    
    exp_info.twist.twist.linear.x = msg.twist.twist.linear.x;
    exp_info.twist.twist.linear.y = msg.twist.twist.linear.y;
    exp_info.twist.twist.linear.z = msg.twist.twist.linear.z;

    exp_info.pose.pose.position.x = msg.pose.pose.position.x;
    exp_info.pose.pose.position.y = msg.pose.pose.position.y;
    exp_info.pose.pose.position.z = msg.pose.pose.position.z;
    
    return;
}


int main(int argc, char **argv)
{
    ADRC_Parameter ADRC_param_X;
    ADRC_Parameter ADRC_param_Y;
    ADRC_Parameter ADRC_param_Z;
    
    ros::init (argc, argv, "ADRC_Output");

    ros::NodeHandle ADRC_Node;

    ADRC_Node.getParam("ADRC_X_r", ADRC_param_X.r);
    ADRC_Node.getParam("ADRC_X_h", ADRC_param_X.h);
    ADRC_Node.getParam("ADRC_X_N", ADRC_param_X.N);
    ADRC_Node.getParam("ADRC_X_omega", ADRC_param_X.omega);
    ADRC_Node.getParam("ADRC_X_b0", ADRC_param_X.b0);
    ADRC_Node.getParam("ADRC_X_k_p", ADRC_param_X.k_p);
    ADRC_Node.getParam("ADRC_X_k_d", ADRC_param_X.k_d);

    ADRC_Node.getParam("ADRC_Y_r", ADRC_param_Y.r);
    ADRC_Node.getParam("ADRC_Y_h", ADRC_param_Y.h);
    ADRC_Node.getParam("ADRC_Y_N", ADRC_param_Y.N);
    ADRC_Node.getParam("ADRC_Y_omega", ADRC_param_Y.omega);
    ADRC_Node.getParam("ADRC_Y_b0", ADRC_param_Y.b0);
    ADRC_Node.getParam("ADRC_Y_k_p", ADRC_param_Y.k_p);
    ADRC_Node.getParam("ADRC_Y_k_d", ADRC_param_Y.k_d);

    ADRC_Node.getParam("ADRC_Z_r", ADRC_param_Z.r);
    ADRC_Node.getParam("ADRC_Z_h", ADRC_param_Z.h);
    ADRC_Node.getParam("ADRC_Z_N", ADRC_param_Z.N);
    ADRC_Node.getParam("ADRC_Z_omega", ADRC_param_Z.omega);
    ADRC_Node.getParam("ADRC_Z_b0", ADRC_param_Z.b0);
    ADRC_Node.getParam("ADRC_Z_k_p", ADRC_param_Z.k_p);
    ADRC_Node.getParam("ADRC_Z_k_d", ADRC_param_Z.k_d);

    ros::Rate loop_rate(100);

    ros::Subscriber sub_odom = ADRC_Node.subscribe("/mavros/local_position/odom", 100, odomCallBack);

    ros::Subscriber sub_exp_expect = ADRC_Node.subscribe("/motion_expect", 100, expCallBack);

    ros::Publisher pub = ADRC_Node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100); 

    ros::Subscriber state_sub = ADRC_Node.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    for (char i = 0; i < 100; i++)
    {
        while(ros::ok())
        { 
            raw_local_info.header.stamp = ros::Time::now();
            raw_local_info.header.frame_id = "base_link";
            raw_local_info.coordinate_frame = 1;
            raw_local_info.type_mask = 2503;
            raw_local_info.velocity.x = 0;
            raw_local_info.velocity.y = 0;
            raw_local_info.velocity.z = 0;
            raw_local_info.yaw = 0;
            pub.publish(raw_local_info);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
    while (current_state != "OFFBOARD"){};
    
    ADRC_X.Init_ADRC(ADRC_param_X);
    ADRC_Y.Init_ADRC(ADRC_param_Y);   
    ADRC_Z.Init_ADRC(ADRC_param_Z);
    
    while(ros::ok())
    { 
        raw_local_info.header.stamp = ros::Time::now();
        raw_local_info.header.frame_id = "base_link";
        raw_local_info.coordinate_frame = 1;
        raw_local_info.type_mask = 2503;
        raw_local_info.velocity.x = ADRC_X.ADRC_run(exp_info.pose.pose.position.x, odom_info.pose.pose.position.x)+exp_info.twist.twist.linear.x;
        raw_local_info.velocity.y = ADRC_Y.ADRC_run(exp_info.pose.pose.position.y, odom_info.pose.pose.position.y)+exp_info.twist.twist.linear.y;
        raw_local_info.velocity.z = ADRC_Z.ADRC_run(exp_info.pose.pose.position.z, odom_info.pose.pose.position.z)+exp_info.twist.twist.linear.z;
        raw_local_info.yaw = 0;
        pub.publish(raw_local_info);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}