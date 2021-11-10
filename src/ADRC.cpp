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

geometry_msgs::Point exp_info;
nav_msgs::Odometry odom_info;
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

void expCallBack(const geometry_msgs::Point& msg)
{
    
    exp_info.x = msg.x; 
    exp_info.y = msg.y; 
    exp_info.z = msg.z; 

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
    ADRC_Node.getParam("ADRC_X_beta_01", ADRC_param_X.beta_01);
    ADRC_Node.getParam("ADRC_X_beta_02", ADRC_param_X.beta_02);
    ADRC_Node.getParam("ADRC_X_beta_03", ADRC_param_X.beta_03);
    ADRC_Node.getParam("ADRC_X_b0", ADRC_param_X.b0);
    ADRC_Node.getParam("ADRC_X_k_0", ADRC_param_X.k_0);
    ADRC_Node.getParam("ADRC_X_k_1", ADRC_param_X.k_1);
    ADRC_Node.getParam("ADRC_X_k_2", ADRC_param_X.k_2);
    ADRC_Node.getParam("ADRC_X_N1", ADRC_param_X.N1);
    ADRC_Node.getParam("ADRC_X_C", ADRC_param_X.C);
    ADRC_Node.getParam("ADRC_X_alpha1", ADRC_param_X.alpha1);
    ADRC_Node.getParam("ADRC_X_alpha2", ADRC_param_X.alpha2);
    ADRC_Node.getParam("ADRC_X_zeta", ADRC_param_X.zeta);

    ADRC_Node.getParam("ADRC_Y_r", ADRC_param_Y.r);
    ADRC_Node.getParam("ADRC_Y_h", ADRC_param_Y.h);
    ADRC_Node.getParam("ADRC_Y_N", ADRC_param_Y.N);
    ADRC_Node.getParam("ADRC_Y_beta_01", ADRC_param_Y.beta_01);
    ADRC_Node.getParam("ADRC_Y_beta_02", ADRC_param_Y.beta_02);
    ADRC_Node.getParam("ADRC_Y_beta_03", ADRC_param_Y.beta_03);
    ADRC_Node.getParam("ADRC_Y_b0", ADRC_param_Y.b0);
    ADRC_Node.getParam("ADRC_Y_k_0", ADRC_param_Y.k_0);
    ADRC_Node.getParam("ADRC_Y_k_1", ADRC_param_Y.k_1);
    ADRC_Node.getParam("ADRC_Y_k_2", ADRC_param_Y.k_2);
    ADRC_Node.getParam("ADRC_Y_N1", ADRC_param_Y.N1);
    ADRC_Node.getParam("ADRC_Y_C", ADRC_param_Y.C);
    ADRC_Node.getParam("ADRC_Y_alpha1", ADRC_param_Y.alpha1);
    ADRC_Node.getParam("ADRC_Y_alpha2", ADRC_param_Y.alpha2);
    ADRC_Node.getParam("ADRC_Y_zeta", ADRC_param_Y.zeta);

    ADRC_Node.getParam("ADRC_Z_r", ADRC_param_Z.r);
    ADRC_Node.getParam("ADRC_Z_h", ADRC_param_Z.h);
    ADRC_Node.getParam("ADRC_Z_N", ADRC_param_Z.N);
    ADRC_Node.getParam("ADRC_Z_beta_01", ADRC_param_Z.beta_01);
    ADRC_Node.getParam("ADRC_Z_beta_02", ADRC_param_Z.beta_02);
    ADRC_Node.getParam("ADRC_Z_beta_03", ADRC_param_Z.beta_03);
    ADRC_Node.getParam("ADRC_Z_b0", ADRC_param_Z.b0);
    ADRC_Node.getParam("ADRC_Z_k_0", ADRC_param_Z.k_0);
    ADRC_Node.getParam("ADRC_Z_k_1", ADRC_param_Z.k_1);
    ADRC_Node.getParam("ADRC_Z_k_2", ADRC_param_Z.k_2);
    ADRC_Node.getParam("ADRC_Z_N1", ADRC_param_Z.N1);
    ADRC_Node.getParam("ADRC_Z_C", ADRC_param_Z.C);
    ADRC_Node.getParam("ADRC_Z_alpha1", ADRC_param_Z.alpha1);
    ADRC_Node.getParam("ADRC_Z_alpha2", ADRC_param_Z.alpha2);
    ADRC_Node.getParam("ADRC_Z_zeta", ADRC_param_Z.zeta);

    ros::Rate loop_rate(100);



    ros::Subscriber sub_odom = ADRC_Node.subscribe("/mavros/local_position/odom", 100, odomCallBack);

    ros::Subscriber sub_expect = ADRC_Node.subscribe("/bot_motion_expect", 100, expCallBack);

    ros::Publisher pub = ADRC_Node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100); 

    ros::Subscriber state_sub = ADRC_Node.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::ServiceClient set_mode_client = ADRC_Node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    


    ADRC_X.Init_ADRC(ADRC_param_X);
    ADRC_Y.Init_ADRC(ADRC_param_Y);   
    ADRC_Z.Init_ADRC(ADRC_param_Z);
    
    while(ros::ok())
    { 
        raw_local_info.header.stamp = ros::Time::now();
        raw_local_info.header.frame_id = "base_link";
        raw_local_info.coordinate_frame = 1;
        raw_local_info.type_mask = 2503;
        raw_local_info.velocity.x = ADRC_X.ADRC_run(exp_info.x, odom_info.pose.pose.position.x, odom_info.twist.twist.linear.x);
        raw_local_info.velocity.y = ADRC_Y.ADRC_run(exp_info.y, odom_info.pose.pose.position.y, odom_info.twist.twist.linear.y);
        raw_local_info.velocity.z = ADRC_Z.ADRC_run(exp_info.z, odom_info.pose.pose.position.z, odom_info.twist.twist.linear.z);
        raw_local_info.yaw = 1.5708;
        pub.publish(raw_local_info);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}