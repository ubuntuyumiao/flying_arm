#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
static int loop=0;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
std_msgs::Float64MultiArray  IMU_Euler ;
//给定一个单位长度的旋转轴(x, y, z)和一个角度θ。对应的四元数为：
//q=((x,y,z)sinθ2, cosθ2) 
void qua_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //quaternion to erlue angle
  IMU_Euler.data.resize(3);
  IMU_Euler.data[0]= 180.0/3.1415926 * atan(2*(msg->orientation.w*msg->orientation.x+msg->orientation.y*msg->orientation.z)
	  /(1-2*(msg->orientation.x*msg->orientation.x+msg->orientation.y*msg->orientation.y))) ;
 
  IMU_Euler.data[1]= 180.0/3.1415926 * asin(2*(msg->orientation.w*msg->orientation.y-
	  msg->orientation.z*msg->orientation.x)) ;
  
  IMU_Euler.data[2]= 180.0/3.1415926 * atan(2*(msg->orientation.w*msg->orientation.z+msg->orientation.x*msg->orientation.y)
	  /(1-2*(msg->orientation.y*msg->orientation.y+msg->orientation.z*msg->orientation.z))) ;
				 
//ROS_INFO_STREAM("Euler_r: " << IMU_Euler.data[0] << 
//		  "Euler_p: " << IMU_Euler.data[0] << 
//		  "Euler_y"  << IMU_Euler.data[0]);



}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_ctl");
    ros::NodeHandle nh("/");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");   
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",5,qua_callback);
    
    ros::Publisher Euler = nh.advertise<std_msgs::Float64MultiArray>
	    ("ymdefined/imu/erlue",10) ;
    
    ros::NodeHandle nh_param("~");
    double px4start_x ;
    double px4start_y ;
    double px4start_z ;
    
    double px4start_orix ;
    double px4start_oriy ;
    double px4start_oriz ;
    double px4start_oriw ;
    nh_param.param<double>("px4start_x", px4start_x, 0.0);
    nh_param.param<double>("px4start_y", px4start_y, 0.0);
    nh_param.param<double>("px4start_z", px4start_z, 3.0);
    
    
    nh_param.param<double>("px4start_orix", px4start_orix, 0.0);
    nh_param.param<double>("px4start_oriy", px4start_oriy, 0.0);
    nh_param.param<double>("px4start_oriz", px4start_oriz, 0.0);
    nh_param.param<double>("px4start_oriw", px4start_oriw, 1.0);
    
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //system("rosrun rqt_plot rqt_plot /mavros/imu/data/linear_acceleration") ;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = px4start_x;
    pose.pose.position.y = px4start_y;
    pose.pose.position.z = px4start_z;
     
    pose.pose.orientation.x = px4start_orix ;
    pose.pose.orientation.y = px4start_oriy ;
    pose.pose.orientation.z = px4start_oriz ;
    pose.pose.orientation.w = px4start_oriw ;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
  //  pose.pose.position.x = 0.5;
  //  pose.pose.position.y = 0.5;
  //  pose.pose.position.z = 3.5;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time control_start = ros::Time::now();
    ros::Time last_time_record = ros::Time::now();  // ALTCTL  MANUAL POSCTL

    while(( ros::Time::now() -last_time_record < ros::Duration(10.0)))
    {  
        if( current_state.mode != "OFFBOARD" )
	   {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	    {
	      ROS_INFO("Offboard enabled");
	    }
	        last_time_record = ros::Time::now();
	   } 
	else  if(!current_state.armed && (ros::Time::now() - last_time_record > ros::Duration(3.0)))
        if(arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
	  ROS_INFO("Fly ing. Be Careful!!!!!") ;
	      break ;
	}
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    last_time_record = ros::Time::now();
          nh_param.getParam("px4start_x",px4start_x);
      nh_param.getParam("px4start_y",px4start_y);
      nh_param.getParam("px4start_z",px4start_z);
      
      nh_param.getParam("px4start_orix",px4start_orix);
      nh_param.getParam("px4start_oriy",px4start_oriy);
      nh_param.getParam("px4start_oriz",px4start_oriz);
      nh_param.getParam("px4start_oriw",px4start_oriw);
      pose.pose.position.x = px4start_x; 
      pose.pose.position.y = px4start_y;
      pose.pose.position.z = px4start_z;
    while(ros::ok())
    {
    if(ros::Time::now() - last_time_record > ros::Duration(6.0)) 
    {
      pose.pose.position.x = 0.0; 
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 4.0;
      
    }
        if(ros::Time::now() - last_time_record > ros::Duration(14.0)) 
    {
      pose.pose.position.x = 4.0; 
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 4.0;
      
    }
        if(ros::Time::now() - last_time_record > ros::Duration(20.0)) 
    {
      pose.pose.position.x = 4.0; 
      pose.pose.position.y = 4.0;
      pose.pose.position.z = 4.0;
      
    }
        if(ros::Time::now() - last_time_record > ros::Duration(26.0)) 
    {
      pose.pose.position.x = 0.0; 
      pose.pose.position.y = 4.0;
      pose.pose.position.z = 4.0;
      last_time_record = ros::Time::now() ;
    }
      pose.pose.orientation.x = px4start_orix ;
      pose.pose.orientation.y = px4start_oriy ;
      pose.pose.orientation.z = px4start_oriz ;
      pose.pose.orientation.w = px4start_oriw ;
      local_pos_pub.publish(pose);
      Euler.publish<std_msgs::Float64MultiArray>(IMU_Euler) ;
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
