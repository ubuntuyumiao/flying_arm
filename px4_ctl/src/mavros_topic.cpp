#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <math.h>
#include "ctime"
#include "time.h"
double array_x[] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 
                     10.0, 8.0, 6.0, 4.0, 2.0, 0.0
} ;
double array_y[] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 
                     10.0, 8.0, 6.0, 4.0, 2.0, 0.0
} ;

template<class T>
int array_length(T& arr)
{
    //cout << sizeof(arr[0]) << endl;
    //cout << sizeof(arr) << endl;
    return sizeof(arr) / sizeof(arr[0]);
}

int angle,angleStep;
int array_num ;
bool isFly;
bool hasInitFlyCircle;
bool hasInitFlyDiscrete;
float radius;
float speed;

ros::Publisher local_pos_pub ;
ros::Publisher vel_sp_pub;

geometry_msgs::PoseStamped currentPos,nextPos,initCirclePos,initDiscretePos;
geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped local_twist; 
mavros_msgs::State current_state;
std_msgs::Float64MultiArray  IMU_Euler ;

void flyto(geometry_msgs::PoseStamped pose_,float speed_)
{
  if(!hasInitFlyDiscrete) {
    initDiscretePos = currentPos;
    nextPos = pose_;
    
    nextPos.pose.position.x = pose_.pose.position.x;
    nextPos.pose.position.y = pose_.pose.position.y;
    nextPos.pose.position.z = pose_.pose.position.z;
 
    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;
    velocityVector.z = nextPos.pose.position.z - currentPos.pose.position.z;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y + velocityVector.z*velocityVector.z );
    velocityVector.x = velocityVector.x/length * speed_;
    velocityVector.y = velocityVector.y/length * speed_;
    velocityVector.z = velocityVector.z/length * speed_;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.twist.linear.z = velocityVector.z;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
    ROS_INFO_STREAM("next point:" << nextPos.pose.position.x << "  " << nextPos.pose.position.y<< "  "<<nextPos.pose.position.z);
    hasInitFlyDiscrete = true;
    return;
  } 
  
  // judge whether is reached
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                        ( currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y) +
			( currentPos.pose.position.z - nextPos.pose.position.z)*(currentPos.pose.position.z - nextPos.pose.position.z));
  double threshold = 0.3 ;
  if (distance < threshold)
  {
   // send next pos
    nextPos = pose_;
    local_pos_pub.publish<geometry_msgs::PoseStamped>(nextPos);
  }
  else{
    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;
    velocityVector.z = nextPos.pose.position.z - currentPos.pose.position.z;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y+ velocityVector.z*velocityVector.z);
    velocityVector.x = velocityVector.x/length * speed_;
    velocityVector.y = velocityVector.y/length * speed_;
    velocityVector.z = velocityVector.z/length * speed_;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.twist.linear.z = velocityVector.z;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
    
    ROS_INFO_STREAM("next point:" << nextPos.pose.position.x << "  " << nextPos.pose.position.y<< "  "<<nextPos.pose.position.z);
    ROS_INFO_STREAM("curr vel:" << vs.twist.linear.x << "  " << vs.twist.linear.y<< "  "<<vs.twist.linear.z);
  }
}
void flydiscrete(double speed)
{
  if(!hasInitFlyDiscrete) {
    initDiscretePos = currentPos;
    nextPos = initDiscretePos;
    array_num ++ ;
    
    nextPos.pose.position.x = array_x[array_num] + initDiscretePos.pose.position.x;
    nextPos.pose.position.y = array_y[array_num] + initDiscretePos.pose.position.y;
    
    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y);
    velocityVector.x = velocityVector.x/length * speed;
    velocityVector.y = velocityVector.y/length * speed;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
    ROS_INFO_STREAM("next point:" << nextPos.pose.position.x << "  " << nextPos.pose.position.y);
    hasInitFlyDiscrete = true;
    return;
  } 
  
  // judge whether is reached
  bool  isReached = false;
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                        ( currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
  double threshold = 0.2 ;
  if (distance < threshold)
  {
    isReached = true;
  }
  if (isReached)
  {
    // send next pos
    array_num++ ;
    if(array_num > 12) array_num = 0;
     
    nextPos = initDiscretePos;
    
    nextPos.pose.position.x = array_x[array_num] + initDiscretePos.pose.position.x;
    nextPos.pose.position.y = array_y[array_num] + initDiscretePos.pose.position.y;
    
    ROS_INFO_STREAM("next point:" << nextPos.pose.position.x << "  " << nextPos.pose.position.y);
  }

    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y);
    velocityVector.x = velocityVector.x/length * speed;
    velocityVector.y = velocityVector.y/length * speed;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
}


void flyCircleWithRadius(double r,double speed)
{
  if(!hasInitFlyCircle){
    initCirclePos = currentPos;
    nextPos = initCirclePos;
    angle = angle + angleStep;
    nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
    
    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y);
    velocityVector.x = velocityVector.x/length * speed;
    velocityVector.y = velocityVector.y/length * speed;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
    ROS_INFO_STREAM("next angle:" << angle);
    hasInitFlyCircle = true;
    return;
  } 
  
  // judge whether is reached
  bool isReached = false;
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
  double threshold = 0.4;
  if (distance < threshold)
  {
    isReached = true;
  }

  if (isReached)
  {
    // send next pos
    angle = angle + angleStep;
    if(angle > 360) angle = angle - 360;
    nextPos = initCirclePos;
    nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
    ROS_INFO_STREAM("next angle:" << angle);

  }

    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y);
    velocityVector.x = velocityVector.x/length * speed;
    velocityVector.y = velocityVector.y/length * speed;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
 
  
}
//给定一个单位长度的旋转轴(x, y, z)和一个角度θ。对应的四元数为：
//q=((x,y,z)sinθ2, cosθ2) 
geometry_msgs::PoseStamped set_localxyz( double x, double y, double z, 
		  double ori_x, double ori_y, double ori_z, double ori_w)
{
  geometry_msgs::PoseStamped pose_ ;
    pose_.pose.position.x = x;
    pose_.pose.position.y = y;
    pose_.pose.position.z = z;
    pose_.pose.orientation.x = ori_x ;
    pose_.pose.orientation.y = ori_y ;
    pose_.pose.orientation.z = ori_z ;
    pose_.pose.orientation.w = ori_w ;
    
    return pose_ ;
}
void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos = *msg;
}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void qua_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  //quaternion to erlue angle

  IMU_Euler.data.resize(3);
  IMU_Euler.data[0]= 180.0/3.1415926 * atan(2*(imu_msg->orientation.w*imu_msg->orientation.x+imu_msg->orientation.y*imu_msg->orientation.z)
	  /(1-2*(imu_msg->orientation.x*imu_msg->orientation.x+imu_msg->orientation.y*imu_msg->orientation.y))) ;
 
  IMU_Euler.data[1]= 180.0/3.1415926 * asin(2*(imu_msg->orientation.w*imu_msg->orientation.y-
	  imu_msg->orientation.z*imu_msg->orientation.x)) ;
  
  IMU_Euler.data[2]= 180.0/3.1415926 * atan(2*(imu_msg->orientation.w*imu_msg->orientation.z+imu_msg->orientation.x*imu_msg->orientation.y)
	  /(1-2*(imu_msg->orientation.y*imu_msg->orientation.y+imu_msg->orientation.z*imu_msg->orientation.z))) ;
				 
//ROS_INFO_STREAM("Euler_r: " << IMU_Euler.data[0] << 
//		  "Euler_p: " << IMU_Euler.data[0] << 
//		  "Euler_y"  << IMU_Euler.data[0]);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_topic");
    ros::NodeHandle nh("/");
//------------------订阅、发布话题------------------
    ros::Subscriber state_sub= nh.subscribe<mavros_msgs::State>("/mavros/state", 5,state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",5,qua_callback);
    
    ros::Subscriber localPositionSubsciber = nh.subscribe("/mavros/local_position/pose", 5, localPositionReceived);
    
    vel_sp_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 1);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
	    ("/mavros/setpoint_velocity/cmd_vel",1);
    ros::Publisher tur_pub = nh.advertise<geometry_msgs::Twist>
	    ("/turtle1/cmd_vel",1);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");   
    ros::Publisher Euler = nh.advertise<std_msgs::Float64MultiArray>
	    ("ymdefined/imu/erlue",1) ;
	    
  isFly = false;
  hasInitFlyDiscrete = false ;
  hasInitFlyCircle   = false;
  angle = 0;
  radius = 3;
  angleStep = 5;
  speed = 1.2;
//------------------消息过滤器同步------------------
   /*  typedef message_filters::sync_policies::ApproximateTime
     <
     sensor_msgs::Imu,
     nav_msgs::Odometry
     > MySyncPolicy;    
    message_filters::Synchronizer<MySyncPolicy>sync
    (MySyncPolicy(20),
     imu_sub,
     now_pos_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2)) ;*/
   
    ros::NodeHandle nh_param("~");
    double px4start_x ;
    double px4start_y ;
    double px4start_z ;
    
    double px4start_orix ;
    double px4start_oriy ;
    double px4start_oriz ;
    double px4start_oriw ; 
    
    if(true){
    nh_param.param<double>("px4start_x", px4start_x, 0.0);
    nh_param.param<double>("px4start_y", px4start_y, 0.0);
    nh_param.param<double>("px4start_z", px4start_z, 3.0);
  
    nh_param.param<double>("px4start_orix", px4start_orix, 0.0);
    nh_param.param<double>("px4start_oriy", px4start_oriy, 0.0);
    nh_param.param<double>("px4start_oriz", px4start_oriz, 0.0);
    nh_param.param<double>("px4start_oriw", px4start_oriw, 1.0);
    }
    
    ros::Rate rate(20.0);
    geometry_msgs::Twist turtle1sim;
    geometry_msgs::PoseStamped pose;
    pose = set_localxyz(px4start_x, px4start_y, px4start_z,
      px4start_orix, px4start_oriy, px4start_oriz, px4start_oriw
    );
    
    mavros_msgs::SetMode offb_set_mode;   
    mavros_msgs::CommandBool arm_cmd;
//------------------连接到pixhawk------------------
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    offb_set_mode.request.custom_mode = "OFFBOARD"; 
    arm_cmd.request.value = true;
    
    ros::Time control_start = ros::Time::now();
    ros::Time last_time_record = ros::Time::now();  // ALTCTL  MANUAL POSCTL

    while(ros::ok())
    {  
        if( current_state.mode != "OFFBOARD" )
	   {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	    {
	      ROS_INFO("Offboard enabled");
	    }
	        last_time_record = ros::Time::now();
	   } 
     else   {
       if(arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
	  ROS_INFO("Fly ing. Be Careful!!!!!") ;
	  break ;
	}
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

    geometry_msgs::PoseStamped pose_tar ;
    array_num = 0;
    while(ros::ok())
    {
      nh_param.getParam("px4start_x",px4start_x);
      nh_param.getParam("px4start_y",px4start_y);
      nh_param.getParam("px4start_z",px4start_z);
      
      nh_param.getParam("px4start_orix",px4start_orix);
      nh_param.getParam("px4start_oriy",px4start_oriy);
      nh_param.getParam("px4start_oriz",px4start_oriz);
      nh_param.getParam("px4start_oriw",px4start_oriw);

      vs.header.seq++;
      if((ros::Time::now()- last_time_record )< ros::Duration(10.0))
	  local_pos_pub.publish(pose);
      else{
	
      if(!isFly){
        vs.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM("send ps" << ps);
        vel_sp_pub.publish(vs);
	isFly = true ;
      } else 
      {
      pose = set_localxyz(6,6,6,
			  0,0,0,1) ;
      flyto(pose,speed);
      //flydiscrete(speed);
      //flyCircleWithRadius(radius,speed);
      
      }
      }
      //pose.header.stamp = ros::Time::now() ;
    
      Euler.publish<std_msgs::Float64MultiArray>(IMU_Euler) ;
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
