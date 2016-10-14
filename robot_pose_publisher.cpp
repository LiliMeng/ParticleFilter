#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
using namespace std;

struct particle
{
        double x;
        double y;
        double theta;
        double weight;


        particle(double posex, double posey, double poset, double w)
        {
            x=posex;
            y=posey;
            theta=poset;
            weight=w;

        }
};


void motion_model_velocity_update(double linear_vel, double angular_vel, vector<particle> pose, vector<particle> &new_pose, double time_interval, int num_particles)
{

    double alpha1 = 5;
    double alpha2 = 10;
    double alpha3 = 5;
    double alpha4 = 0.2;
    double alpha5 = 3;
    double alpha6 = 1;

    std::random_device r;
    std::mt19937 gen(r());

    normal_distribution<> sample1(0, alpha1*pow(linear_vel,2)+alpha2*pow(angular_vel,2));
    normal_distribution<> sample2(0, alpha3*pow(linear_vel,2)+alpha4*pow(angular_vel,2));
    normal_distribution<> sample3(0, alpha5*pow(linear_vel,2)+alpha6*pow(angular_vel,2));

    cout<<"no problem with sampling"<<endl;

    double new_linear_vel=linear_vel+sample1(gen);
    double new_angualr_vel=angular_vel+sample2(gen);
    double gamma=sample3(gen);

    for(int i=0; i<num_particles; i++)
    {
        double new_linear_vel=linear_vel+sample1(gen);
        double new_angular_vel=angular_vel+sample2(gen);
        double gamma=sample3(gen);


        new_pose[i].x=pose[i].x - new_linear_vel/new_angular_vel*sin(pose[i].theta) + new_linear_vel/new_angular_vel*sin(pose[i].theta+new_angular_vel*time_interval);
        new_pose[i].y=pose[i].y + new_linear_vel/new_angular_vel*cos(pose[i].theta) - new_linear_vel/new_angular_vel*cos(pose[i].theta+new_angular_vel*time_interval);
        new_pose[i].theta=pose[i].theta + new_angular_vel*time_interval + gamma*time_interval;
        cout<<new_pose[i].x<<" "<<new_pose[i].y<<" "<<new_pose[i].theta<<endl;
    }

}

/*!
 * Creates and runs the robot_pose_publisher node.
 *
 */
float f = 0.0;
int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

   /// configure parameters
  std::string map_frame, base_frame;

  nh_priv.param<std::string>("map_frame", map_frame, "/map");
  nh_priv.param<std::string>("base_frame", base_frame, "/base_link");


  double publish_frequency=10;
  bool is_stamped;

  ros::Publisher p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  ros::Publisher m_particlecloudPub = nh.advertise<geometry_msgs::PoseArray>("particlecloud",1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markerArray", 1);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;



  std::vector<particle> pose, new_pose;

  int num_particles=1000;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;


  for(int i=0; i<num_particles; i++)
  {
        pose.push_back(particle(x,y,th, 1.0/num_particles));
  }

  for(int i=0; i<num_particles; i++)
  {
        new_pose.push_back(particle(x,y,th, 1.0/num_particles));
  }


  tf::TransformBroadcaster tf_map_to_pose_broadcaster;


  ros::Rate rate(publish_frequency);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  vector<double> poseX, poseY, poseZ;

  visualization_msgs::MarkerArray ma;


  while (nh.ok())
  {


    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    double map_th=0.0;
     //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(map_th);
       //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped tf_map_to_pose;
    tf_map_to_pose.header.stamp = ros::Time::now();
    tf_map_to_pose.header.frame_id = "map";
    tf_map_to_pose.child_frame_id = "base_link";

    tf_map_to_pose.transform.translation.x = 0.0;
    tf_map_to_pose.transform.translation.y = 0.0;
    tf_map_to_pose.transform.translation.z = 0.0;
    tf_map_to_pose.transform.rotation = map_quat;

    tf_map_to_pose_broadcaster.sendTransform(tf_map_to_pose);

    // construct a pose message
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.header.stamp = ros::Time::now();


    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);

    pose_stamped.pose.orientation = pose_quat;

    pose_stamped.pose.position.x = tf_map_to_pose.transform.translation.x+x;
    pose_stamped.pose.position.y = tf_map_to_pose.transform.translation.y+y;
    pose_stamped.pose.position.z = tf_map_to_pose.transform.translation.z+0.0;

    poseX.push_back(pose_stamped.pose.position.x);
    poseY.push_back(pose_stamped.pose.position.y);
    poseZ.push_back(pose_stamped.pose.position.z);


    motion_model_velocity_update(vx, vth, pose, new_pose, dt, num_particles);

    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "base_link";
    cloud_msg.poses.resize(num_particles);

    for(int i=0;i<num_particles;i++)
    {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(new_pose[i].theta),
                                 tf::Vector3(tf_map_to_pose.transform.translation.x+new_pose[i].x, tf_map_to_pose.transform.translation.x+new_pose[i].y, tf_map_to_pose.transform.translation.z+0)),
                                 cloud_msg.poses[i]);

        pose[i].x=new_pose[i].x;
        pose[i].y=new_pose[i].y;
        pose[i].theta=new_pose[i].theta;
    }


    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.id++;


    marker.pose.position.x = pose_stamped.pose.position.x;
    marker.pose.position.y = pose_stamped.pose.position.y;
    marker.pose.position.z = pose_stamped.pose.position.z;
    marker.pose.orientation= pose_quat;
    ma.markers.push_back(marker);


      //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    m_particlecloudPub.publish(cloud_msg);
    p_pub.publish(pose_stamped);
    marker_pub.publish(ma);
    last_time = current_time;
    rate.sleep();

  }

}
