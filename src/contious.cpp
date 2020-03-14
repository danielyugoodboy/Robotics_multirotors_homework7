#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include "week_3/ncrl_tf.h"
#include <string>
void transpoint_test(void)
{
  static tf::TransformBroadcaster br;
  auto now = ros::Time::now();

  ncrl_tf::Trans tf, tf2 ,tf3, world_now;
  tf::Transform world;

  tf.v << 1.0,1.0,0.0;
  Eigen::Vector3d euler(60,0,0);
  tf.q = ncrl_tf::Euler2Q(euler);

  tf2.v << 1.0,1.0,0;
  tf2.q = ncrl_tf::Euler2Q(euler);

  tf3.v<<1.0,1.0,0;
  tf3.q = ncrl_tf::Euler2Q(euler);

  ncrl_tf::setTfTrans(world , tf.q ,tf.v);
  br.sendTransform(tf::StampedTransform(world, now ,"world","A"));

  world_now.q = tf.q * tf2.q;
  world_now.v = tf.q.inverse() * tf2.v + tf.v;

  ncrl_tf::setTfTrans(world , world_now.q ,world_now.v);
  br.sendTransform(tf::StampedTransform(world, now ,"world","B"));

  world_now.v = world_now.q.inverse() * tf3.v + world_now.v;
  world_now.q = world_now.q * tf3.q;

  ncrl_tf::setTfTrans(world , world_now.q ,world_now.v);
  br.sendTransform(tf::StampedTransform(world, now ,"world","C"));

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_class_test");
  ros::NodeHandle nh;

  ROS_INFO("start tf class test");

  int count = 0;

  ros::Rate r(1);
  while(ros::ok()){
    transpoint_test();
    count = 0;
    ros::spinOnce();
    r.sleep();
  }
}
