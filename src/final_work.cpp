#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

void broadcastTF(const ros::TimerEvent& timer_event)
{
  //A_B
  Eigen::Quaterniond q_B_A(1, 0, 0, 0);
  Eigen::Vector3d v_B_A(1, 0, 0);

  static tf::TransformBroadcaster br;
  tf::Transform tf_B_A;
  tf::Quaternion tf_q_B_A(q_B_A.x(),q_B_A.y(),q_B_A.z(),q_B_A.w());
  tf::Vector3 tf_v_B_A(v_B_A(0), v_B_A(1), v_B_A(2));
  //std::cout<<"quaternion"<<q_A_B.x()<<"\t"<<q_A_B.y()<<"\t"<<q_A_B.z()<<std::endl;

  tf_B_A.setOrigin(tf_v_B_A);
  tf_B_A.setRotation(tf_q_B_A);
  br.sendTransform(tf::StampedTransform(tf_B_A
                                        ,ros::Time::now()
                                        ,"A"
                                        ,"B"));

  //A_C
  // w x y z
  Eigen::Quaterniond q_C_A(1, 0, 0, 0);
  Eigen::Vector3d v_C_A(0, 1, 0);

  tf::Transform tf_C_A;
  tf::Quaternion tf_q_C_A(q_C_A.x(),q_C_A.y(),q_C_A.z(),q_C_A.w());
  tf::Vector3 tf_v_C_A(v_C_A(0), v_C_A(1), v_C_A(2));
  //std::cout<<"quaternion"<<q_A_B.x()<<"\t"<<q_A_B.y()<<"\t"<<q_A_B.z()<<std::endl;

  tf_C_A.setOrigin(tf_v_C_A);
  tf_C_A.setRotation(tf_q_C_A);
  br.sendTransform(tf::StampedTransform(tf_C_A
                                        ,ros::Time::now()
                                        ,"A"
                                        ,"C"));

  //C 和 C1 connectation
  Eigen::Quaterniond q_C_C1(1, 0, 0, 0);
  Eigen::Vector3d v_C_C1(0, 0, 0);

  tf::Transform tf_C_C1;
  tf::Quaternion tf_q_C_C1(q_C_C1.x(),q_C_C1.y(),q_C_C1.z(),q_C_C1.w());
  tf::Vector3 tf_v_C_C1(v_C_C1(0), v_C_C1(1), v_C_C1(2));

  tf_C_C1.setOrigin(tf_v_C_C1);
  tf_C_C1.setRotation(tf_q_C_C1);
  br.sendTransform(tf::StampedTransform(tf_C_C1
                                        ,ros::Time::now()
                                        ,"C"
                                        ,"C1"));


  //B_C
  Eigen::Quaterniond q_B_C;
  Eigen::Vector3d v_B_C;

  q_B_C = q_C_A * q_B_A.inverse();
  v_B_C = v_B_A + q_B_A.inverse() * (- 1 * v_C_A);


  tf::Transform tf_B_C;
  tf::Quaternion tf_q_B_C(q_B_C.x(),q_B_C.y(),q_B_C.z(),q_B_C.w());
  tf::Vector3 tf_v_B_C(v_B_C(0), v_B_C(1), v_B_C(2));

  tf_B_C.setOrigin(tf_v_B_C);
  tf_B_C.setRotation(tf_q_B_C);
  br.sendTransform(tf::StampedTransform(tf_B_C
                                        ,ros::Time::now()
                                        ,"C1"
                                        ,"B1"));


}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_learning");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
