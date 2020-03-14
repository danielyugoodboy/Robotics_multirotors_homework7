#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;

  //B_C
  //Eigen::Quaterniond w x y z
  //tf::Quaternion x y z w
  //轉60度
  //不可以 1/2
  Eigen::Quaterniond q_C_B(std::sqrt(3)/2, 0, 0, 0.5);
  Eigen::Vector3d v_C_B(1, 0, 0);

  tf::Transform tf_C_B;
  tf::Quaternion tf_q_C_B(q_C_B.x(),q_C_B.y(),q_C_B.z(),q_C_B.w());
  tf::Vector3 tf_v_C_B(v_C_B(0), v_C_B(1), v_C_B(2));
  //std::cout<<"q_B_C"<<q_B_C.x()<<"\t"<<q_B_C.y()<<"\t"<<q_B_C.z()<<std::endl;

  tf_C_B.setOrigin(tf_v_C_B);
  tf_C_B.setRotation(tf_q_C_B);
  //C 指向 B
  br.sendTransform(tf::StampedTransform(tf_C_B
                                        ,ros::Time::now()
                                        ,"C"
                                        ,"B"));


  //B_A
  //轉90度
  Eigen::Quaterniond q_B_A(std::sqrt(2)/2, 0, 0, std::sqrt(2)/2);
  Eigen::Vector3d v_B_A(1, 0, 0);

  tf::Transform tf_B_A;
  tf::Quaternion tf_q_B_A(q_B_A.x(),q_B_A.y(),q_B_A.z(),q_B_A.w());
  tf::Vector3 tf_v_B_A(v_B_A(0), v_B_A(1), v_B_A(2));

  //std::cout<<"q_A_B"<<q_A_B.x()<<"\t"<<q_A_B.y()<<"\t"<<q_A_B.z()<<std::endl;

  tf_B_A.setOrigin(tf_v_B_A);
  tf_B_A.setRotation(tf_q_B_A);
  // A is parent frame in tf, string
  //B is child frame
  //B 指向 A
  br.sendTransform(tf::StampedTransform(tf_B_A
                                        ,ros::Time::now()
                                        ,"B"
                                        ,"A"));


  //A_A1
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


  //C_A
  Eigen::Quaterniond q_C_A;
  Eigen::Vector3d v_C_A;

  Eigen::Matrix3d R;

  q_C_A =   q_C_B * q_B_A;
  v_C_A =   v_C_B + q_C_B * v_B_A;

  //轉60度的matrix
  R = q_C_B.matrix();

  std::cout<<"v_B_A"<<v_B_A<<std::endl;
  std::cout<<"v_C_B"<<v_C_B<<std::endl;
  std::cout<<"q_B_C * v_B_A"<<q_C_B * v_B_A<<std::endl;
  std::cout<<"v_C_A"<<v_C_A<<std::endl;

  std::cout<<"R"<<R<<std::endl;

  tf::Transform tf_C_A;
  tf::Quaternion tf_q_C_A(q_C_A.x(),q_C_A.y(),q_C_A.z(),q_C_A.w());
  tf::Vector3 tf_v_C_A(v_C_A(0), v_C_A(1), v_C_A(2));


  tf_C_A.setOrigin(tf_v_C_A);
  tf_C_A.setRotation(tf_q_C_A);
  br.sendTransform(tf::StampedTransform(tf_C_A
                                        ,ros::Time::now()
                                        ,"C1"
                                        ,"A1"));


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
