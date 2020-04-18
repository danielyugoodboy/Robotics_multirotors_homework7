#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;

  // C -> B
  Eigen::Quaterniond q_C_B(std::sqrt(3)/2, 0, 0, 0.5);
  Eigen::Vector3d v_C_B(1, 0, 0);
  tf::Transform tf_C_B;
  tf::Quaternion tf_q_C_B(q_C_B.x(), q_C_B.y(), q_C_B.z(), q_C_B.w());
  tf::Vector3 tf_v_C_B(v_C_B(0), v_C_B(1), v_C_B(2));
  tf_C_B.setOrigin(tf_v_C_B);
  tf_C_B.setRotation(tf_q_C_B);

  // B -> A
  Eigen::Quaterniond q_B_A(std::sqrt(2)/2, 0, 0, std::sqrt(2)/2);
  Eigen::Vector3d v_B_A(1, 0, 0);
  tf::Transform tf_B_A;
  tf::Quaternion tf_q_B_A(q_B_A.x(), q_B_A.y(), q_B_A.z(), q_B_A.w());
  tf::Vector3 tf_v_B_A(v_B_A(0), v_B_A(1), v_B_A(2));
  tf_B_A.setOrigin(tf_v_B_A);
  tf_B_A.setRotation(tf_q_B_A);

  // A -> A1
  tf::Transform tf_A_A1;
  tf::Quaternion tf_q_A_A1(0, 0, 0, 1);
  tf::Vector3 tf_v_A_A1(0, 0, 0);
  tf_A_A1.setOrigin(tf_v_A_A1);
  tf_A_A1.setRotation(tf_q_A_A1);

  // C -> C1
  tf::Transform tf_C_C1;
  tf::Quaternion tf_q_C_C1(0, 0, 0, 1);
  tf::Vector3 tf_v_C_C1(0, 0, 0);
  tf_C_C1.setOrigin(tf_v_C_C1);
  tf_C_C1.setRotation(tf_q_C_C1);

  // C -> A
  tf::Transform tf_C_A;
  tf_C_A = tf_C_B*tf_B_A;
  tf::Transform tf_A_C;
  tf_A_C = tf_C_A.inverse();


  br.sendTransform(tf::StampedTransform(tf_C_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "C", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_B_A, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "B", // paranet frame ID
                                        "A")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_A_A1, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "A1")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_C_C1, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "C", // paranet frame ID
                                        "C1")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_C_A, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "C1", // paranet frame ID
                                        "A1")); // child frame ID

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "answer1");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
