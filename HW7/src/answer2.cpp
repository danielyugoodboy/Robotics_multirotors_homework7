#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;

  // A-> B
  Eigen::Quaterniond q_A_B(1, 0, 0, 0);
  Eigen::Vector3d v_A_B(-1, 0, 0);
  tf::Transform tf_A_B;
  tf::Quaternion tf_q_A_B(q_A_B.x(), q_A_B.y(), q_A_B.z(), q_A_B.w());
  tf::Vector3 tf_v_A_B(v_A_B(0), v_A_B(1), v_A_B(2));
  tf_A_B.setOrigin(tf_v_A_B);
  tf_A_B.setRotation(tf_q_A_B);

  // A -> C
  Eigen::Quaterniond q_A_C(1, 0, 0, 0);
  Eigen::Vector3d v_A_C(0, -1, 0);
  tf::Transform tf_A_C;
  tf::Quaternion tf_q_A_C(q_A_C.x(), q_A_C.y(), q_A_C.z(), q_A_C.w());
  tf::Vector3 tf_v_A_C(v_A_C(0), v_A_C(1), v_A_C(2));
  tf_A_C.setOrigin(tf_v_A_C);
  tf_A_C.setRotation(tf_q_A_C);

  // B -> B1
  tf::Transform tf_B_B1;
  tf::Quaternion tf_q_B_B1(0, 0, 0, 1);
  tf::Vector3 tf_v_B_B1(0, 0, 0);
  tf_B_B1.setOrigin(tf_v_B_B1);
  tf_B_B1.setRotation(tf_q_B_B1);

  // C -> C1
  tf::Transform tf_C_C1;
  tf::Quaternion tf_q_C_C1(0, 0, 0, 1);
  tf::Vector3 tf_v_C_C1(0, 0, 0);
  tf_C_C1.setOrigin(tf_v_C_C1);
  tf_C_C1.setRotation(tf_q_C_C1);


  // C -> B
  tf::Transform tf_C_B;
  tf_C_B = (tf_A_C.inverse())*tf_A_B;


  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "B")); // child frame ID
  br.sendTransform(tf::StampedTransform(tf_A_C, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "C")); // child frame ID
  br.sendTransform(tf::StampedTransform(tf_B_B1, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "B", // paranet frame ID
                                        "B1")); // child frame ID
  br.sendTransform(tf::StampedTransform(tf_C_C1, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "C", // paranet frame ID
                                        "C1")); // child frame ID
  br.sendTransform(tf::StampedTransform(tf_C_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "C1", // paranet frame ID
                                        "B1")); // child frame ID

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "answer2");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
