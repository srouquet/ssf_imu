#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Dense>
#include "qualisys/Subject.h"



using namespace Eigen;

Vector4d q = Vector4d::Zero();
double dt=1.00/200.00;
Vector3d b_w;
Vector3d p;
Vector3d v;
MatrixXd P(13,13);
MatrixXd Q(9,9);
MatrixXd R(7,7);

void chatterCallback(const sensor_msgs::Imu msg_imu){
  //ROS_INFO("I heard: [%f]", msg_imu.linear_acceleration.x);
  Vector3d a,aw;
  a<<msg_imu.linear_acceleration.x,msg_imu.linear_acceleration.y,msg_imu.linear_acceleration.z;
  Vector3d omega_m;
  omega_m<<msg_imu.angular_velocity.x,msg_imu.angular_velocity.y,msg_imu.angular_velocity.z;


  //Prediction part
  MatrixXd Omega(4,3);
  Omega<<-q[1],-q[2],-q[3], q[0],-q[3],q[2], q[3],q[0],-q[1], -q[2],q[1],q[0];
  double norm_q;


  //Rotation matrix from inertial frame to body
  Matrix3d Rbe;
  Rbe<<pow(q[0],2)+pow(q[1],2)-pow(q[2],2)-pow(q[3],2),2*(q[1]*q[2]+q[0]*q[3]),2*(q[1]*q[3]-q[0]*q[2]),
  2*(q[1]*q[2]-q[0]*q[3]),pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2),2*(q[2]*q[3]+q[0]*q[1]),
  2*(q[1]*q[3]+q[0]*q[2]),2*(q[2]*q[3]-q[0]*q[1]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2);

//  std::cout << "Rbe \n"<<Rbe << std::endl;


  Vector3d g;
  g<<0,0,9.8314;

  //Prediction
  p=p+v*dt;
  v=v+(Rbe.transpose()*a+g)*dt;
  q=q+1.00/2.00*Omega*(omega_m-b_w)*dt;
  //b_w<<0,0,0;
  //Normalize Quaternion
  norm_q=pow(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2),0.5);
  q<<q[0]/norm_q,q[1]/norm_q,q[2]/norm_q,q[3]/norm_q;


  MatrixXd F(13,13);
  F<<0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 2*a[0]*q[0] - 2*a[2]*q[2] + 2*a[1]*q[3], 2*a[0]*q[1] + 2*a[1]*q[2] + 2*a[2]*q[3], -2*a[2]*q[0] + 2*a[1]*q[1] - 2*a[0]*q[2], 2*a[1]*q[0] + 2*a[2]*q[1] - 2*a[0]*q[3], 0, 0, 0,
  0, 0, 0, 0, 0, 0, 2*a[1]*q[0] + 2*a[2]*q[1] - 2*a[0]*q[3], 2*a[2]*q[0] - 2*a[1]*q[1] + 2*a[0]*q[2], 2*a[0]*q[1] + 2*a[1]*q[2] + 2*a[2]*q[3], -2*a[0]*q[0] + 2*a[2]*q[2] - 2*a[1]*q[3], 0, 0, 0,
  0, 0, 0, 0, 0, 0, 2*a[2]*q[0] - 2*a[1]*q[1] + 2*a[0]*q[2], -2*a[1]*q[0] - 2*a[2]*q[1] + 2*a[0]*q[3], 2*a[0]*q[0] - 2*a[2]*q[2] + 2*a[1]*q[3], 2*a[0]*q[1] + 2*a[1]*q[2] + 2*a[2]*q[3], 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, (b_w[0] - omega_m[0])/2.00, (b_w[1] - omega_m[1])/2.00, (b_w[2] - omega_m[2])/2.00, q[1]/2.00, q[2]/2.00, q[3]/2.00,
  0, 0, 0, 0, 0, 0, 1.00/2.00*(-b_w[0] + omega_m[0]), 0, 1.00/2.00*(-b_w[2] + omega_m[2]), (b_w[1] - omega_m[1])/2.00, -(q[0]/2.00), q[3]/2.00, -(q[2]/2.00),
  0, 0, 0, 0, 0, 0, 1.00/2.00*(-b_w[1] + omega_m[1]), (b_w[2] - omega_m[2])/2.00, 0, 1.00/2.00*(-b_w[0] + omega_m[0]), -(q[3]/2.00), -(q[0]/2.00), q[1]/2.00,
  0, 0, 0, 0, 0, 0, 1.00/2.00*(-b_w[2] + omega_m[2]), 1.00/2.00*(-b_w[1] + omega_m[1]), (b_w[0] - omega_m[0])/2.00, 0, q[2]/2.00, -(q[1]/2.00), -(q[0]/2.00),
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;


  MatrixXd G(13,9);
  G<<0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, pow(q[0],2) + pow(q[1],2) - pow(q[2],2) - pow(q[3],2), 2*(q[1]*q[2] + q[0]*q[3]), 2*(-q[0]*q[2] + q[1]*q[3]), 0, 0, 0,
  0, 0, 0, 2*(q[1]*q[2] - q[0]*q[3]), pow(q[0],2) - pow(q[1],2) + pow(q[2],2) - pow(q[3],2), 2*(q[0]*q[1] + q[2]*q[3]), 0, 0, 0,
  0, 0, 0, 2*(q[0]*q[2] + q[1]*q[3]), 2*(-q[0]*q[1] + q[2]*q[3]), pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2), 0, 0, 0,
  -(q[1]/2.00), -(q[2]/2.00), -(q[3]/2.00), 0, 0, 0, 0, 0, 0,
  q[0]/2.00, -(q[3]/2.00), q[2]/2.00, 0, 0, 0, 0, 0, 0,
  q[3]/2.00, q[0]/2.00, -(q[1]/2.00), 0, 0, 0, 0, 0, 0,
  -(q[2]/2.00), q[1]/2.00, q[0]/2.00, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 1;

  P=(MatrixXd::Identity(13,13)+F*dt)*P*(MatrixXd::Identity(13,13)+F*dt).transpose()+pow(dt,2)*G*Q*G.transpose();
  std::cout << "positions\n"<<p << std::endl;
  std::cout << "orientation\n"<<q << std::endl;
}


void mocapCallback(qualisys::Subject mocap_msg){
  //Update part
  MatrixXd H(7,13);

  H<<1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;




  VectorXd error(7);
  error<<p[0]-mocap_msg.position.x,
         p[1]-mocap_msg.position.y,
         p[2]-mocap_msg.position.z,
         q[0]-mocap_msg.orientation.w,
         q[1]-mocap_msg.orientation.x,
         q[2]-mocap_msg.orientation.y,
         q[3]-mocap_msg.orientation.z;

  MatrixXd K(13,7);
  K=P*H.transpose()*(H*P*H.transpose()+R).inverse();


  //Update
  VectorXd x(13);
  x<<p[0],p[1],p[2],v[0],v[1],v[2],q[0],q[1],q[2],q[3],b_w[0],b_w[1],b_w[2];
  x=x-K*error;


  p<<x[0],x[1],x[2];
  v<<x[3],x[4],x[5];
  q<<x[6],x[7],x[8],x[9];
  b_w<<x[10],x[11],x[12];

  P=P-K*H*P;

  double norm_q;
  //Normalize Quaternion
  norm_q=pow(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2),0.5);
  q<<q[0]/norm_q,q[1]/norm_q,q[2]/norm_q,q[3]/norm_q;
  //std::cout << "q \n " << q << std::endl;
  //std::cout << "\n " << b_w << std::endl;
  //std::cout << "\n " << P << std::endl;

  Vector3d EulerAngles;

  EulerAngles[0] = -atan2(2*(q[2]*q[3]-q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
  EulerAngles[1] = asin(2*(q[1]*q[3]+q[0]*q[2]));
  EulerAngles[2] = -atan2(2*(q[1]*q[2]-q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);

  std::cout << "Euler \n " << EulerAngles << std::endl;

  std::cout << "orientation\n"<<q << std::endl;

  std::cout << "speeds\n"<<v << std::endl;
  std::cout << "positions\n"<<p << std::endl;

//  std::cout << "P\n" <<P<< std::endl;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ssf_imu_position_node");
  ros::NodeHandle n;
  b_w<<0,0,0;
  q<<1,0,0,0;
  p<<0,0,0;
  v<<0,0,0;
  // Covariance on the state estimator
  P<<MatrixXd::Identity(13,13)*0.1;
  // Covariance gyro and gyro biais
  Q<<MatrixXd::Identity(9,9)*0.001;
  // Coariance position and the angle of the mocap system
  R<<0.001,0,0,0,0,0,0,
     0,0.001,0,0,0,0,0,
     0,0,0.001,0,0,0,0,
     0,0,0,0.1,0,0,0,
     0,0,0,0,0.1,0,0,
     0,0,0,0,0,0.1,0,
     0,0,0,0,0,0,0.1;

  ros::Subscriber sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Subscriber sub1 = n.subscribe("qualisys/imu", 1000, mocapCallback);
  ros::spin();

  return 0;
}
