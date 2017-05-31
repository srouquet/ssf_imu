#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Dense>


using namespace Eigen;

Vector4d q = Vector4d::Zero();
double dt=1.00/200.00;
Vector3d b_w;
MatrixXd P(7,7);
MatrixXd Q(6,6);
MatrixXd R(4,4);

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


  //Prediction
  q=q+1.00/2.00*Omega*(omega_m-b_w)*dt;
  //b_w<<0,0,0;
  //Normalize Quaternion
  norm_q=pow(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2),0.5);
  q<<q[0]/norm_q,q[1]/norm_q,q[2]/norm_q,q[3]/norm_q;


  MatrixXd F(7,7);
  F<<0, 1.00/2.00*(b_w[0] - omega_m[0]), 1.00/2.00*(b_w[1] - omega_m[1]), 1.00/2.00*(b_w[2] - omega_m[2]), q[1]/2.00, q[2]/2.00, q[3]/2.00,
  1.00/2.00*(-b_w[0] + omega_m[0]), 0, 1.00/2.00*(-b_w[2] + omega_m[3]), 1.00/2.00*(b_w[1] - omega_m[1]), -(q[0]/2.00), q[3]/2.00, -(q[2]/2.00),
  1.00/2.00*(-b_w[1] + omega_m[1]), 1.00/2.00*(b_w[2] - omega_m[3]), 0, 1.00/2.00*(-b_w[0] + omega_m[0]), -(q[3]/2.00), -(q[0]/2.00), q[1]/2.00,
  1.00/2.00*(-b_w[2] + omega_m[3]), 1.00/2.00*(-b_w[1] + omega_m[1]), 1.00/2.00*(b_w[0] - omega_m[0]), 0, q[2]/2.00, -(q[1]/2.00), -(q[0]/2.00),
  0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0;

  MatrixXd G(7,6);
  G<<-(q[1]/2.00), -(q[2]/2.00), -(q[3]/2.00),0,0,0,
  q[0]/2.00, -(q[3]/2.00), q[2]/2.00,0,0,0,
  q[3]/2.00, q[0]/2.00, -(q[1]/2.00),0,0,0,
  -(q[2]/2.00), q[1]/2.00, q[0]/2.00,0,0,0,
  0, 0, 0,1,0,0,
  0, 0, 0,0,1,0,
  0, 0, 0,0,0,1;

  P=(MatrixXd::Identity(7,7)+F*dt)*P*(MatrixXd::Identity(7,7)+F*dt).transpose()+pow(dt,2)*G*Q*G.transpose();



  //Update part
  MatrixXd H(4,7);
  H<<1, 0, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0, 0,
  0, 0, 0, 1, 0, 0, 0;

  //Calculate the angle by the accelerometer

  Vector4d q2;
  //q2<<1,0,0,0;
  q2=q;
  //Rotation matrix from body to inertial frame
  Matrix3d Rbe;
  Rbe<<pow(q2[0],2)+pow(q2[1],2)-pow(q2[2],2)-pow(q2[3],2),2*(q2[1]*q2[2]+q2[0]*q2[3]),2*(q2[1]*q2[3]-q2[0]*q2[2]),
       2*(q2[1]*q2[2]-q2[0]*q2[3]),pow(q2[0],2)-pow(q2[1],2)+pow(q2[2],2)-pow(q2[3],2),2*(q2[2]*q2[3]+q2[0]*q2[1]),
       2*(q2[1]*q2[3]+q2[0]*q2[2]),2*(q2[2]*q2[3]-q2[0]*q2[1]),pow(q2[0],2)-pow(q2[1],2)-pow(q2[2],2)+pow(q2[3],2);







  double norm_a;
  norm_a=pow(pow(a[0],2)+pow(a[1],2)+pow(a[2],2),0.5);
  a<<a[0]/norm_a,a[1]/norm_a,a[2]/norm_a;

  aw=Rbe.inverse()*a;
  //aw=Rbe*a;

  std::cout << "aaaa \n"<<aw << std::endl;

  // Cross product with the gravity
  Vector3d Caw;
  Caw<<-aw[1],aw[0],0;
  //Caw<<aw[1],-aw[0],0;

  Vector3d Ca;
  Ca=Rbe*Caw;
  //Ca=Rbe.transpose()*Caw;

  Vector4d q_corr;

  q_corr<<q2[0],q2[1]-Ca[0],q2[2]-Ca[1],q2[3]-Ca[2];

//pow(1-(pow(q[1]-Ca[0],2)+pow(q[2]-Ca[1],2)+pow(q[3]-Ca[2],2)),0.5)

  norm_q=pow(pow(q_corr[0],2)+pow(q_corr[1],2)+pow(q_corr[2],2)+pow(q_corr[3],2),0.5);
  q_corr<<q_corr[0]/norm_q,q_corr[1]/norm_q,q_corr[2]/norm_q,q_corr[3]/norm_q;
  //std::cout << "q_corr \n " << q_corr << std::endl;

  MatrixXd K(7,4);
  K=P*H.transpose()*(H*P*H.transpose()+R).inverse();


  //Update
  VectorXd x(7);
  x<<q[0],q[1],q[2],q[3],b_w[0],b_w[1],b_w[2];
  x=x+K*(q-q_corr);

  q<<x[0],x[1],x[2],x[3];
  b_w<<x[4],x[5],x[6];

  P=P-K*H*P;

  //Normalize Quaternion
  norm_q=pow(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2),0.5);
  q<<q[0]/norm_q,q[1]/norm_q,q[2]/norm_q,q[3]/norm_q;
  std::cout << "q \n " << q << std::endl;
//  std::cout << "\n " << b_w << std::endl;
  //std::cout << "\n " << P << std::endl;

  Vector3d EulerAngles;

  EulerAngles[0] = -atan2(2*(q[2]*q[3]-q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
  EulerAngles[1] = asin(2*(q[1]*q[3]+q[0]*q[2]));
  EulerAngles[2] = -atan2(2*(q[1]*q[2]-q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);

  std::cout << "Euler \n " << EulerAngles << std::endl;

}

int main(int argc, char **argv){
  ros::init(argc, argv, "ssf_imu_node");
  ros::NodeHandle n;
  b_w<<0,0,0;
  q<<1,0,0,0;
  // Covariance on the state estimator
  P<<MatrixXd::Identity(7,7)*0.1;
  // Covariance gyro and gyro biais
  Q<<MatrixXd::Identity(6,6)*0.001;
  // Coariance angle of the accelerometer
  R<<MatrixXd::Identity(4,4)*10.0;

  ros::Subscriber sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::spin();

  return 0;
}
