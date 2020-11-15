/*

 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include <math.h>
#include <vector>

#include <sstream>

typedef struct DroneInternalState
{
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
  double vx_innertial;
  double vy_innertial;
  double vz_innertial;  
}DroneInternalState;

DroneInternalState droneInternalState;


typedef struct CurrentDestPoint
{
  double x;
  double y;
  double z;
  double psic;  
}CurrentDestPoint;

CurrentDestPoint currentDestPoint;

#define distanceErrorTH (0.1)
#define yawErrorTH (0.05)
#define DRONE_MASS (1.56)
#define ACC_G (9.806)

#define KpYaw (0.35)

//geometry_msgs::Point currentDestPoint;
std::vector<geometry_msgs::PoseStamped> currentPath;
mav_msgs::RollPitchYawrateThrust control;
// %Tag(PUBLISHER)%
ros::Publisher control_pub;
// %EndTag(PUBLISHER)%


double calcDistance(double x, double y, double z, double xp, double yp, double zp)
{
  double distance = sqrt( powf(x - xp, 2) + powf(y - yp, 2) + powf(z - zp, 2));
  return distance;
}



// %Tag(CALLBACK)%
void guideCallback(const nav_msgs::Odometry::ConstPtr& droneOdometry)
{
  
  droneInternalState.x = droneOdometry->pose.pose.position.x;
  droneInternalState.y = droneOdometry->pose.pose.position.y;
  droneInternalState.z = droneOdometry->pose.pose.position.z;

  droneInternalState.vx = droneOdometry->twist.twist.linear.x;
  droneInternalState.vy = droneOdometry->twist.twist.linear.y;
  droneInternalState.vz = droneOdometry->twist.twist.linear.z;

  geometry_msgs::Quaternion droneOrientation = droneOdometry->pose.pose.orientation;
  double psi = atan2(2*droneOrientation.w * droneOrientation.z, 1 - 2*droneOrientation.z * droneOrientation.z);


  droneInternalState.vx_innertial = cos(psi)*droneInternalState.vx - sin(psi)*droneInternalState.vy;
  droneInternalState.vy_innertial = sin(psi)*droneInternalState.vx + cos(psi)*droneInternalState.vy;
  droneInternalState.vz_innertial = droneInternalState.vz;

  double xc = currentDestPoint.x;
  double yc = currentDestPoint.y;
  double zc = currentDestPoint.z;
  double psic = currentDestPoint.psic;

  ROS_WARN("Position from odo is: [%f %f %f %f]", droneInternalState.x, droneInternalState.y, droneInternalState.z, psi);
  ROS_WARN("Current dest point is: [%f %f %f, %f]", xc, yc, zc, psic);

  double distance = calcDistance(droneInternalState.x, droneInternalState.y, droneInternalState.z, xc, yc, zc);
  ROS_WARN("Distance is: [%f]", distance);



  double errorPsi = psi - psic;
  if(errorPsi > M_PI)
  {
    errorPsi = errorPsi - 2*M_PI;
  }
  else if(errorPsi < -M_PI)
  {
    errorPsi = errorPsi + 2*M_PI;
  }


  if( (distance > distanceErrorTH) || (errorPsi >yawErrorTH)  )
  {  /*update control for roll pitch and thrust*/

    double ux = xc - droneInternalState.x - 2*droneInternalState.vx_innertial;  
    double uy = yc - droneInternalState.y - 2*droneInternalState.vy_innertial;
    double uz = zc - droneInternalState.z - 2*droneInternalState.vz_innertial;

    double thrust = DRONE_MASS*(uz + ACC_G);
    double theta = DRONE_MASS*(ux*(1/tan(psi)) + uy)/( ( (1/tan(psi))*cos(psi) + sin(psi) ) *thrust);
    double phi = (theta*sin(psi) - DRONE_MASS*uy/thrust )/cos(psi);

    control.roll = phi;
    control.pitch = theta;
    control.thrust.z = thrust;

    /*update control for yaw*/
    control.yaw_rate = -KpYaw*errorPsi;

    ROS_WARN("send control");
    ROS_WARN("Control is: [%f %f %f %f]", control.roll, control.pitch, control.thrust.z, control.yaw_rate);

    control_pub.publish(control);

  }

}



// %Tag(CALLBACK)%
//void updtTargetPointCallback(const geometry_msgs::Point::ConstPtr& newPoint)
void updtTargetPointCallback(const geometry_msgs::PoseStamped::ConstPtr& newPose) 
//void updtTargetPointCallback(const CurrentDestPoint::ConstPtr& newPoint)
{
  ROS_WARN("New target point is: [%f %f %f %f %f %f %f]", newPose->pose.position.x, newPose->pose.position.y, newPose->pose.position.z, newPose->pose.orientation.x, newPose->pose.orientation.y, newPose->pose.orientation.z, newPose->pose.orientation.w);


  currentDestPoint.x = newPose->pose.position.x;
  currentDestPoint.y = newPose->pose.position.y;
  currentDestPoint.z = newPose->pose.position.z;
  geometry_msgs::Quaternion droneOrientation = newPose->pose.orientation;
  currentDestPoint.psic = atan2 (2*droneOrientation.w * droneOrientation.z, 1 - 2*droneOrientation.z * droneOrientation.z);


}
// %EndTag(CALLBACK)%



// %Tag(CALLBACK)%
void updtPathCallback(const nav_msgs::Path::ConstPtr& newPath)
{

  //int pointsNbr = sizeof(newPath->poses)/sizeof(newPath->poses[0]); 

  int pointsNbr = newPath->poses.size(); 

  ROS_WARN("New target path [%d] points", pointsNbr );

  currentPath.clear();

  for(int i=0; i< pointsNbr; i++ )
  {
    currentPath.push_back(newPath->poses[i]);
  }

  currentDestPoint.x = currentPath[0].pose.position.x;
  currentDestPoint.y = currentPath[0].pose.position.y;
  currentDestPoint.z = currentPath[0].pose.position.z;
  geometry_msgs::Quaternion droneOrientation = currentPath[0].pose.orientation;
  currentDestPoint.psic = atan2 (2*droneOrientation.w * droneOrientation.z, 1 - 2*droneOrientation.z * droneOrientation.z);

}
// %EndTag(CALLBACK)%


int main(int argc, char **argv)
{

// %Tag(INIT)%
  ros::init(argc, argv, "drone_talker");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%


   /*Get parameters*/   
   //geometry_msgs::Twist control;
   //ros::param::get("~vel",control.linear.x);


// %Tag(PUBLISHER)%
  //vel_pub = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
  control_pub = n.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", 1000);
// %EndTag(PUBLISHER)%


// %Tag(SUBSCRIBER)%
  /*Husky pose*/
  ros::Subscriber sub = n.subscribe("/firefly/odometry_sensor1/odometry", 10000, guideCallback);
  /*Husky target destination*/
  ros::Subscriber subTarget = n.subscribe("/targetPoint/droneDest", 10000, updtTargetPointCallback);
  /*Husky path*/
  ros::Subscriber subPath = n.subscribe("/targetPath/dronePath", 10000, updtPathCallback);
// %EndTag(SUBSCRIBER)%



// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%


  int pathStep =0;
// %Tag(ROS_OK)%
  while (ros::ok())
  {
// %EndTag(ROS_OK)%


  if(calcDistance(droneInternalState.x, droneInternalState.y, droneInternalState.z, currentDestPoint.x, currentDestPoint.y, currentDestPoint.z) < distanceErrorTH)
  {    

    if( pathStep + 1 < currentPath.size() )
    {
      geometry_msgs::Pose tmpPose = currentPath[pathStep+1].pose; 
      currentDestPoint.x = tmpPose.position.x;
      currentDestPoint.y = tmpPose.position.y;
      currentDestPoint.z = tmpPose.position.z;
      geometry_msgs::Quaternion droneOrientation = tmpPose.orientation;
      currentDestPoint.psic = atan2 (2*droneOrientation.w * droneOrientation.z, 1 - 2*droneOrientation.z * droneOrientation.z);
    }
    else
    {
      pathStep=0;
    }
  }

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  }


  return 0;
}
// %EndTag(FULLTEXT)%

