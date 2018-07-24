#ifndef MYMAV_H
#define MYMAV_H

#include <Eigen/Dense>

class mymav
{
public:
	// components
	Eigen::Vector3d position;	// x, y, z
	Eigen::Vector3d angle;		// yaw, pitch, roll
	Eigen::Vector3d speed;		// in body frame

	// constructor
	mymav();
	mymav(float x, float y, float z, float yaw, float pitch, float roll, float vx, float vy, float vz);

	// print info
	void info();
};

class control_input
{
public:
	double throttle;
	double yaw;
	double pitch;
	double roll;
};

#endif
