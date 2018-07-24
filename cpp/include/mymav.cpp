#include <iostream>
#include <mymav.h>
#include <Eigen/Dense>

mymav::mymav()
{
	position << 0,0,0;
	angle    << 0,0,0;
	speed    << 0,0,0;
}

mymav::mymav(float x, float y, float z, float yaw, float pitch, float roll, float vx, float vy, float vz)
{
	position << x,y,z;
	angle    << yaw,pitch,roll;
	speed    << vx,vy,vz;
}

void mymav::info()
{
	printf("\nObject's information:");
	printf("\n\tposition\t%+0.3f\t%+0.3f\t%+0.3f",position(0),position(1),position(2));
	printf("\n\tangle\t\t%+0.3f\t%+0.3f\t%+0.3f",angle(0),angle(1),angle(2));
	printf("\n\tspeed\t\t%+0.3f\t%+0.3f\t%+0.3f",speed(0),speed(1),speed(2));
}
