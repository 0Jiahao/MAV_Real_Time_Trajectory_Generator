#include <iostream>
#include "ALGLIB/stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "ALGLIB/optimization.h"
#include <mymav.h>
#include <Eigen/Dense>

alglib::real_1d_array time_optimal_path_planner(mymav mav, mymav tgt)
{
    // gravity drag force
    double g = 9.80665;
    // clock start
    clock_t startTime = clock();
    // rotational matrix (from body to world)
    Eigen::MatrixXd R_b_w(3,3);
    Eigen::MatrixXd R_w_b(3,3);
    R_w_b(0,0) = cos(mav.angle(1))*cos(mav.angle(2)) ;
	R_w_b(0,1) = cos(mav.angle(1))*sin(mav.angle(2)) ;
	R_w_b(0,2) = -sin(mav.angle(1));
	R_w_b(1,0) = sin(mav.angle(0))*sin(mav.angle(1))*cos(mav.angle(2))-cos(mav.angle(0))*sin(mav.angle(2));
	R_w_b(1,1) = sin(mav.angle(0))*sin(mav.angle(1))*sin(mav.angle(2))+cos(mav.angle(0))*cos(mav.angle(2));
	R_w_b(1,2) = sin(mav.angle(0))*cos(mav.angle(1));
	R_w_b(2,0) = cos(mav.angle(0))*sin(mav.angle(1))*cos(mav.angle(2))+sin(mav.angle(0))*sin(mav.angle(2));
	R_w_b(2,1) = cos(mav.angle(0))*sin(mav.angle(1))*sin(mav.angle(2))-sin(mav.angle(0))*cos(mav.angle(2));
	R_w_b(2,2) = cos(mav.angle(0))*cos(mav.angle(1));
    R_b_w = R_w_b.transpose();
    // mav's speed in world frame
    Eigen::Vector3d speed_w;
    speed_w = R_b_w * mav.speed;
    // orientation of the target
    double ts = sin(tgt.angle(0)); // sin
    double tc = cos(tgt.angle(0)); // cos
    // construct the QP problem  
    alglib::real_2d_array H = "[[2,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
			                   "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"  
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,2,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"  
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"   
				               "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]";  
    alglib::real_1d_array f =  "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]";
    alglib::real_1d_array s =  "[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]";
    alglib::real_2d_array c;
    alglib::integer_1d_array ct = "[0,0,0,0,0,0,0,0,0,-1,0,0]"; // 0 is =; -1 is <=; 1 is >= 
    alglib::real_1d_array x;
    alglib::minqpstate state;
    alglib::minqpreport rep;
    // create solver, set quadratic/linear terms
    alglib::minqpcreate(15, state);
    alglib::minqpsetquadraticterm(state, H);
    alglib::minqpsetlinearterm(state, f);
    alglib::minqpsetscale(state, s);
    alglib::minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    alglib::real_1d_array x0 = "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]";
    alglib::minqpsetstartingpoint(state, x0);
    // print information of each iteration
    printf("\n========================================================================");
    printf("\nNr.\t\tPitch\t\t\tThrottle\t\tReport");
    for(int i = 0; i < 32; i = i + 1)
    {
        double t = exp(-2.5 + 0.175 * i);
        // construct the constraints
        double _c[] = {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,mav.position(0), // x of start point
                       0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,mav.position(1), // y of start point
                       0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,mav.position(2), // z of start point
                       pow(t,4),pow(t,3),pow(t,2),pow(t,1),1,0,0,0,0,0,0,0,0,0,0,tgt.position(0), // x of final point
                       0,0,0,0,0,pow(t,4),pow(t,3),pow(t,2),pow(t,1),1,0,0,0,0,0,tgt.position(1), // y of final point
                       0,0,0,0,0,0,0,0,0,0,pow(t,4),pow(t,3),pow(t,2),pow(t,1),1,tgt.position(2), // z of final point
                       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,speed_w(0), // vx of start point
                       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,speed_w(1), // vy of start point
                       0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,speed_w(2), // vz of start point
                       tc*4*pow(t,3),tc*3*pow(t,2),tc*2*pow(t,1),tc,0,ts*4*pow(t,3),ts*3*pow(t,2),ts*2*pow(t,1),ts,0,0,0,0,0,0,2,         // vx of final point (<= 2) can be tunned
                       -ts*4*pow(t,3),-ts*3*pow(t,2),-ts*2*pow(t,1),-ts,0,tc*4*pow(t,3),tc*3*pow(t,2),tc*2*pow(t,1),tc,0,0,0,0,0,0,0,     // vy of final point
                       0,0,0,0,0,0,0,0,0,0,4*pow(t,3),3*pow(t,2),2*pow(t,1),1,0,0};                                                       // vz of final point
        c.setcontent(12,16,_c);
        alglib::minqpsetlc(state, c, ct);
        // Solve problem with BLEIC-based QP solver.
        alglib::minqpoptimize(state);
        alglib::minqpresults(state, x, rep);
        // find the most aggressive trajectory
        double pitch = abs(atan2(sqrt(pow(2 * x(2),2) + pow(2 * x(7),2)), abs(2 * x(12) - g)));
        printf("\n#%i\t\t%+0.6f\t\t%+0.6f\t\t%d",i+1,pitch,sqrt(pow(2 * x(2),2) + pow(2 * x(7),2) + pow(2 * x(12) - g,2)),int(rep.terminationtype));     // display for iteration
        // unsatisfy the boundary or not?
        bool throttle_ub_sat = ((2 * x(12) - g < 0) && (sqrt(pow(2 * x(2),2) + pow(2 * x(7),2) + pow(2 * x(12) - g,2)) < 2 * g));
        bool throttle_lb_sat = ((2 * x(12) - g > 0) && (sqrt(pow(2 * x(2),2) + pow(2 * x(7),2) + pow(2 * x(12) - g,2)) < g));
        if(pitch <= M_PI/12 && ( throttle_lb_sat || throttle_ub_sat ))
        {
            printf("\n------------------------------------------------------------------------");
            printf("\nTime for arrival:\t%0.6f",t);
            // clock stop
            clock_t endTime = clock();
            clock_t clockTicksTaken = endTime - startTime;
            double timeInSeconds = clockTicksTaken / (double) CLOCKS_PER_SEC;
            printf("\nOptimizationTime:\t%0.6f",timeInSeconds);
            break;
        }
    }
    return x;
}

control_input minimum_snap_control(mymav mav, mymav tgt, alglib::real_1d_array x, double ts)
{
    // construct the control input
    control_input input;
    // gravity drag force
    double g = 9.80665;
    // desired acceleration
    double x_pp = 2 * x[2];
    double y_pp = 2 * x[7];
    double z_pp = 2 * x[12] - g;
    double norm = sqrt(pow(x_pp,2) + pow(y_pp,2) + pow(z_pp,2));
    // store throttle
    input.throttle = -norm;
    Eigen::Vector3d zb(x_pp,y_pp,z_pp);
    zb = - zb / norm;       
    // calculate the desired yaw angle
    double toward = atan2(tgt.position(1)-mav.position(1),tgt.position(0)-mav.position(0));
    double increment = toward - mav.angle(0);
    // normalize the increment yaw angle
    increment = fmod((increment + M_PI),(2 * M_PI)) - M_PI;
    if (abs(increment) >=  M_PI/(2/ts))   // assume rotational velocity < 90 degree per second
    {
        increment = abs(increment)/increment * 1/2 * M_PI / (1/ts);
    }
    double yaw = mav.angle(0) + increment;
    Eigen::Vector3d xc(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d yb;
    yb = zb.cross(xc);
    // vector normalization
    norm = sqrt(pow(yb(0),2) + pow(yb(1),2) + pow(yb(2),2));
    yb = yb / norm;
    Eigen::Vector3d xb;
    xb = yb.cross(zb);
    // generate the euler angle
    yaw = atan2(xb(1),xb(0));
    double pitch = atan2(-xb(2),sqrt(pow(yb(2),2)+pow(zb(2),2)));
    //normalize pitch
    pitch = fmod((pitch + M_PI),(2 * M_PI)) - M_PI;
    double roll = atan2(yb(2),zb(2));
    // normalize roll
    pitch = fmod((pitch + M_PI),(2 * M_PI)) - M_PI;
    // store yaw, pitch, roll
    input.yaw = yaw;
    input.pitch = pitch;
    input.roll = roll;
    // display the result
    printf("\n\tthrottle:\t%+0.6f",input.throttle);
    printf("\n\tyaw:\t\t%+0.6f",input.yaw);
    printf("\n\tpitch:\t\t%+0.6f",input.pitch);
    printf("\n\troll:\t\t%+0.6f",input.roll);
}   

int main()
{
	// create target
	mymav tgt;
	tgt.info();
	// create mav
	mymav mav(-16,0,-4.5,0,0,0,0,0,0);
	mav.info();
    // solve the optimal trajectory
    alglib::real_1d_array x;
    x = time_optimal_path_planner(mav,tgt);
    // compute the control input 
    control_input input;
    input = minimum_snap_control(mav, tgt, x, 0.05);
    std::cout << "\n[solution]\t";
    for(int i = 0; i < 15; i++)
    {
        printf("%+0.1f\t",x[i]);
    }
    return 0;
}
