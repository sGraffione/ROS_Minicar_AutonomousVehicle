#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/Motors.h"
#include "minicar/BtsData.h"
#include "minicar/accel.h"
#include "minicar/EKFstate.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <casadi/casadi.hpp>

#define MAX_SPEED 0.3
#define MAX_SPEED_RATE 0.05 // tuned by hand
#define MAX_DUTY_CYCLE 4000
#define MAX_DELTA 0.523599
#define MAX_DELTA_RATE 0.1
#define GAMMA_STEER 30/30
//LQT horizon
#define T 5
#define T_LESS_1 4
#define Np 15
#define Nc 5

using namespace casadi;

int leftMotor = 23;
int dirLeftMotor = 24;
float Ts = 0.1;
float position[3];
float roll = 0, pitch = 0, yaw = 0;
//double MAX_DELTA_RATE = (M_PI/3*Ts)/0.17; // computation based on datasheet of MG996R servo motor

double L = 0.14;

void stateCallback(const minicar::EKFstate& msg){
	position[0] = msg.state[0];
	position[1] = msg.state[1];
	yaw = msg.state[2];
	//ROS_INFO("(%f %f) - %f", position[0], position[1], yaw);
}
void gazeboPositionCallback(const nav_msgs::Odometry& msg){
	
	position[0] = msg.pose.pose.position.x;
	position[1] = msg.pose.pose.position.y;
	position[2] = msg.pose.pose.position.z;
    double x = msg.pose.pose.orientation.x;
	double y = msg.pose.pose.orientation.y;
	double z = msg.pose.pose.orientation.z;
	double w = msg.pose.pose.orientation.w;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "controller");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	ros::Publisher current_pub = n.advertise<minicar::Motors>("motorsCtrl",1);
	//ros::Publisher current_pub_drive = n.advertise<geometry_msgs::Twist>("/minicar_driving_controller/cmd_vel",1);
	//ros::Publisher current_pub_steer = n.advertise<std_msgs::Float64MultiArray>("/minicar_steer_controller/command",1);
	
	ros::Subscriber sub = n.subscribe("EKFstate",1,stateCallback);
	//ros::Subscriber sub = n.subscribe("/localization/state",1,gazeboPositionCallback);
	
	ros::Rate loop_rate(1/Ts);
	
	double lr = 0.07;
	double lf = 0.07;
	double width = 0.11;

	double q1,q2,q3,q4,q5,r1,r2;
	n.getParam("/controller/q1", q1);
	n.getParam("/controller/q2", q2);
	n.getParam("/controller/q3", q3);
	n.getParam("/controller/q2", q4);
	n.getParam("/controller/q3", q5);
	n.getParam("/controller/r1", r1);
	n.getParam("/controller/r2", r2);
	
	double X_init = 0.6;
	double Y_init = 0.0;
	double theta_init = M_PI_2;

	MX X = MX::sym("X", 1);
	MX Y = MX::sym("Y", 1);
	MX theta = MX::sym("theta", 1);
	MX V = MX::sym("V", 1);
	MX delta = MX::sym("delta", 1); 

	MX V_rate = MX::sym("V_rate", 1);
	MX delta_rate = MX::sym("delta_rate", 1); 

	MX state = vertcat(std::vector<MX>{X,Y,theta,V,delta});
	MX controls = vertcat(V_rate, delta_rate);

	// Number of differential states
	int nx = state.size1();

	// Number of controls
	int nu = controls.size1();

	// Bounds and initial guess for the control
	std::vector<double> u_min =  { -MAX_SPEED_RATE, -MAX_DELTA_RATE };
	std::vector<double> u_max  = {  MAX_SPEED_RATE, MAX_DELTA_RATE };
	std::vector<double> u_init = {  0.0,  0.0  };

	// Bounds and initial guess for the state
	std::vector<double> x_min  = { -inf, -inf, -inf, -MAX_SPEED, -MAX_DELTA };
	std::vector<double> x_max  = {  inf,  inf,  inf,  MAX_SPEED,  MAX_DELTA };
	std::vector<double> x_init = { position[0], position[1], M_PI_2, 0.0, 0.0 }; // to get from localization topic

	// Non linear, time continuous equations of the system
	MX beta = atan((lr*tan(delta*GAMMA_STEER))/(lr+lr));
	MX rhs = vertcat(std::vector<MX>{X+Ts*V*cos(theta+beta),
									Y+Ts*V*sin(theta+beta),
									theta+Ts*V*(tan(delta*GAMMA_STEER)*cos(beta))/(lr+lf),
									V+V_rate,
									delta+delta_rate});
	
	MXDict dae = {{"x",state},{"p",controls},{"ode",rhs}}; // build a dictionary containing state, control and equations vector

	Function f = Function("f",{state,controls},{rhs});
	
	MX P = MX::sym("P",nx+nx,1);

	MX states = MX::sym("state",nx,Np+1); // state matrix over the prediction horizon
	MX U = MX::sym("U",nu,Np); // control vector over the prediction horizon
	
	// varVect has a length equal to the total number of variable for the NLP
	int NV = nx*(Np+1)+nu*Np;
	std::vector<double> v_min,v_max,v_init;

	MX varVect = MX::sym("varVect",NV);

	// Construction of symbolic cost function
	MXVector st, con;

	// Offset in varVect
	int offset = 0;

	for(int i = 0; i < Np; i++){
		st.push_back(varVect.nz(Slice(offset,offset+nx)));
		v_min.insert(v_min.end(),x_min.begin(),x_min.end());
		v_max.insert(v_max.end(),x_max.begin(),x_max.end());
		v_init.insert(v_init.end(), x_init.begin(), x_init.end());
		offset += nx;

		con.push_back(varVect.nz(Slice(offset,offset+nu)));
		v_min.insert(v_min.end(),u_min.begin(),u_min.end());
		v_max.insert(v_max.end(),u_max.begin(),u_max.end());
		v_init.insert(v_init.end(), u_init.begin(), u_init.end());
		offset += nu;
	}
	st.push_back(varVect.nz(Slice(offset,offset+nx)));
	v_min.insert(v_min.end(), x_min.begin(), x_min.end());
	v_max.insert(v_max.end(), x_max.begin(), x_max.end());
	v_init.insert(v_init.end(), x_init.begin(), x_init.end());
	offset += nx;

	// verify that offset has reached the end of varVect
	casadi_assert(offset==NV,"offset!=NV");

	MX obj = 0; // Objective function
	MXVector g; // constraints vector and bounds
	g.push_back(st[0]-P.nz(Slice(0,nx)));

	// Loop over shooting nodes
	for(int k=0; k<Np; ++k){
	// Create an evaluation node
		MXDict I_out = f(MXDict{{"i0", st[k]}, {"i1", con[k]}});

		// Add objective function contribution
		obj += q1*pow(st[k](0)-P(nx),2) + q2*pow(st[k](1)-P(nx+1),2) + q3*pow(st[k](2)-P(nx+2),2) + q4*pow(st[k](3)-P(nx+3),2) + q5*pow(st[k](4)-P(nx+4),2) + r1*pow(con[k](0),2) + r2*pow(con[k](1),2);

		// Save continuity constraints
		g.push_back( st[k+1] - I_out.at("o0") );
	}
	
	// NLP
	MXDict nlp = {{"x", varVect}, {"f", obj}, {"g", vertcat(g)},{"p",P}};

	// Set options
	Dict opts;
	opts["ipopt.tol"] = 1e-2;
	opts["ipopt.max_iter"] = 50;

	// To turn off all outputs uncomment this piece of options
	//====================================//
	opts["ipopt.print_level"] = 0; // default 5 (0-12)
	opts["print_time"] = 0;
	opts["ipopt.sb"] = "yes";
	//====================================//
	// Create an NLP solver and buffers
	Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);
	std::map<std::string, DM> arg, res;

	// Bounds and initial guess
	arg["lbx"] = v_min;
	arg["ubx"] = v_max;
	arg["lbg"] = 0;
	arg["ubg"] = 0;

	
	// Vettore di waypoints:
	// numero di waypoints
	int rows = 3; 
	// number of data on the waypoint (x,y,yaw,speed) 
	int cols = 4; 

	double waypoints[rows][cols] = {{0.6, 2.4, M_PI_2, 0.2},
					{0.8, 4.8, M_PI/4,    0.2},
					{2.0, 4.8, 0.0,    0.0}};
	int indexWP = 0;
	
	ROS_INFO("Target (%f %f)",waypoints[indexWP][0],waypoints[indexWP][1]);

	//std_msgs::Float64MultiArray cmdS;
	//geometry_msgs::Twist cmdV;
	minicar::Motors motorsCtrl;
			
	double delta_opt = 0.0, Vel_opt = 0.0, delta_rate_opt = 0.0, Vel_rate_opt = 0.0;

	// Sleep for 5 second before starting. It gives time to gazebo to open.
	for (int i = 0; i < 10; i++){
		ROS_INFO("Starting in %i",10-i);
		ros::Duration(1).sleep();
	}

	std::vector<double> P_nlp;

	while(ros::ok()){
		
		ros::spinOnce();
		
		arg["x0"] = v_init;

		P_nlp = { position[0], position[1], yaw, Vel_opt, delta_opt, waypoints[indexWP][0], waypoints[indexWP][1], waypoints[indexWP][2], waypoints[indexWP][3], 0.0 };
		arg["p"] = P_nlp;

		// Solve the problem
		res = solver(arg);

		// Optimal solution of the NLP
		std::vector<double> V_opt(res.at("x"));
		// Get the optimal state trajectory
		std::vector<double> v_init;
		for(int i=1; i<=Np; ++i){
			v_init.insert(v_init.end(), V_opt.begin()+(i*(nx+nu)), V_opt.begin()+(nx+i*(nx+nu)));
		}
		v_init.insert(v_init.end(), V_opt.begin()+(Np*(nx+nu)), V_opt.begin()+(nx+Np*(nx+nu)));

		//v_init.at(10*(nx+nu))

		Vel_opt = v_init.at(3);
		delta_opt = v_init.at(4);

		// Get the optimal control
		Vel_rate_opt = V_opt.at(nx);
		delta_rate_opt = V_opt.at(nx+1);
		
		//ROS_INFO("state: [%f %f %f]\n\t\t\t\t position: [%f %f]",Vel_opt,delta_opt,yaw,position[0],position[1]);
		//ROS_INFO("Optimal control [%f %f]",Vel_rate_opt,delta_rate_opt);

		// Compute left and right steer angle according to an ackermann steering system to have a more accurate simulation
		// TO COMMENT OUT WHEN USED ON REAL MINICAR
		//ROS_INFO("[%f %f]", Vel_opt, delta_opt);
		/*
		double R = (lr+lf)/tan(delta_opt);
		double left_delta, right_delta;
		left_delta = atan((lr+lf)/(R-width/2));
		right_delta = atan((lr+lf)/(R+width/2));
		//ROS_INFO("front steering [%f %f]",left_delta,right_delta);

		cmdS.data= {left_delta, right_delta};
		cmdV.linear.x = Vel_opt;
		cmdV.linear.y = 0;
		cmdV.linear.z = 0;
		cmdV.angular.x = 0;
		cmdV.angular.y = 0;
		cmdV.angular.z = 0;

		current_pub_steer.publish(cmdS);
		current_pub_drive.publish(cmdV);
		*/
		motorsCtrl.throttle = Vel_opt;
		motorsCtrl.steering = delta_opt;
		current_pub.publish(motorsCtrl);
		
		double dist = sqrt(pow(position[0]-waypoints[indexWP][0],2)+pow(position[1]-waypoints[indexWP][1],2));
		if(dist < 0.3){
			if(indexWP < rows-1){
				indexWP += 1;
				ROS_INFO("Target (%f %f)",waypoints[indexWP][0],waypoints[indexWP][1]);
			}
		}
		
		loop_rate.sleep();
	}
	
	return 0;
}
