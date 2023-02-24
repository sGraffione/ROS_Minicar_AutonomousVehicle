#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/Motors.h"
#include "minicar/BtsData.h"
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
#define MAX_SPEED_RATE 0.1 // tuned by hand
#define MAX_DUTY_CYCLE 4000
#define MAX_DELTA 0.523599
#define MAX_DELTA_RATE 0.1
//LQT horizon
#define T 5
#define T_LESS_1 4
#define Np 20
#define Nc 5

using namespace casadi;

int leftMotor = 23;
int dirLeftMotor = 24;
float Ts = 0.1;
float position[3];
double roll = 0, pitch = 0, yaw = M_PI_2;
//double MAX_DELTA_RATE = (M_PI/3*Ts)/0.17; // computation based on datasheet of MG996R servo motor

double L = 0.14;


void stateUpdate(Eigen::MatrixXd &state, float speed, float delta){
	state(0,0) = state(0,0)+Ts*speed*cos(state(2,0));
	state(1,0) = state(1,0)+Ts*speed*sin(state(2,0));
	state(2,0) = state(2,0)+Ts*(speed*tan(delta)/L);
}

void linearizeSystem(Eigen::MatrixXd X, double v_lin, double delta_lin, Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd){
	Ad(0,0) = 1;
	Ad(0,2) = -(v_lin*sin(X(2)))/10;
	Ad(1,1) = 1;
	Ad(1,2) = v_lin*cos(X(2))/10;
	Ad(2,2) = 1;
	Bd(0,0) = cos(X(2))/10;
	Bd(1,0) = sin(X(2))/10;
	Bd(2,0) = (5*tan(delta_lin))/7;
	Bd(2,1) = (5*v_lin*(tan(delta_lin)*tan(delta_lin) + 1))/7;
}

void positionCallback(const minicar::BtsData& msg){
	//ROS_INFO("Position: [%f %f %f] (%i)", msg.position[0],msg.position[1],msg.position[2],msg.quality);
	position[0] = msg.position[0];
	position[1] = msg.position[1];
	position[2] = msg.position[2];
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
/*
void potentialFieldObstacleControl(minicar::Motors &motors, double goal_x, double goal_y, double x, double y, double theta, double ox, double oy){
	double K1 = 3.0;
	double K2 = 1.0;
	
	double gx = goal_x-x;
	double gy = goal_y-y;
	
	double dg = sqrt(gx*gx+gy*gy);
	double fgx = gx/dg;
	double fgy = gy/dg;
	
	double rho_0 = 4;
	double dox = x-ox;
	double doy = y-oy;
	double rho = sqrt(dox*dox+doy*doy);
	double ux = dox/rho;
	double uy = doy/rho;
	
	double fox, foy;
	
	if(rho >= rho_0){
		fox = 0;
		foy = 0;
	}else{
		fox = K1*(1/rho-1/rho_0)*ux/(rho*rho);
		fox = K1*(1/rho-1/rho_0)*uy/(rho*rho);
	}
	
	double vx = fgx + fox;
	double vy = fgy + foy;
	
	double vdir = atan2(vy,vx);
	double angledif = vdir - theta;
	
	if (angledif < -M_PI) angledif += 2*M_PI;
	if (angledif > M_PI) angledif -= 2*M_PI;
	
	if(dg < 0.2){
		motors.throttle = 0.0;
		motors.steering= 0.0;
	}else{
		motors.throttle = 0.0;//speed2DutyCycle(0.1);
		double delta = K2*angledif;
		if (delta > MAX_DELTA) delta = MAX_DELTA;
		if (delta < -MAX_DELTA) delta = -MAX_DELTA;
		motors.steering= delta;
	}
}

bool potentialFieldControl(minicar::Motors &motors, double goal_x, double goal_y, double x, double y, double theta){
	double K1 = 50.0;

	double gx = goal_x-x;
	double gy = goal_y-y;
	
	double dg = sqrt(gx*gx+gy*gy);

	double vx = gx/dg;
	double vy = gy/dg;
	
	double vdir = atan2(vy,vx);
	double angledif = vdir - theta;
	
	if (angledif < -M_PI) angledif += 2*M_PI;
	if (angledif > M_PI) angledif -= 2*M_PI;
	
	if(dg < 0.2){
		motors.throttle = 0.0;
		motors.steering= 0.0;
		return true;
	}else{
		motors.throttle = speed2DutyCycle(0.2);
		double delta = K1*angledif;
		if (delta > MAX_DELTA) delta = MAX_DELTA;
		if (delta < -MAX_DELTA) delta = -MAX_DELTA;
		motors.steering = delta;
		return false;
	}
	//ROS_INFO("[%s %s]",motors.motors[0],motors.motors[2]);
}*/
/*
double finiteDifferenceTangent(double *pt0, double *pt1, double *pt2){
	double mk = 0.5*((pt2[1]-pt1[1])/(pt2[0]-pt1[0])+(pt1[1]-pt0[1])/(pt1[0]-pt0[0]));
	return mk;
}
*/
/*
Eigen::MatrixXd LQT(Eigen::MatrixXd X, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q, Eigen::MatrixXd R){
	
	Eigen::MatrixXd X_ref(3,1);
	X_ref.setZero();
	
	// std::cout << "Q\n" << Q << std::endl;
	// std::cout << "R\n" << R << std::endl;
	// std::cout << "A\n" << A << std::endl;
	// std::cout << "B\n" << B << std::endl;
	// std::cout << "C\n" << C << std::endl;
	
	Eigen::MatrixXd F = Q;
	Eigen::MatrixXd E = B*R.inverse()*B.transpose();
	Eigen::MatrixXd V = C.transpose()*Q*C;
	Eigen::MatrixXd W = C.transpose()*Q;
	
	//std::cout << "E\n" << E << std::endl;
	//std::cout << "V\n" << V << std::endl;
	//std::cout << "W\n" << W << std::endl;
	
	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(A.rows(),A.cols());
	std::vector<Eigen::MatrixXd> P_list(T,P);
	
	Eigen::MatrixXd g(A.rows(),T);
	
	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(B.cols(),B.rows());
	std::vector<Eigen::MatrixXd> L_list(T_LESS_1,L);

	Eigen::MatrixXd Lg = Eigen::MatrixXd::Zero(B.cols(),B.rows());
	std::vector<Eigen::MatrixXd> Lg_list(T_LESS_1,Lg);

	Eigen::MatrixXd x_opt(A.rows(),T);
	x_opt = Eigen::MatrixXd::Zero(A.rows(),T);
	x_opt.block(0,0,x_opt.rows(),1) = X;
	//std::cout << "x_opt:\n" << x_opt << std::endl;

	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(),A.cols());
	
	P_list[T-1] = C.transpose()*F*C;

	g.block(0,T-1,g.rows(),1) = C.transpose()*F*X_ref;
	//std::cout << "P_list[" << T-1 << "]\n" << P_list[T-1] << std::endl;
	for(int i=T-2; i>=0; i--){
		P_list[i] = A.transpose()*P_list[i+1]*(I+E*P_list[i+1]).inverse()*A+V;
		
		g.block(0,i,g.rows(),1) = A.transpose()*(I-(P_list[i+1].inverse()+E).inverse())*g.block(0,i+1,g.rows(),1)+W*X_ref;
	}
	//std::cout << "g\n" << g << std::endl;
	for(int i=0; i<T-1; i++){
		P = P_list[i+1];
		//std::cout << "P_list[" << i+1 << "]\n" << P_list[i+1] << std::endl;
		Eigen::MatrixXd temp = ((R+B.transpose()*P_list[i+1]*B).inverse())*B.transpose();
		L_list[i] = temp*P_list[i+1]*A;
		//std::cout << "L_list[" << i+1 << "]\n" << L_list[i] << std::endl;
		Lg_list[i] = temp;
		//std::cout << "Lg_list[" << i+1 << "]\n" << Lg_list[i] << std::endl;
		//L = L_list[i];
		//Lg = Lg_list[i];
		
		//std::cout << "x_opt.block\n" << x_opt.block(0,i,x_opt.rows(),1) << std::endl;
		//std::cout << "g.block\n" << g.block(0,i+1,g.rows(),1) << std::endl;
		Eigen::MatrixXd xblock = x_opt.block(0,i,x_opt.rows(),1);
		Eigen::MatrixXd gblock = g.block(0,i+1,g.rows(),1);
		
		Eigen::MatrixXd temp2 = (A-B*L_list[i])*xblock + B*Lg_list[i]*gblock;
		//std::cout << "temp2" << temp2 << std::endl;
		x_opt.block(0,i+1,3,1) = temp2;
		//std::cout << "x_opt: " << i << " \n" << x_opt << std::endl;
	}

	Eigen::MatrixXd U = -L_list[1]*x_opt.block(0,1,x_opt.rows(),1)+Lg_list[1]*g.block(0,2,g.rows(),1);
	
	return U;
	//std::cout << "U\n" << U << std::endl;
}
*/
/*
Eigen::MatrixXd MPC(Eigen::VectorXd Xr, Eigen::VectorXd X, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q_bar, Eigen::MatrixXd R_bar){
	int n1 = (int)A.rows();
	int m = (int)B.cols();
	int q = (int)C.rows();
	
	Eigen::MatrixXd X_ref(Xr.rows() * Np, 1);
	Xr.setZero();
	for (int i = 0; i < Np; i++)
	{
		X_ref.block(i*Xr.rows(), 0, Xr.rows(), 1) = Xr;
	}
	
	Eigen::MatrixXd Cp_element = C*A;
	Eigen::MatrixXd Cp = Cp_element;
	
	int rows = (int)Cp_element.rows();
	for (int i = 1; i < Np; i++){
		Cp_element *= A;
		Cp.conservativeResize(Cp.rows()+rows, Cp.cols());
		Cp.bottomRows(rows) = Cp_element;
	}

	Eigen::MatrixXd Dp_firstCol = C*B;
	Eigen::MatrixXd tempElement, AElem;
	AElem = A;
	int blockSizeRow = (int)Dp_firstCol.rows();
	int blockSizeCol = (int)Dp_firstCol.cols();
	for (int i = 1; i < Np; i++){
		if (i >= 2){
			AElem *= A;
		}
		tempElement = C*AElem*B;
		Dp_firstCol.conservativeResize(Dp_firstCol.rows()+blockSizeRow,Dp_firstCol.cols());
		Dp_firstCol.bottomRows(tempElement.rows()) = tempElement;
	}
	
	Eigen::MatrixXd Dp = Dp_firstCol;
	Eigen::MatrixXd Dp_newCol(Dp_firstCol.rows(), Dp_firstCol.cols()), zeroBlock, tempBlock;
	

	for (int j = 1; j < Nc; j++){
		zeroBlock.setZero(blockSizeRow*j,blockSizeCol);
		Dp_newCol.topRows(zeroBlock.rows()) = zeroBlock;
		tempBlock = Dp_firstCol.topRows(Dp_firstCol.rows() - blockSizeRow * j);
		Dp_newCol.bottomRows(tempBlock.rows()) = tempBlock;

		Dp.conservativeResize(Dp.rows(), Dp.cols()+Dp_newCol.cols());
		Dp.rightCols(blockSizeCol) = Dp_newCol;
	}
	
	std::cout << "Dp\n" << Dp << std::endl;
	std::cout << "Q_bar\n" << Q_bar << std::endl;
	std::cout << "R_bar\n" << R_bar << std::endl;
	
	Eigen::MatrixXd H = Dp.transpose()*Q_bar*Dp + R_bar;
	
	std::cout << "X\n" << X << std::endl;
	std::cout << "X_ref\n" << X_ref << std::endl;
	std::cout << "Cp\n" << Cp << std::endl;
	
	Eigen::MatrixXd f = 2*Dp.transpose()*(Cp*X-X_ref);
	
	Eigen::MatrixXd U = -H.inverse()*f;
	return U.block(0,0,2,1); // Return only the first m controls
	//std::cout << U << std::endl;
}
*/
int main(int argc, char **argv){
	ros::init(argc, argv, "controller");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	ros::Publisher current_pub = n.advertise<minicar::Motors>("motorsCtrl",1);
	//ros::Publisher current_pub_drive = n.advertise<geometry_msgs::Twist>("/minicar_driving_controller/cmd_vel",1);
	//ros::Publisher current_pub_steer = n.advertise<std_msgs::Float64MultiArray>("/minicar_steer_controller/command",1);
	
	ros::Subscriber sub = n.subscribe("position",1,positionCallback);
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
	MX beta = atan((lr*tan(delta))/(lr+lr));
	MX rhs = vertcat(std::vector<MX>{X+Ts*V*cos(theta+beta),
									Y+Ts*V*sin(theta+beta),
									theta+Ts*V*(tan(delta)*cos(beta))/(lr+lf),
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

	double waypoints[rows][cols] = {{0.4, 2.4, M_PI_2, 0.2},
									{0.6, 4.3, 0.0,    0.2},
									{3.0, 4.5, 0.0,    0.0}};
	int indexWP = 0;
	
	ROS_INFO("Target (%f %f)",waypoints[indexWP][0],waypoints[indexWP][1]);

	//std_msgs::Float64MultiArray cmdS;
	//geometry_msgs::Twist cmdV;
	minicar::Motors motorsCtrl;
			
	double delta_opt = 0.0, Vel_opt = 0.0, delta_rate_opt = 0.0, Vel_rate_opt = 0.0;

	// Sleep for 3 second before starting. It gives time to gazebo to open.
	ros::Duration(3).sleep();

	while(ros::ok()){
		
		ros::spinOnce();

		arg["x0"] = v_init;

		std::vector<double> P_nlp = { position[0], position[1], yaw, Vel_opt, delta_opt, waypoints[indexWP][0], waypoints[indexWP][1], waypoints[indexWP][2], waypoints[indexWP][3], 0.0 };
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
		
		yaw = v_init.at(2);

		// Get the optimal control
		Vel_rate_opt = V_opt.at(nx);
		delta_rate_opt = V_opt.at(nx+1);
		
		ROS_INFO("[%f %f %f]",Vel_opt,delta_opt,yaw);
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
		motorsCtrl.steering = -delta_opt;
		current_pub.publish(motorsCtrl);
		
		double dist = sqrt(pow(position[0]-waypoints[indexWP][0],2)+pow(position[1]-waypoints[indexWP][1],2));
		if(dist < 0.2){
			if(indexWP < rows-1){
				indexWP += 1;
				ROS_INFO("Target (%f %f)",waypoints[indexWP][0],waypoints[indexWP][1]);
			}
		}
		
		loop_rate.sleep();
	}
	
	return 0;
}
