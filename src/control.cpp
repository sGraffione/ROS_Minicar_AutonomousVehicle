#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/Motors.h"
#include "minicar/BtsData.h"

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
#define MAX_DUTY_CYCLE 4000
#define MAX_DELTA 0.5236
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

double dutyCycle2speed(double dutyCycle){return MAX_SPEED*dutyCycle/MAX_DUTY_CYCLE;}
double speed2DutyCycle(double speed){return speed*MAX_DUTY_CYCLE/MAX_SPEED;}

void positionCallback(const minicar::BtsData& msg){
	//ROS_INFO("Position: [%f %f %f] (%i)", msg.position[0],msg.position[1],msg.position[2],msg.quality);
	position[0] = msg.position[0];
	position[1] = msg.position[1];
	position[2] = msg.position[2];
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

double finiteDifferenceTangent(double *pt0, double *pt1, double *pt2){
	double mk = 0.5*((pt2[1]-pt1[1])/(pt2[0]-pt1[0])+(pt1[1]-pt0[1])/(pt1[0]-pt0[0]));
	return mk;
}

Eigen::MatrixXd LQT(Eigen::MatrixXd X, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q, Eigen::MatrixXd R){
	
	Eigen::MatrixXd X_ref(3,1);
	X_ref.setZero();
	/*
	std::cout << "Q\n" << Q << std::endl;
	std::cout << "R\n" << R << std::endl;
	std::cout << "A\n" << A << std::endl;
	std::cout << "B\n" << B << std::endl;
	std::cout << "C\n" << C << std::endl;
	*/
	Eigen::MatrixXd F = Q;
	Eigen::MatrixXd E = B*R.inverse()*B.transpose();
	Eigen::MatrixXd V = C.transpose()*Q*C;
	Eigen::MatrixXd W = C.transpose()*Q;
	/*
	std::cout << "E\n" << E << std::endl;
	std::cout << "V\n" << V << std::endl;
	std::cout << "W\n" << W << std::endl;
	*/
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
	
	/*std::cout << "Dp\n" << Dp << std::endl;
	std::cout << "Q_bar\n" << Q_bar << std::endl;
	std::cout << "R_bar\n" << R_bar << std::endl;*/
	
	Eigen::MatrixXd H = Dp.transpose()*Q_bar*Dp + R_bar;
	/*
	std::cout << "X\n" << X << std::endl;
	std::cout << "X_ref\n" << X_ref << std::endl;
	std::cout << "Cp\n" << Cp << std::endl;
	*/
	Eigen::MatrixXd f = 2*Dp.transpose()*(Cp*X-X_ref);
	
	Eigen::MatrixXd U = -H.inverse()*f;
	return U.block(0,0,2,1); // Return only the first m controls
	//std::cout << U << std::endl;
}
/*
void MPCCasadi(){
	casadi::SX X = casadi::SX::sym("X", 1);
	casadi::SX Y = casadi::SX::sym("Y", 1);
	casadi::SX psi = casadi::SX::sym("psi", 1);
	casadi::SX V = casadi::SX::sym("V", 3);
	casadi::SX delta = casadi::SX::sym("delta", 3);
}
*/
int main(int argc, char **argv){
	ros::init(argc, argv, "controller");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	ros::Publisher current_pub = n.advertise<minicar::Motors>("motorsCtrl",1);
	ros::Subscriber sub = n.subscribe("position",1,positionCallback);
	
	ros::Rate loop_rate(1/Ts);
	
	double lr = 0.07;
	double lf = 0.07;
	
	int num_state = 5;
	int num_control = 2;
	
	SX X = SX::sym("X", 1);
	SX Y = SX::sym("Y", 1);
	SX psi = SX::sym("psi", 1);
	SX V = SX::sym("V", 1);
	SX delta = SX::sym("delta", 1);

	SX V_rate = SX::sym("V_rate", 1);
	SX delta_rate = SX::sym("delta_rate", 1); 
	
	SX beta = atan((lr*tan(delta))/(lr+lr));
	
	std::vector<SX> rhs = std::vector<SX>{X+Ts*V*cos(psi+beta),
									Y+Ts*V*cos(psi+beta),
									psi+Ts*V*(cos(beta)/((lr+lf)*tan(delta))),
									V+V_rate,
									delta+delta_rate};
	std::cout << rhs << std::endl;
	std::vector<SX> state = std::vector<SX>{X,Y,psi,V,delta};
	std::vector<SX> controls = std::vector<SX>{V_rate, delta_rate};
std::cout << state << std::endl;
	Function f("f",{state,controls},rhs);
	std::cout << f << std::endl;/*
	MX P_initState = MX::sym("P_initState",num_state); // params (initial state)
	MX P_refState = MX::sym("P_refState",num_state); // params (reference state)

	MX states = MX::sym("states",num_state,Np+1);
	MX U = MX::sym("U",num_control,Np);
	
	MX st = MX::sym("st",num_state);
	MX con = MX::sym("con",num_control);
	states.get(st,1,Slice(),Slice(0,0,1));
	std::cout << st << std::endl;
	/*
	MX obj = MX::sym("obj",1); 
	MX g = st-P_initState; // constraints vector
	for(int i = 0; i < Np; i++){
		states.get(st,1,Slice(),Slice(i,i,1));
		U.get(con,1,Slice(),Slice(i,i,1));
		obj = obj + q1*pow(st(0)-P_refState(0),2) + q2*pow(st(1)-P_refState(1),2) + q3*pow(st(2)-P_refState(2),2) + q4*pow(st(3)-P_refState(3),2) + q5*pow(st(4)-P_refState(4),2) + r1*pow(con(0),2) + r2*pow(con(1),2);
		
		MX st_next = MX::sym("st_next",num_state);
		states.get(st_euler,1,Slice(),Slice(i+1,i+1,1));
		MX st_next_euler = 
	}
	
	*/
	
	/*
	double goal[2];
	//double q1,q2,q3,r1,r2;
	/*
	ros::param::get("/controller/q1",q1);
	ros::param::get("/controller/q2",q2);
	ros::param::get("/controller/q3",q3);
	ros::param::get("/controller/r1",r1);
	ros::param::get("/controller/r2",r2);
	
	// Vettore di waypoints:
	int rows = 3; // numero di waypoints
	int cols = 2; // numero di coordinate per waypoint (x,y) 
	//double waypoints[rows][cols] = {{0.3,2.4},{0.6,5},{4.8,2.0}};
	Eigen::MatrixXd waypoints(rows,cols);
	waypoints << 0.3,2.4,
				0.6,5,
				4.8,2.0;
	int indexWP = 0;
	
	
	ROS_INFO("Target (%f %f)",waypoints(0,0),waypoints(1,0));
	
	// Init LQT matrices
	Eigen::MatrixXd X_ref(3,1);
	Eigen::MatrixXd X(3,1);
	Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(3,3);
	Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(3,2);
	Eigen::MatrixXd Cd = Eigen::MatrixXd::Identity(3,3);

	Eigen::MatrixXd Q(3,3);
	Q.setZero();
	Eigen::MatrixXd R(2,2);
	R.setZero();
	
	Q(0,0) = 10;
	Q(1,1) = 10;
	Q(2,2) = 0.001;
	R(0,0) = 1;
	R(1,1) = 0.5;
	
	Eigen::MatrixXd Q_bar(Q.rows() * Np, Q.cols() * Np);
	Q_bar.setZero();
	for (int i = 0; i < Np; i++)
	{
		Q_bar.block(i*Q.rows(), i*Q.cols(), Q.rows(), Q.cols()) = Q;
	}
	Eigen::MatrixXd R_bar(R.rows() * Nc, R.cols() * Nc);
	R_bar.setZero();
	for (int i = 0; i < Nc; i++)
	{
		R_bar.block(i*R.rows(), i*R.cols(), R.rows(), R.cols()) = R;
	}
	
	X << 0.6,
		 0,
		 M_PI/2;
	X_ref << waypoints(0,0),
			 waypoints(0,1),
			 0;
			 
	Eigen::MatrixXd X_ref_Np(X_ref.rows() * Np, 1);
	X_ref_Np.setZero();
	for (int i = 0; i < Np; i++)
	{
		X_ref_Np.block(i*X_ref.rows(), 0, X_ref.rows(), 1) = X_ref;
	}
	
	int v_lin = 0.2;
	int delta_lin = 0;
	
	// Linearization of the model (+ discrete)
	linearizeSystem(X,v_lin,delta_lin,Ad,Bd);
	
	std::cout << "Ad:\n" << Ad << std::endl;
	std::cout << "Bd:\n" << Bd << std::endl;
	std::cout << "Cd:\n" << Cd << std::endl;
	
	
	Eigen::MatrixXd U(2,1);
	U.setZero();
	//LQT(X_ref, X, Ad, Bd, Cd, Q, R);
	double speed = 0;
	double delta = 0;*/
			
	while(ros::ok()){
		
		ros::spinOnce();

		minicar::Motors motorsCtrl;
		
		/*if(indexWP<rows){
			//ROS_INFO("Position: [%f %f]",position[0],position[1]);
			X(0,0) = position[0];
			X(1,0) = position[1];
			X_ref(0,0) = waypoints(indexWP,0);
			X_ref(1,0) = waypoints(indexWP,1);
			linearizeSystem(X,speed,delta,Ad,Bd);
			//std::cout << "Ad:\n" << Ad << std::endl;
			//std::cout << X-X_ref << std::endl;
			//U = MPC(X_ref, X, Ad, Bd, Cd, Q_bar, R_bar);
			U = LQT(X-X_ref, Ad, Bd, Cd, Q, R);
			
			speed = U(0,0);
			delta = U(1,0);
			if (U(0,0)>MAX_SPEED){speed = MAX_SPEED;}else if(U(0,0)<-MAX_SPEED){speed = -MAX_SPEED;}
			if (U(1,0)>MAX_DELTA){delta = MAX_DELTA;}else if(U(1,0)<-MAX_DELTA){delta = -MAX_DELTA;}
			motorsCtrl.throttle = speed2DutyCycle(speed);
			motorsCtrl.steering = delta*180/M_PI;
			stateUpdate(X,speed,delta);
			
			if(sqrt(pow(position[0]-waypoints(indexWP,0),2)+pow(position[1]-waypoints(indexWP,1),2))<=0.3){
				++indexWP;
				ROS_INFO("Target (%f %f)",waypoints(indexWP,0),waypoints(indexWP,1));
			}
		}else{
			ROS_INFO("End of the path");
			motorsCtrl.throttle = 0.0;
			motorsCtrl.steering= 0.0;
		}*/
		//ROS_INFO("Position: [%f %f %f]\nControl: [%f %f]", position[0],position[1],X(2,0)*180/M_PI,dutyCycle2speed(motorsCtrl.throttle), motorsCtrl.steering);
		
		current_pub.publish(motorsCtrl);
		
		//ROS_INFO("Control actions [%f %f]",speed,delta);
		
		loop_rate.sleep();
	}
	
	return 0;
}
