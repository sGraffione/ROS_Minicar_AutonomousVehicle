#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/Motors.h"
#include "minicar/BtsData.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"

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
#define Np 5
#define Nc 2

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

// E/* igen::MatrixXd LQT(Eigen::MatrixXd X, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q, Eigen::MatrixXd R){
	
// 	Eigen::MatrixXd X_ref(3,1);
// 	X_ref.setZero();
// 	/*
// 	std::cout << "Q\n" << Q << std::endl;
// 	std::cout << "R\n" << R << std::endl;
// 	std::cout << "A\n" << A << std::endl;
// 	std::cout << "B\n" << B << std::endl;
// 	std::cout << "C\n" << C << std::endl;
// 	*/
// 	Eigen::MatrixXd F = Q;
// 	Eigen::MatrixXd E = B*R.inverse()*B.transpose();
// 	Eigen::MatrixXd V = C.transpose()*Q*C;
// 	Eigen::MatrixXd W = C.transpose()*Q;
// 	/*
// 	std::cout << "E\n" << E << std::endl;
// 	std::cout << "V\n" << V << std::endl;
// 	std::cout << "W\n" << W << std::endl;
// 	*/
// 	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(A.rows(),A.cols());
// 	std::vector<Eigen::MatrixXd> P_list(T,P);
	
// 	Eigen::MatrixXd g(A.rows(),T);
	
// 	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(B.cols(),B.rows());
// 	std::vector<Eigen::MatrixXd> L_list(T_LESS_1,L);

// 	Eigen::MatrixXd Lg = Eigen::MatrixXd::Zero(B.cols(),B.rows());
// 	std::vector<Eigen::MatrixXd> Lg_list(T_LESS_1,Lg);

// 	Eigen::MatrixXd x_opt(A.rows(),T);
// 	x_opt = Eigen::MatrixXd::Zero(A.rows(),T);
// 	x_opt.block(0,0,x_opt.rows(),1) = X;
// 	//std::cout << "x_opt:\n" << x_opt << std::endl;

// 	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(),A.cols());
	
// 	P_list[T-1] = C.transpose()*F*C;

// 	g.block(0,T-1,g.rows(),1) = C.transpose()*F*X_ref;
// 	//std::cout << "P_list[" << T-1 << "]\n" << P_list[T-1] << std::endl;
// 	for(int i=T-2; i>=0; i--){
// 		P_list[i] = A.transpose()*P_list[i+1]*(I+E*P_list[i+1]).inverse()*A+V;
		
// 		g.block(0,i,g.rows(),1) = A.transpose()*(I-(P_list[i+1].inverse()+E).inverse())*g.block(0,i+1,g.rows(),1)+W*X_ref;
// 	}
// 	//std::cout << "g\n" << g << std::endl;
// 	for(int i=0; i<T-1; i++){
// 		P = P_list[i+1];
// 		//std::cout << "P_list[" << i+1 << "]\n" << P_list[i+1] << std::endl;
// 		Eigen::MatrixXd temp = ((R+B.transpose()*P_list[i+1]*B).inverse())*B.transpose();
// 		L_list[i] = temp*P_list[i+1]*A;
// 		//std::cout << "L_list[" << i+1 << "]\n" << L_list[i] << std::endl;
// 		Lg_list[i] = temp;
// 		//std::cout << "Lg_list[" << i+1 << "]\n" << Lg_list[i] << std::endl;
// 		//L = L_list[i];
// 		//Lg = Lg_list[i];
		
// 		//std::cout << "x_opt.block\n" << x_opt.block(0,i,x_opt.rows(),1) << std::endl;
// 		//std::cout << "g.block\n" << g.block(0,i+1,g.rows(),1) << std::endl;
// 		Eigen::MatrixXd xblock = x_opt.block(0,i,x_opt.rows(),1);
// 		Eigen::MatrixXd gblock = g.block(0,i+1,g.rows(),1);
		
// 		Eigen::MatrixXd temp2 = (A-B*L_list[i])*xblock + B*Lg_list[i]*gblock;
// 		//std::cout << "temp2" << temp2 << std::endl;
// 		x_opt.block(0,i+1,3,1) = temp2;
// 		//std::cout << "x_opt: " << i << " \n" << x_opt << std::endl;
// 	}

// 	Eigen::MatrixXd U = -L_list[1]*x_opt.block(0,1,x_opt.rows(),1)+Lg_list[1]*g.block(0,2,g.rows(),1);
	
// 	return U;
// 	//std::cout << "U\n" << U << std::endl;
// }

// Eigen::MatrixXd MPC(Eigen::VectorXd Xr, Eigen::VectorXd X, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q_bar, Eigen::MatrixXd R_bar){
// 	int n1 = (int)A.rows();
// 	int m = (int)B.cols();
// 	int q = (int)C.rows();
	
// 	Eigen::MatrixXd X_ref(Xr.rows() * Np, 1);
// 	Xr.setZero();
// 	for (int i = 0; i < Np; i++)
// 	{
// 		X_ref.block(i*Xr.rows(), 0, Xr.rows(), 1) = Xr;
// 	}
	
// 	Eigen::MatrixXd Cp_element = C*A;
// 	Eigen::MatrixXd Cp = Cp_element;
	
// 	int rows = (int)Cp_element.rows();
// 	for (int i = 1; i < Np; i++){
// 		Cp_element *= A;
// 		Cp.conservativeResize(Cp.rows()+rows, Cp.cols());
// 		Cp.bottomRows(rows) = Cp_element;
// 	}

// 	Eigen::MatrixXd Dp_firstCol = C*B;
// 	Eigen::MatrixXd tempElement, AElem;
// 	AElem = A;
// 	int blockSizeRow = (int)Dp_firstCol.rows();
// 	int blockSizeCol = (int)Dp_firstCol.cols();
// 	for (int i = 1; i < Np; i++){
// 		if (i >= 2){
// 			AElem *= A;
// 		}
// 		tempElement = C*AElem*B;
// 		Dp_firstCol.conservativeResize(Dp_firstCol.rows()+blockSizeRow,Dp_firstCol.cols());
// 		Dp_firstCol.bottomRows(tempElement.rows()) = tempElement;
// 	}
	
// 	Eigen::MatrixXd Dp = Dp_firstCol;
// 	Eigen::MatrixXd Dp_newCol(Dp_firstCol.rows(), Dp_firstCol.cols()), zeroBlock, tempBlock;
	

// 	for (int j = 1; j < Nc; j++){
// 		zeroBlock.setZero(blockSizeRow*j,blockSizeCol);
// 		Dp_newCol.topRows(zeroBlock.rows()) = zeroBlock;
// 		tempBlock = Dp_firstCol.topRows(Dp_firstCol.rows() - blockSizeRow * j);
// 		Dp_newCol.bottomRows(tempBlock.rows()) = tempBlock;

// 		Dp.conservativeResize(Dp.rows(), Dp.cols()+Dp_newCol.cols());
// 		Dp.rightCols(blockSizeCol) = Dp_newCol;
// 	}
	
// 	/*std::cout << "Dp\n" << Dp << std::endl;
// 	std::cout << "Q_bar\n" << Q_bar << std::endl;
// 	std::cout << "R_bar\n" << R_bar << std::endl;*/
	
// 	Eigen::MatrixXd H = Dp.transpose()*Q_bar*Dp + R_bar;
// 	/*
// 	std::cout << "X\n" << X << std::endl;
// 	std::cout << "X_ref\n" << X_ref << std::endl;
// 	std::cout << "Cp\n" << Cp << std::endl;
// 	*/
// 	Eigen::MatrixXd f = 2*Dp.transpose()*(Cp*X-X_ref);
	
// 	Eigen::MatrixXd U = -H.inverse()*f;
// 	return U.block(0,0,2,1); // Return only the first m controls
// 	//std::cout << U << std::endl;
// }
/*
void MPCCasadi(){
	casadi::MX X = casadi::MX::sym("X", 1);
	casadi::MX Y = casadi::MX::sym("Y", 1);
	casadi::MX psi = casadi::MX::sym("psi", 1);
	casadi::MX V = casadi::MX::sym("V", 3);
	casadi::MX delta = casadi::MX::sym("delta", 3);
}
*/
int main(int argc, char **argv){
	ros::init(argc, argv, "controller");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	//ros::Publisher current_pub = n.advertise<minicar::Motors>("motorsCtrl",1);
	ros::Publisher current_pub_drive = n.advertise<geometry_msgs::Twist>("/minicar_driving_controller/cmd_vel",1);
	ros::Publisher current_pub_steer = n.advertise<std_msgs::Float64MultiArray>("/minicar_steer_controller/command",1);
	
	ros::Subscriber sub = n.subscribe("position",1,positionCallback);
	
	ros::Rate loop_rate(1/Ts);
	
	double lr = 0.07;
	double lf = 0.07;

	double q1 = 1;
	double q2 = 1;
	double q3 = 1;
	double q4 = 1;
	double q5 = 1;
	double r1 = 1;
	double r2 = 2;

	double X_init = 0.6;
	double Y_init = 0.6;
	double psi_init = M_PI_2;

	MX X = MX::sym("X", 1);
	MX Y = MX::sym("Y", 1);
	MX psi = MX::sym("psi", 1);

	MX V = MX::sym("V", 1);
	MX delta = MX::sym("delta", 1); 

	MX state = vertcat(X,Y,psi);
	MX controls = vertcat(V, delta);

	// Number of differential states
	int nx = state.size1();

	// Number of controls
	int nu = controls.size1();

	// Bounds and initial guess for the control
	std::vector<double> u_min =  { -0.3, -0.523599  };
	std::vector<double> u_max  = {  0.3,  0.523599  };
	std::vector<double> u_init = {  0.0,  0.0  };

	// Bounds and initial guess for the state
	std::vector<double> x_min  = { 0, 0, -inf };
	std::vector<double> x_max  = { 6, 6,  inf };
	std::vector<double> x_init = { X_init, Y_init, psi_init }; // to get from localization topic

	// Non linear, time continuous equations of the system
	MX beta = atan((lr*tan(delta))/(lr+lr));
	MX rhs = vertcat(V*cos(psi+beta),
					 V*cos(psi+beta),
					 V*(cos(beta)/((lr+lf)*tan(delta))));
	
	MXDict dae = {{"x",state},{"p",controls},{"ode",rhs}}; // build a dictionary containing state, control and equations vector

	// Create an integrator to derivate the rhs (rk param states for RungeKutta integration)
	Function f = integrator("integrator","rk",dae,{{"t0",0},{"tf",Np}});
	
	MX P_initState = MX::sym("P_initState",nx); // params (initial state)
	MX P_refState = MX::sym("P_refState",nx); // params (reference state)
	MX P = vertcat(P_initState,P_refState);

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

	// Loop over shooting nodes
	for(int k=0; k<Np; ++k){
	// Create an evaluation node
		MXDict I_out = f(MXDict{{"x0", st[k]}, {"p", con[k]}});
		//std::cout << I_out << std::endl;
		// Save continuity constraints
		g.push_back( st[k+1] - I_out.at("xf") );
		obj += q1*pow(st[k](0)-P_refState(0),2) + q2*pow(st[k](1)-P_refState(1),2) + q3*pow(st[k](2)-P_refState(2),2) + r1*pow(con[k](0),2) + r2*pow(con[k](1),2);
		// Add objective function contribution
		//obj += I_out.at("qf");
	}
	
	// NLP
	MXDict nlp = {{"x", varVect}, {"f", obj}, {"g", vertcat(g)},{"p",P}};

	// Set options
	Dict opts;
	opts["ipopt.tol"] = 1e-2;
	opts["ipopt.max_iter"] = 100;

	// Create an NLP solver and buffers
	Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);
	std::map<std::string, DM> arg, res;

	// Bounds and initial guess
	arg["lbx"] = v_min;
	arg["ubx"] = v_max;
	arg["lbg"] = 0;
	arg["ubg"] = 0;
	arg["x0"] = v_init;

	std::vector<double> P_nlp = { X_init, Y_init, psi_init, 0.6, 5.0, 0 };

	arg["p"] = P_nlp;

	// Solve the problem
	res = solver(arg);

	// Optimal solution of the NLP
	std::vector<double> V_opt(res.at("x"));

	// Get the optimal state trajectory
	std::vector<double> r_opt(Np+1), s_opt(Np+1);
	for(int i=0; i<=Np; ++i){
		r_opt[i] = V_opt.at(i*(nx+1));
		s_opt[i] = V_opt.at(1+i*(nx+1));
	}
	std::cout << "r_opt = " << std::endl << r_opt << std::endl;
	std::cout << "s_opt = " << std::endl << s_opt << std::endl;

	// Get the optimal control
	std::vector<double> u_opt(Np);
	for(int i=0; i<Np; ++i){
		u_opt[i] = V_opt.at(nx + i*(nx+1));
	}
	std::cout << "u_opt = " << std::endl << u_opt << std::endl;

/*
	// Bounds and initial guess
	arg["lbx"] = v_min;
	arg["ubx"] = v_max;
	arg["lbg"] = 0;
	arg["ubg"] = 0;
	arg["x0"] = v_init;

	// Solve the problem
	res = solver(arg);

	// Optimal solution of the NLP
	vector<double> V_opt(res.at("x"));

	// Get the optimal state trajectory
	vector<double> r_opt(ns+1), s_opt(ns+1);
	for(int i=0; i<=ns; ++i){
	r_opt[i] = V_opt.at(i*(nx+1));
	s_opt[i] = V_opt.at(1+i*(nx+1));
	}
	cout << "r_opt = " << endl << r_opt << endl;
	cout << "s_opt = " << endl << s_opt << endl;

	// Get the optimal control
	vector<double> u_opt(ns);
	for(int i=0; i<ns; ++i){
	u_opt[i] = V_opt.at(nx + i*(nx+1));
	}
	cout << "u_opt = " << endl << u_opt << endl;
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

		std_msgs::Float64MultiArray cmdS;
		geometry_msgs::Twist cmdV;
		cmdS.data= {0.3, 0.3};
		cmdV.linear.x = 0.1;
		cmdV.linear.y = 0;
		cmdV.linear.z = 0;
		cmdV.angular.x = 0;
		cmdV.angular.y = 0;
		cmdV.angular.z = 0;

		current_pub_steer.publish(cmdS);
		current_pub_drive.publish(cmdV);

		//minicar::Motors motorsCtrl;
		
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
		
		//current_pub.publish(motorsCtrl);
		
		//ROS_INFO("Control actions [%f %f]",speed,delta);
		
		loop_rate.sleep();
	}
	
	return 0;
}
