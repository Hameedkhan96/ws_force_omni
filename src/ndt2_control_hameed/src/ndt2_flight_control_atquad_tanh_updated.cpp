#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"

using namespace std;
using namespace Eigen;

//*-------------------------Functions----------------------*/
float c(float x);		            //-------Short cosine function
float s(float x);		            //-------Short sine function
float sat(float _x,float val);	    //--Saturation function
float db(float _x,float val);	    //--Dead band for control
Matrix3d QuatToMat(Vector4d Quat);  //--Quaternion to matrix
Vector3d R2XYZ(Matrix3d R);         //--Rotation to XYZ angles
void save();			            //-------Saving data function
float _fmin(float a,float b),_fmax(float a,float b);	//-------mix max functions
Matrix3f Jac(float _q2, float _q3, float _q5 );
/*-------------------------Functions----------------------*/

/*--------------------------Matrices----------------------*/
MatrixXf F(6,8);		//-------Allocator
MatrixXf invF(8,6);		//-------inverse of allocator
Matrix3f J;		        //-------Manipulator Jacobian
/*--------------------------Matrices----------------------*/

/*--------------------------Vectors-----------------------*/
Vector3f eR;			//-------Orientation error
Vector3f eO;			//-------Orientation velocity error
VectorXf u(6);			//-------Nonlinear control
VectorXf f(8);			//-------Allocated forces
VectorXf T(4);			//-------Vectorized thrusts
VectorXf a(4);			//-------Tilt angles
VectorXf S(4);			//-------Orientation rotors
VectorXf eps(4);		//-------Epsilon: tilting direction
/*--------------------------Vectors-----------------------*/

/*------------------Parameters of dynamics---------------*/
float pi = 3.1416;		//-------pi
float g = 9.81;
float m = 6.3833;
//float m = 5.;
//float m = 12., Ixx = 0.3653, Iyy = 0.3653, Izz = 0.4418;
float kf = 8.54858e-05, km = 1.75e-04, sig = kf/km;
//float kf = 8.54858e-05, km = 1.6e-2, sig = kf/km;

//float kf = 0.00019065, km = 0.000001333, sig = kf/km;
float l = 0.183847763;
float body_width = 0.470;  float body_height = 0.380;
/* 
float Ixx = 0.29163;
float Iyy = 0.29163;
float Izz = 0.3527;
*/

float Ixx = (1/12.) * m * (body_height * body_height + body_width * body_width);
float Iyy = (1/12.) * m * (body_height * body_height + body_width * body_width);
float Izz = (1/12.) * m * (body_width * body_width + body_width * body_width);

/*------------------Parameters of dynamics---------------*/

/*--------------------Control variables------------------*/
float x=0.0,y=0.0,z=0.0,dx=0.0,dy=0.0,dz=0.0,psi=0.0,the=0.0,phi=0.0,dphi=0.0,dthe=0.0,dpsi=0.0;
float fx,fy,fz,tx,ty,tz;

geometry_msgs::WrenchStamped wrench_stamped_msg;
geometry_msgs::WrenchStamped wrench_est_msg;

float ex,ey,ez,ephi,ethe,epsi;
float dex,dey,dez,dephi,dethe,depsi;

Vector3d _orientation;
Vector3f dX,dX0,dX1,dq;
/*--------------------Control variables------------------*/


/*----------------------References-----------------------*/
float xd=0.0, yd=0.0, zd=1.5;
float phid=0.*pi/180.0,thed=0.*pi/180.0,psid= 0.0*pi/180.0;
float dxd=0.0,dyd=0.0,dzd=0.0,ddxd=0.0,ddyd=0.0,ddzd=0.0;
float dphid=0.0,dthed=0.0,dpsid=0.0,ddphid=0.0,ddthed=0.0,ddpsid=0.0;
float take_off = 1.0,sw = 0;
float Fxd = 0,Fx = 0,Fx0 = 0;
/*----------------------References-----------------------*/

/*---------------------ROS variables---------------------*/
void joint_states_callback(const sensor_msgs::JointState& joint_states_msg);
void odometry_callback(const nav_msgs::Odometry odometry_msg);
void wrench_estimation_callback(const geometry_msgs::WrenchStamped wrench_est_msg);
void references_callback(const std_msgs::Float32MultiArray references_msg);
std_msgs::Float64 t_msg1;
std_msgs::Float64 t_msg2;
std_msgs::Float64 t_msg3;
std_msgs::Float64 t_msg4;
std_msgs::Float32MultiArray motor_vel;
std_msgs::Float32MultiArray errors;
nav_msgs::Odometry odom;
std_msgs::Float32MultiArray references;
sensor_msgs::JointState joints;
geometry_msgs::WrenchStamped wrench_est;
/*---------------------ROS variables---------------------*/

int main(int argc, char **argv)
{
	/*------ROS Setup-------*/
	ros::init(argc, argv,"flight_controller_atquad");
	ros::NodeHandle nh;
	
	ros::Publisher rotors_publisher  = nh.advertise<std_msgs::Float32MultiArray>("/ndt2/cmd/motor_vel",0);
	ros::Publisher error_publisher  = nh.advertise<std_msgs::Float32MultiArray>("errors",0);
	ros::Publisher tilt_1_publisher = nh.advertise<std_msgs::Float64>("/tilt_motor_1/command",0);
	ros::Publisher tilt_2_publisher = nh.advertise<std_msgs::Float64>("/tilt_motor_2/command",0);
	ros::Publisher tilt_3_publisher = nh.advertise<std_msgs::Float64>("/tilt_motor_3/command",0);
	ros::Publisher tilt_4_publisher = nh.advertise<std_msgs::Float64>("/tilt_motor_4/command",0);
	
	ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("/ndt2/cmd/wrench",0);

	ros::Subscriber target = nh.subscribe("/references", 1, references_callback);
	ros::Subscriber feedback = nh.subscribe("/ndt2/odometry", 1, odometry_callback);
	ros::Subscriber estimation = nh.subscribe("/wrench_estimation", 1, wrench_estimation_callback);

	motor_vel.data.resize(8);
	errors.data.resize(6);
	references.data.resize(21);
	
	ros::Rate loop_rate(1000);

	/*---Allocation----*/
	eps<<-1,1,-1,1;
	S(0) = pi/4.; S(1) = 3*pi/4.; S(2) = -3*pi/4.; S(3) = -pi/4.;

	F(0,0) =  0; 			F(0,1) =  0;  			F(0,2) =  0; 			F(0,3) =  0;
	F(1,0) =  0; 			F(1,1) =  0; 			F(1,2) =  0; 			F(1,3) =  0;
	F(2,0) =  1; 			F(2,1) =  1; 			F(2,2) =  1; 			F(2,3) =  1;
	
	F(3,0) =  l*s(S[0]);  	F(3,1) =  l*s(S[1]); 	F(3,2) =  l*s(S[2]); 	F(3,3) =  l*s(S[3]);
	F(4,0) = -l*c(S[0]);  	F(4,1) = -l*c(S[1]); 	F(4,2) = -l*c(S[2]); 	F(4,3) = -l*c(S[3]);
	F(5,0) = -eps(0)*sig; 	F(5,1) = -eps(1)*sig;	F(5,2) = -eps(2)*sig; 	F(5,3) = -eps(3)*sig;
	//------X-Y Thrust components--------------------
	F(0,4) =  s(S[0]); 			 F(0,5) =  s(S[1]); 		  F(0,6) =  s(S[2]); 		   F(0,7) =  s(S[3]);
	F(1,4) = -c(S[0]); 			 F(1,5) = -c(S[1]); 		  F(1,6) = -c(S[2]); 		   F(1,7) = -c(S[3]);
	F(2,4) =  0; 				 F(2,5) =  0; 				  F(2,6) =  0; 			 	   F(2,7) =  0;
	
	F(3,4) = -s(S[0])*eps(0)*sig;F(3,5) = -s(S[1])*eps(1)*sig;F(3,6) = -s(S[2])*eps(2)*sig;F(3,7) =  -s(S[3])*eps(3)*sig;
	F(4,4) =  c(S[0])*eps(0)*sig;F(4,5) =  c(S[1])*eps(1)*sig;F(4,6) =  c(S[2])*eps(2)*sig;F(4,7) = c(S[3])*eps(3)*sig;
	F(5,4) = -l;				 F(5,5) = -l; 				  F(5,6) = -l; 				    F(5,7) = -l;
	
	/*------invF--------*/
	CompleteOrthogonalDecomposition<MatrixXf> cod(F);
	invF = cod.pseudoInverse();
	/*--Integration variables--*/	
	float vx = 0.0;	float vy = 0.0;	float vz = 0.0;
	float vphi = 0.0; float vthe = 0.0;	float vpsi = 0.0;
	/*------ROS Loop-------*/
	while (ros::ok())
	{
		cout<<"-------------------------------------"<<endl;
		
		// take_off = 1.0;            
	      take_off = references.data[18];
		  sw = references.data[19];

		/*-----References-----------*/
	
	
		xd = db( references.data[0],1e-2 );
		yd = db( references.data[1],1e-2 );
		zd = db( references.data[2],1e-2 );
		phid = db( references.data[3],1e-2 );
		thed = db( references.data[4],1e-2 );
		psid = db( references.data[5],1e-2 );
		
		dxd = db( references.data[6],1e-2 );
		dyd = db( references.data[7],1e-2 );
		dzd = db( references.data[8],1e-2 );
		dphid = db( references.data[9],1e-2 );
		dthed = db( references.data[10],1e-2 );
		dpsid = db( references.data[11],1e-2 );
		
		ddxd = db( references.data[12],1e-2 );
		ddyd = db( references.data[13],1e-2 );
		ddzd = db( references.data[14],1e-2 );
		ddphid = db( references.data[15],1e-2 );
		ddthed = db( references.data[16],1e-2 );
		ddpsid = db( references.data[17],1e-2 );
	
    /*
		xd = 0;
		yd = 0;
    	zd = 1.5;
    	phid = 0;
		thed = 0;
		psid = 0;
	
		dxd = 0;
		dyd = 0;
		dzd = 0;
		dphid = 0;
		dthed = 0;
		dpsid = 0;
		
		ddxd = 0;
		ddyd = 0;
		ddzd = 0;
		ddphid = 0;
		ddthed = 0;
		ddpsid = 0;
    */
	
		// Fxd = references.data[20];
		/*-----Feedback-----------*/
		x =  odom.pose.pose.position.x;
		y = -odom.pose.pose.position.y;
		z = -odom.pose.pose.position.z;
		
		dx =  odom.twist.twist.linear.x;
		dy = -odom.twist.twist.linear.y;
		dz = -odom.twist.twist.linear.z;
		
		_orientation = R2XYZ( QuatToMat ( Vector4d( odom.pose.pose.orientation.w,  odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,  -odom.pose.pose.orientation.z ) ) );
		
		phi =  _orientation[0];
		the =  _orientation[1];
		psi =  _orientation[2];
		
		dphi =  odom.twist.twist.angular.x;
		dthe =  odom.twist.twist.angular.y;
		dpsi = -odom.twist.twist.angular.z;
		
		/*-----Error signals--------*/
		ex  = xd - x;   ey  = yd - y;   ez  = zd - z;
		dex = dxd - dx; dey = dyd - dy; dez = dzd - dz;		
		ephi  = phid - phi;   ethe  = thed  - the;  epsi  = psid  - psi;
		dephi = dphid - dphi; dethe = dthed - dthe; depsi = dpsid - dpsi;
		/*-----FST Control----------*/
		/*float kpx = 0;
		if(z>0.8){kpx = 9;}
		else{kpx = 4;}*/
		//fx =  (5*tanh(ex)+4*dex+0.1*tanh(vx);
	
 /*
		fx =  (1-sw)*(5*tanh(ex)+4*dex+0.1*tanh(vx))+sw*Fxd;
  	    fy = -(1.0*tanh(ey)+2*dey+0.03*tanh(vy));
  	    fz =  80*tanh(ez)+40*dez+10*tanh(vz);
  	    tx =   2*dephi + 10*ephi;
  	    ty = -(2*dethe + 10*ethe + 0.3*vthe);
  	    tz =   2*depsi + 2*epsi + 0.01*vpsi;
	  	u<<fx,fy,fz,tx,ty,tz;
	  */

        fx =  (1-sw)*(5*tanh(ex)+4*dex+0.1*tanh(vx))+sw*Fxd;
		fy = -(5.5*tanh(ey) + 9.0*dey + 0.1*tanh(vy));
		//fy = -(6.0*tanh(ey)+10.50*dey+0.1152*tanh(vy));
		//fy = -(5.5*tanh(ey) + 8*dey+  0.08*tanh(vy));
		//fz =  180*tanh(ez)+80*dez+30*tanh(vz);
		fz =  180*tanh(ez) + 80*dez + 25*tanh(vz);
		tx =   2*dephi + 10*ephi;
		ty = -(2*dethe + 10*ethe + 0.3*vthe);
		tz =   2*depsi + 2*epsi + 0.01*vpsi;
		u<<fx,fy,fz,tx,ty,tz;

		// Set wrench values as ROS1 topic
    	wrench_stamped_msg.wrench.force.x = fx;
    	wrench_stamped_msg.wrench.force.y = fy;
    	wrench_stamped_msg.wrench.force.z = fz;
		wrench_stamped_msg.wrench.torque.x = tx;
    	wrench_stamped_msg.wrench.torque.y = ty;
    	wrench_stamped_msg.wrench.torque.z = tz;
		// Update the timestamp
        wrench_stamped_msg.header.stamp = ros::Time::now();

        /*
        fx =  (1-sw)*(5*tanh(ex)+4*dex+0.1*tanh(vx))+sw*Fxd;
		fy = -(1.5*tanh(ey)+2*dey+0.03*tanh(vy));
		fz =  400*tanh(ez)+200*dez+100*tanh(vz);
		tx =   2*dephi + 10*ephi;
		ty = -(2*dethe + 10*ethe + 0.3*vthe);
		tz =   2*depsi + 2*epsi + 0.01*vpsi;
		u<<fx,fy,fz,tx,ty,tz;
        */

		/*-----Allocation------------*/
		f = invF*u;
		/*-----Tilting----------*/
		for(int i=0; i<4; i++ ) {
			T[i] = sqrt( pow(f[i],2)+pow(f[i+4],2) );
			a[i] = atan2( f[i+4],abs(f[i]) );
		}
		cout<<"a:"<<endl;
		cout<<180*db(a[0],1e-3)/pi<<','<<180*db(a[1],1e-3)/pi <<','<<180*db(a[2],1e-3)/pi<<','<<180*db(a[3],1e-3)/pi<<endl;
		
		/*-----Publishing----------*/
		if(take_off == 1.0){
			t_msg1.data =  db(a[0],1e-3);
			t_msg2.data =  db(a[1],1e-3);
			t_msg3.data =  db(a[2],1e-3);
			t_msg4.data =  db(a[3],1e-3);
			
			motor_vel.data[1] = sat( sqrt(T[0]/kf),1100 );   // red front-left up --2-- CW    
			motor_vel.data[2] = sat( sqrt(T[1]/kf),1100 );   // blue backt-left up--3-- CCW
			motor_vel.data[3] = sat( sqrt(T[2]/kf),1100 );   // blue back-right up--4-- CW
			motor_vel.data[0] = sat( sqrt(T[3]/kf),1100 );   // red front-right up--1-- CCW

			motor_vel.data[4] = sat( sqrt(T[0]/kf),1100 );   // red front-left down --5-- CCW 
			motor_vel.data[7] = sat( sqrt(T[1]/kf),1100 );   // blue backt-left down--8-- CW
			motor_vel.data[6] = sat( sqrt(T[2]/kf),1100 );   // blue backt-right down--7-- CCW
			motor_vel.data[5] = sat( sqrt(T[3]/kf),1100 );   // red front-right down --6-- CW 
		
		
		/*
			motor_vel.data[2] = sat( sqrt(T[0]/kf),1100 );
			motor_vel.data[1] = sat( sqrt(T[1]/kf),1100 );
			motor_vel.data[3] = sat( sqrt(T[2]/kf),1100 );
			motor_vel.data[0] = sat( sqrt(T[3]/kf),1100 );
			
			motor_vel.data[4] = 0*sat( sqrt(T[0]/kf),1100 );
			motor_vel.data[5] = 0*sat( sqrt(T[1]/kf),1100 );
			motor_vel.data[6] = 0*sat( sqrt(T[2]/kf),1100 );
			motor_vel.data[7] = 0*sat( sqrt(T[3]/kf),1100 );
		*/
		}
		
		else{
			t_msg1.data =  0.;
			t_msg2.data =  0.;
			t_msg3.data =  0.;
			t_msg4.data =  0.;	
			motor_vel.data[2] = 0.;
			motor_vel.data[1] = 0.;
			motor_vel.data[3] = 0.;
			motor_vel.data[0] = 0.;

            motor_vel.data[4] = 0.;
			motor_vel.data[5] = 0.;
			motor_vel.data[6] = 0.;
			motor_vel.data[7] = 0.;

		}
		cout<<"rotor velocities:"<<endl;
		cout<<"rotor"<<1<<" = "<<motor_vel.data[2]<<endl;
		cout<<"rotor"<<2<<" = "<<motor_vel.data[1]<<endl;
		cout<<"rotor"<<3<<" = "<<motor_vel.data[3]<<endl;
		cout<<"rotor"<<4<<" = "<<motor_vel.data[0]<<endl;
		tilt_1_publisher.publish(t_msg1);
		tilt_2_publisher.publish(t_msg2);
		tilt_3_publisher.publish(t_msg3);
		tilt_4_publisher.publish(t_msg4);
		
		rotors_publisher.publish(motor_vel);

		// Wrench controler
		wrench_publisher.publish(wrench_stamped_msg);
		
		errors.data[0] = ex;		
		errors.data[1] = ey;		
		errors.data[2] = ez;
		errors.data[3] = ephi*180.0/pi;		
		errors.data[4] = ethe*180.0/pi;		
		errors.data[5] = epsi*180.0/pi;
		
		error_publisher.publish(errors);
		/*-----Control info-----*/
		cout<<"x = "<<x<<", y = "<<y<<", z = "<<z<<endl;
		cout<<"fx = "<<u[0]<<", fy = "<<u[1]<<", fz = "<<u[2]<<endl;
		cout<<"phi = "<<phi*180.0/pi<<", the = "<<the*180.0/pi<<", psi = "<<psi*180.0/pi<<endl;
		cout<<"tx = "<<u[3]<<", ty = "<<u[4]<<", tz = "<<u[5]<<endl;
		cout<<"fx_est = "<<wrench_est.wrench.force.x<<endl;
		cout<<"\n-------------"<<endl;
			
		ros::spinOnce();
		loop_rate.sleep();
		
		/*-----Integration-----*/
		if(take_off == 1.0){
			vx = vx + 0.1*ex;
			vy = vy + 0.1*ey;
			vz = vz + 0.1*ez;
			vphi = vphi;
			vthe = vthe + 0.1*ethe;
			vpsi = vpsi + 0.1*epsi;
		}
		else{
			vx = 0;
			vy = 0;
			vz = 0;
			vphi = 0;
			vthe = 0;
			vpsi = 0;
		}
		save();
	}
	return 0;
}

void joint_states_callback(const sensor_msgs::JointState& joint_states_msg){
    joints = joint_states_msg;
}

void references_callback(const std_msgs::Float32MultiArray references_msg) {
    references = references_msg;
    //cout<<"rata"<<endl;
}

void odometry_callback(const nav_msgs::Odometry odometry_msg) {
    odom = odometry_msg;
}

void wrench_estimation_callback(const geometry_msgs::WrenchStamped wrench_est_msg) {
    wrench_est = wrench_est_msg;
}

/*
void save(){
	ofstream myfile;
	myfile.open ("Results.txt",std::ios::app);
	myfile <<xd<<","<<yd<<","<<zd<<","<<x-0.5<<","<<y<<","<<z<<endl;
	myfile.close();
}
*/
bool firstWrite = true;  // Global or static variable to keep track if it's the first write

void save() {
    ofstream myfile;

    if (firstWrite) {
        myfile.open("Results.txt", std::ios::trunc);  // Open in trunc mode to clear the file
        firstWrite = false;  // Set to false after first write
    } else {
        myfile.open("Results.txt", std::ios::app);  // Open in append mode for subsequent writes
    }

    myfile << xd << "," << yd << "," << zd << "," << x << "," << y << "," << z << "," << phi << "," << the << "," << psi << "," << fx << "," << wrench_est.wrench.force.x << endl;   
    myfile.close();
}



float c(float _x){
	return cos(_x);
}

float s(float _x){
	return sin(_x);
}

float sat(float _x,float val){
	if(_x>val) return val;
	else if(_x<-val) return -val;
	else return _x;
}

Matrix3f Jac(float _q2, float _q3, float _q5 ){
	Matrix3f _J;
	float W1 = 0.018, H1 = -0.03, L2 = 0.2, L3 = 0.2, L5 = 0.043;
	_J(0,0) = -L2*sin(_q2)-L3*sin(_q2+_q3)-L5*sin(_q2+_q3+_q5);
	_J(0,1) = -L3*sin(_q2+_q3)-L5*sin(_q2+_q3+_q5);
	_J(0,2) = -L5*sin(_q2+_q3+_q5);
	_J(1,0) = 0; _J(1,1) = 0; _J(1,2) = 0;
	_J(2,0) =  L2*cos(_q2) +L3*cos(_q2+_q3)+L5*cos(_q2+_q3+_q5);
	_J(2,1) =  L3*cos(_q2+_q3)+L5*cos(_q2+_q3+_q5);
	_J(2,2) =  L5*cos(_q2+_q3+_q5);
	return _J;
}


float db(float _x,float val){
	if(abs(_x)<val) return 0;
	else return _x;
}

float _fmin(float a,float b){
	if(a>b){
		return b;
	}
	else{
		return a;
	}
}

float _fmax(float a,float b){
	if(a<b){
		return b;
	}
	else{
		return a;
	}
}

MatrixXf hat(float _wx, float _wy, float _wz){
	MatrixXf _hat(3,3);
	_hat(0,0) =   0 ; _hat(0,1) = -_wz; _hat(0,2) =  _wy;
	_hat(1,0) =  _wz; _hat(1,1) =   0 ; _hat(1,2) = -_wx;
	_hat(2,0) = -_wy; _hat(2,1) =  _wx; _hat(2,2) =   0 ;
	return _hat;
}

Matrix3d QuatToMat(Vector4d Quat){
    Matrix3d Rot;
    float _s = Quat[0];
    float _x = Quat[1];
    float _y = Quat[2];
    float _z = Quat[3];
    Rot << 1-2*(_y*_y+_z*_z),2*(_x*_y-_s*_z),2*(_x*_z+_s*_y),
    2*(_x*_y+_s*_z),1-2*(_x*_x+_z*_z),2*(_y*_z-_s*_x),
    2*(_x*_z-_s*_y),2*(_y*_z+_s*_x),1-2*(_x*_x+_y*_y);
    return Rot;
}

Vector3d R2XYZ(Matrix3d R) {
    double _phi=0.0, _theta=0.0, _psi=0.0;
    Vector3d XYZ = Vector3d::Zero();
    
    _theta = asin(R(0,2));
    
    if(fabsf(cos(_theta))>pow(10.0,-10.0))
    {
        _phi=atan2(-R(1,2)/cos(_theta), R(2,2)/cos(_theta));
        _psi=atan2(-R(0,1)/cos(_theta), R(0,0)/cos(_theta));
    }
    else
    {
        if(fabsf(_theta-pi/2.0)<pow(10.0,-5.0))
        {
            _psi = 0.0;
            _phi = atan2(R(1,0), R(2,0));
            _theta = pi/2.0;
        }
        else
        {
            _psi = 0.0;
            _phi = atan2(-R(1,0), R(2,0));
            _theta = -pi/2.0;
        }
    }
    
    XYZ << _phi,_theta,_psi;
    return XYZ;
}
