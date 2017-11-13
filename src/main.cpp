#include <gnssimu.hpp>
/*NOTE: if using g++, glob all .cpp files and build with
g++ main.cpp -o testtest -I /usr/include/eigen3 -I /home/rnl/gnssIMUtest/gnssimu/include/
to get relevant eigen headers*/

//constants
#define pi 3.14159265 //lazy, should just calculate this

//main
int main() 
{
	bool success;
	double t_lastproc, true_long, true_lat, true_height;
	Eigen::Vector3d true_v_enu, true_euler, lever;
	Eigen::Matrix3d true_CTM, Hr1, Hv1, Hv5;
	Eigen::MatrixXd H_ecef;
	Eigen::VectorXd x_imu_ecef, initStates;
	x_imu_ecef.resize(15);  //delta-psi_eb, delta-v_eb, delta-r_eb, b_a, b_g
	H_ecef.resize(6,15);
	initStates.resize(15); initStates.setZero(); lever.setZero();

	gnssimu ins_instance;
	success=ins_instance.setInitStates(initStates,lever);
	//success=ins_instance.setInitCovariance(<DO STUFF HERE>);


    std::cout << "running\n";
    int numepochs=10;
    for(int epoch=2; epoch<numepochs; epoch++)
    {
    	Eigen::Vector3d ee(0,1,pi/2);
    	Eigen::Matrix3d cc = ins_instance.euler_to_CTM(ee);
    	std::cout << cc <<"\n";

    }




    return 0;
}


//cut/paste to gnssimu when ready
gnssimu::gnssimu()
{
	const double omegaie=7292115*pow(10,-11);
	Omega_mat<<0,-omegaie,0,
		omegaie,0,0,
		0,0,0;

	Eigen::VectorXd tmpdiag;
	Pimu.resize(15,15);
	Rgps.resize(6,6);
	tmpdiag.resize(15);
	tmpdiag<<1,1,1, 0.1,0.1,0.1, 0.1,0.1,0.1, 0.1,0.1,0.1, 0.1,0.1,0.1;
	Pimu=tmpdiag.array().matrix().asDiagonal();
	tmpdiag.resize(6);
	Rgps=tmpdiag.array().matrix().asDiagonal();

	Qimu.resize(15,15);
	Qimu=Pimu;
	imustate.resize(15); imustate.setZero();  //imu state with deltax/detlav/deltaeuler
	Gamma.resize(15,15);
	Gamma=Eigen::MatrixXd::Identity(15,15);
	xState.resize(15); //true state
	evxState.resize(9); //pos/vel/euler
	
	Fimu.resize(15,15);

	initImu=false;
	initGps=false;
}


void gnssimu::processGPS(Eigen::VectorXd measGPS)
{
	//ASSUMED MEASUREMENT FORM: tgps, x, v, quaternion
	initGps=true;
	tlastgps=measGPS(0);
	if(initImu)
	{gpsImuCallback(measGPS);}
	else{gpsOnlyCallback(measGPS);}
}


void gnssimu::processIMU(Eigen::VectorXd measIMU)
{
	initImu=true;
	//ASSUMED MEASUREMENT FORM: timu, ax, aq
	tlastimu=measIMU(0);
	//complementary filter for IMU
	
}


void gnssimu::gpsOnlyCallback(Eigen::VectorXd zMeas)
{
	rGPS<<zMeas(1),zMeas(2),zMeas(3);
	vGPS<<zMeas(4),zMeas(5),zMeas(6);
}


void gnssimu::gpsImuCallback(Eigen::VectorXd zMeas)
{
	//recalcluate Fimu for deltaT
	Eigen::VectorXd xkbar, xkout, ztilde;
	xkbar.resize(15); xkout.resize(15);
	xkbar.setZero(); xkout.setZero();
	ztilde.resize(6);

	//assuming t/pose/vel message
	rGPS<<zMeas(1),zMeas(2),zMeas(3);
	vGPS<<zMeas(4),zMeas(5),zMeas(6);
	ztilde=returnMeasurement();

	kalmanPropagate(imustate, Pimu, Fimu, Qimu, Gamma, xkbar, Pimu);
	Eigen::MatrixXd H;
	H.resize(6,15);
	H=returnH();
	kalmanMeasure(xkbar, Pimu, ztilde, H, Rgps, xkout, Pimu);
	//LOOK UP HOW GROVES DOES RECOMBINATION
	for(int i=0; i++; i<=8){
		xState(i)=evxState(i)+imustate(i);
		imustate(i)=0; //deltaxve=0
		//SET XVESTATE TO TRUTH SOMEWHERE
		evxState(i)=xState(i);
	}
	b_a<<imustate(9),imustate(10),imustate(11);
	b_g<<imustate(12), imustate(13), imustate(14);


}

/*void gnssimu::imuCallback(Eigen::Vector zMeas)
{

	rIMU=;
	vIMU=;	
}
*/

//required functions
Eigen::Matrix3d gnssimu::euler_to_CTM(Eigen::Vector3d eul)
{
	Eigen::Matrix3d C;
	double sin_phi, sin_theta, sin_psi, cos_phi, cos_theta, cos_psi;
	sin_phi = sin(eul(0));
	cos_phi = cos(eul(0));
	sin_theta = sin(eul(1));
	cos_theta = cos(eul(1));
	sin_psi = sin(eul(2));
	cos_psi = cos(eul(2));

	C(0,0) = cos_theta * cos_psi;
	C(0,1) = cos_theta * sin_psi;
	C(0,2) = -sin_theta;
	C(1,0) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
	C(1,1) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
	C(1,2) = sin_phi * cos_theta;
	C(2,0) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
	C(2,1) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
	C(2,2) = cos_phi * cos_theta;
	return C;
}


//initialization in ENU
Eigen::VectorXd gnssimu::initialize_enu(Eigen::VectorXd init)
{

}


//Creates skew-symmetric matrix from vec
Eigen::Matrix3d gnssimu::hatmat(Eigen::Vector3d vec)
{
	Eigen::Matrix3d A;
	A<< 0, -vec(2), vec(1),
		vec(2), 0, -vec(0),
		-vec(1), vec(0), 0;
	return A;
}


//updates 15-element IMU state
void gnssimu::updateState(Eigen::VectorXd instate)
{
	imustate=instate;
}


//returns H (see Groves 14.112)
//C_hat is best current estimate of attitude.  May be propagated from previous state (see Groves 14.100+09olk,m)
Eigen::MatrixXd gnssimu::returnH()
{
	Eigen::Matrix<double, 6, 15> H;
	Eigen::Matrix3d Hr1,Hv1,Hv5;
	Eigen::Vector3d dpsi;
	//Hr1=Eigen::Matrix3d::Identity;
	Hr1=hatmat(C_hat*lever_ab);  //see 14.112
	//need omegahat and omegamat
	Hv1=hatmat(C_hat*(hatmat(omega_hat)*lever_ab) - Omega_mat * C_hat *lever_ab);
	Hv5=C_hat*hatmat(lever_ab); //not the grouping on hatmat, Hv5 is NOT Hr1
	/*H<<Hr1, Eigen::Matrix3d::Zero, Eigen::Matrix3d::Identity*-1, Eigen::Matrix3d::Zero, Eigen::Matrix3d::Zero,
		Hv1,  Eigen::Matrix3d::Identity*-1, Eigen::Matrix3d::Zero, Eigen::Matrix3d::Zero, Hv5;*/
	H <<Hr1(0,0),Hr1(0,1),Hr1(0,2), 0,0,0, -1,0,0, 0,0,0, 0,0,0,
		Hr1(1,0),Hr1(1,1),Hr1(1,2), 0,0,0, 0,-1,0, 0,0,0, 0,0,0,
		Hr1(2,0),Hr1(2,1),Hr1(2,2), 0,0,0, 0,0,-1, 0,0,0, 0,0,0,
		Hv1(0,0),Hv1(0,1),Hv1(0,2), -1,0,0, 0,0,0, 0,0,0, Hv5(0,0),Hv5(0,1),Hv5(0,2),
		Hv1(1,0),Hv1(1,1),Hv1(1,2), 0,-1,0, 0,0,0, 0,0,0, Hv5(1,0),Hv5(1,1),Hv5(1,2),
		Hv1(2,0),Hv1(2,1),Hv1(2,2), 0,0,-1, 0,0,0, 0,0,0, Hv5(2,0),Hv5(2,1),Hv5(2,2);
	return H;
}


//initstates is 15x1: euler(0), x(0), v(0), b_a(0), b_g(0)
bool gnssimu::setInitStates(Eigen::VectorXd initstates, Eigen::Vector3d leverarm)
{
	euler(0)=initstates(0); euler(1)=initstates(0); euler(2)=initstates(2);
	x_ecef(0)=initstates(3); x_ecef(1)=initstates(4); x_ecef(2)=initstates(5);  //filtered x in ecef
	v_ecef(0)=initstates(6); v_ecef(1)=initstates(7); v_ecef(2)=initstates(8);	//filtered v in ecef
	b_a(0)=initstates(9); b_a(1)=initstates(10); b_a(2)=initstates(11);
	b_g(0)=initstates(12); b_g(1)=initstates(13); b_g(2)=initstates(14);
	imustate.resize(15);
	imustate<<0,0,0, 0,0,0, 0,0,0, b_a, b_g;
	Pimu.resize(15,15);
	lever_ab=leverarm;
	C_hat=euler_to_CTM(euler);
	return true;
	//else{return false;}
}


bool gnssimu::setInitCovariance(Eigen::MatrixXd initCov, Eigen::MatrixXd initMeasCov)
{
	Qimu.resize(15,15); Rgps.resize(6,6);
	Qimu=initCov;
	Rgps=initMeasCov;
	return true;
}


Eigen::VectorXd gnssimu::returnMeasurement()
{
	Eigen::VectorXd meas;
	meas.resize(6);
	Eigen::Vector3d rmeas=rGPS-rIMU-C_hat*lever_ab;
	Eigen::Vector3d vmeas=vGPS-vIMU-C_hat*(hatmat(omega_hat)*lever_ab)+Omega_mat*C_hat*lever_ab;
	meas<<rmeas,
		vmeas;
	return meas;
}


Eigen::Vector3d gnssimu::ecef2lla(Eigen::Vector3d ecef)
{
	Eigen::Vector3d lla;
	lla(1)=atan2(ecef(1),ecef(0));
	const double ee2 = pow(0.0818191908334158,2);
	const double sqrt1e2 = sqrt(1-ee2);
	const double R0=6378137.0;
	double beta=sqrt(pow(ecef(0),2)+pow(ecef(1),2));
	double xi=atan2(ecef(2),sqrt1e2*beta);

	lla(0)=atan(ecef(2)*sqrt1e2+ee2*R0*pow(sin(xi),3)/(sqrt1e2*(beta-ee2*R0*pow(cos(xi),3))));
	for(int i=0;i<5;i++)
	{
		xi=atan(sqrt1e2*tan(lla(0)));
		lla(0)=atan(ecef(2)*sqrt1e2+ee2*R0*pow(sin(xi),3)/(sqrt1e2*(beta-ee2*R0*pow(cos(xi),3))));
	}
	lla(2)=beta/cos(lla(0))-R0/sqrt(1-ee2*pow(sin(lla(0)),2));
	return lla;
}


//propagate with zero input and zero-mean noise
void gnssimu::kalmanPropagate(Eigen::VectorXd &xk, Eigen::MatrixXd &Pk, Eigen::MatrixXd &F,
		Eigen::MatrixXd &Q, Eigen::MatrixXd &Gamma, Eigen::VectorXd &xkbar, Eigen::MatrixXd &Pkbar)
{
	xkbar=F*xk;
	Pkbar=F*Pk*F.transpose()+Gamma*Q*Gamma.transpose();
}


//linear KF measurement udpate
void gnssimu::kalmanMeasure(Eigen::VectorXd &xkbar, Eigen::MatrixXd &Pkbar, Eigen::VectorXd &zMeas,
		Eigen::MatrixXd &H, Eigen::MatrixXd &R, Eigen::VectorXd &xkp1, Eigen::MatrixXd &Pkp1)
{
	//innovation
	Eigen::MatrixXd K;
	double xsize=xkbar.size();
	K.resize(xsize,H.rows());  //size(K)=size(H')
	K=Pkbar*H.transpose()*(H*Pkbar*H.transpose() + R).inverse();
	xkp1=xkbar+K*(zMeas-H*xkbar);
	Pkp1=(Eigen::MatrixXd::Identity(xsize,xsize)-K*H)*Pkbar*(Eigen::MatrixXd::Identity(xsize,xsize)-K*H).transpose()
		+K*R*K.transpose();  //Joseph form, see 3.58
}



//filter
/*  X1: get latitude from ecef to calculate Omega_mat
	2: find rIMU by propagating imu data forwards in time
	3: construct deltaz from rIMU and rGPS
	4: apply linear filter
	5: update delta-rIMU
	6: find true rIMU from 14.3 (similar for psi and v)
	7: send to complementary filter for imu
*/


