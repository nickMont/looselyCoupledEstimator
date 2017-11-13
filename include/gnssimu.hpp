#include <iostream>
#include <eigen3/Eigen/Geometry>

class gnssimu
{
public:

    //constructor
	gnssimu();

	//working functions
	Eigen::Matrix3d euler_to_CTM(Eigen::Vector3d eul);
	Eigen::Matrix3d hatmat(Eigen::Vector3d vec);
	Eigen::Vector3d ecef2lla(Eigen::Vector3d ecef);

	//workhorses
	Eigen::VectorXd initialize_enu(Eigen::VectorXd init);
	void updateState(Eigen::VectorXd instate);
	Eigen::MatrixXd returnH();
	Eigen::VectorXd returnMeasurement();
	void processIMU(Eigen::VectorXd measIMU);
	void processGPS(Eigen::VectorXd measGPS);

	//KF
	void kalmanMeasure(Eigen::VectorXd &xkbar, Eigen::MatrixXd &Pkbar, Eigen::VectorXd &zMeas,
		Eigen::MatrixXd &H, Eigen::MatrixXd &R, Eigen::VectorXd &xkp1, Eigen::MatrixXd &Pkp1);
	void kalmanPropagate(Eigen::VectorXd &xk, Eigen::MatrixXd &Pk, Eigen::MatrixXd &F,
		Eigen::MatrixXd &Q, Eigen::MatrixXd &Gamma, Eigen::VectorXd &xkbar, Eigen::MatrixXd &Pkbar);
	void gpsImuCallback(Eigen::VectorXd zMeas);
	void gpsOnlyCallback(Eigen::VectorXd zMeas);

	//getters and setters
	Eigen::Vector3d returnXecef() {return x_ecef;}
	Eigen::Vector3d returnVecef() {return v_ecef;}
	bool setInitStates(Eigen::VectorXd initstates, Eigen::Vector3d leverarm);
	bool setInitCovariance(Eigen::MatrixXd initCov, Eigen::MatrixXd initMeasCov);

private:
	Eigen::Vector3d lever_ab, euler, rIMU, rGPS, vIMU, vGPS, b_a, b_g, omega_hat, x_ecef, v_ecef;
	Eigen::VectorXd imustate, xState, evxState;  //xstate contains error states
	double tlastgps, tlastimu;
	Eigen::Matrix3d C_hat, Omega_mat;
	Eigen::MatrixXd Qimu, Rgps, Pimu, Gamma, Fimu;
	bool initImu, initGps;
};