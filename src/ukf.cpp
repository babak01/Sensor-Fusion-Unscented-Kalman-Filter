#include "ukf.h"
#include "Eigen/Dense"
#define PI 3.1415926


using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(5);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 2;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 2;

	/**
	 * DO NOT MODIFY measurement noise values below.
	 * These are provided by the sensor manufacturer.
	 */

	 // Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3;

	/**
	 * End DO NOT MODIFY section for measurement noise values
	 */

	 /**
	  * TODO: Complete the initialization. See ukf.h for other member properties.
	  * Hint: one or more values initialized above might be wildly off...
   //    */


   // State dimension
	n_x_ = 5;

	// Augmented state dimension
	n_aug_ = 7;
	weights_ = VectorXd(2 * n_aug_ + 1);
	// 	Sigma point spreading parameter
	lambda_ = 3 - n_x_;

	// initially set to false, set to true in first call of ProcessMeasurement
	is_initialized_ = false;

	// predicted sigma points matrix
	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

	// 	time when the state is true, in us
	time_us_ = 0;

	NIS_radar_ = 0.0;

	NIS_laser_ = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/**
	 * TODO: Complete this function! Make sure you switch between lidar and radar
	 * measurements.
	 */
	if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) || (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)) {

		if (!is_initialized_) {
			x_ << 1, 1, 0, 0, 0;
			P_ << 0.15, 0, 0, 0, 0,
				0, 0.15, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, 1, 0,
				0, 0, 0, 0, 1;

			time_us_ = meas_package.timestamp_;

			// Lidar Measurement

			if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
				//Initilize the px and py with Lidar measurement values
				x_(0) = meas_package.raw_measurements_(0);
				x_(1) = meas_package.raw_measurements_(1);
			}
			else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
				double ro = meas_package.raw_measurements_(0);
				double phi = meas_package.raw_measurements_(1);
				double ro_dot = meas_package.raw_measurements_(2);
				x_(0) = ro * cos(phi);
				x_(1) = ro * sin(phi);
			}
			is_initialized_ = true;
			return;

		}
		double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
		time_us_ = meas_package.timestamp_;

		Prediction(dt);


		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			UpdateLidar(meas_package);
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			UpdateRadar(meas_package);
		}


	}



}

void UKF::Prediction(double delta_t) {
	/**
	 * TODO: Complete this function! Estimate the object's location.
	 * Modify the state vector, x_. Predict sigma points, the state,
	 * and the state covariance matrix.
	 */

	 // Generate Sigmas - Lesson 5: Section 15,16


	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

	MatrixXd A = P_.llt().matrixL();

	Xsig.col(0) = x_;

	for (int i = 0; i < n_x_; ++i) {
		Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
		Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
	}


	// Augmentation - Lesson 5: Section 17,18


	VectorXd x_aug = VectorXd(7);
	MatrixXd P_aug = MatrixXd(7, 7);
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;

	MatrixXd L = P_aug.llt().matrixL();

	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug_; ++i) {
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}


	// Sigma Point Prediction: Lesson 5: Section 21,22

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		double px_p, py_p;

		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
		}
		else {
			px_p = p_x + v * delta_t * cos(yaw);
			py_p = p_y + v * delta_t * sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
		py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}


	// Predicted Mean and Covariance - Lesson 5: Section 23,24

	double weight_0 = lambda_ / (lambda_ + n_aug_);
	weights_(0) = weight_0;
	for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
		double weight = 0.5 / (n_aug_ + lambda_);
		weights_(i) = weight;
	}

	x_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}

	P_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {

		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}



}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
	 * TODO: Complete this function! Use lidar data to update the belief
	 * about the object's position. Modify the state vector, x_, and
	 * covariance, P_.
	 * You can also calculate the lidar NIS, if desired.
	 */
	int n_z = 2;
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

	VectorXd z_pred = VectorXd(n_z);

	MatrixXd S = MatrixXd(n_z, n_z);

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);

		Zsig(0, i) = px;
		Zsig(1, i) = py;

	}

	z_pred.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {

		VectorXd z_diff = Zsig.col(i) - z_pred;
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_laspx_ * std_laspx_, 0,
		0, std_laspx_* std_laspy_;
	S = S + R;


	VectorXd z = meas_package.raw_measurements_;
	MatrixXd Tc = MatrixXd(n_x_, n_z);

	Tc.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		VectorXd diff_x = Xsig_pred_.col(i) - x_;

		VectorXd deff_z = Zsig.col(i) - z_pred;

		Tc += weights_(i) * diff_x * deff_z.transpose();
	}

	MatrixXd K = Tc * S.inverse();

	x_ = x_ + K * (z - z_pred);
	P_ = P_ - K * S * K.transpose();

	VectorXd deff_z = z - z_pred;
	NIS_radar_ = deff_z.transpose() * S.inverse() * deff_z;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
	 * TODO: Complete this function! Use radar data to update the belief
	 * about the object's position. Modify the state vector, x_, and
	 * covariance, P_.
	 * You can also calculate the radar NIS, if desired.
	 */

	int n_z = 3;
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

	VectorXd z_pred = VectorXd(n_z);

	MatrixXd S = MatrixXd(n_z, n_z);

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		Zsig(0, i) = sqrt(pow(px, 2) + pow(py, 2));
		Zsig(1, i) = atan2(py, px);
		Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / (sqrt(pow(px, 2) + pow(py, 2)));
	}

	z_pred.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {

		VectorXd z_diff = Zsig.col(i) - z_pred;

		while (z_diff(1) > PI) z_diff(1) -= 2. * PI;
		while (z_diff(1) < -PI) z_diff(1) += 2. * PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_radr_ * std_radr_, 0, 0,
		0, std_radphi_* std_radphi_, 0,
		0, 0, std_radrd_* std_radrd_;
	S = S + R;


	VectorXd z = meas_package.raw_measurements_;
	MatrixXd Tc = MatrixXd(n_x_, n_z);

	Tc.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		VectorXd diff_x = Xsig_pred_.col(i) - x_;
		while (diff_x(3) > PI) diff_x(3) -= 2. * PI;
		while (diff_x(3) < -PI) diff_x(3) += 2. * PI;

		VectorXd deff_z = Zsig.col(i) - z_pred;
		while (deff_z(1) > PI) deff_z(1) -= 2. * PI;
		while (deff_z(1) < -PI) deff_z(1) += 2. * PI;

		Tc += weights_(i) * diff_x * deff_z.transpose();
	}

	MatrixXd K = Tc * S.inverse();

	x_ = x_ + K * (z - z_pred);
	P_ = P_ - K * S * K.transpose();

	VectorXd deff_z = z - z_pred;
	NIS_radar_ = deff_z.transpose() * S.inverse() * deff_z;
}
