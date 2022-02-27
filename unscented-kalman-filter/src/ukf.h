#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF
{
   public:
    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    // time when the state is true, in us
    long long time_us_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    // Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    // Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    // Radar measurement noise standard deviation radius in m
    double std_radr_;

    // Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    // Weights of sigma points
    Eigen::VectorXd weights_;

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Sigma point spreading parameter
    double lambda_;

    // number of sigma points
    int n_sig_;

    // Radar measurement noise covariance matrix
    Eigen::MatrixXd R_radar_;

    // Lidar measurement noise covariance matrix
    Eigen::MatrixXd R_lidar_;

   private:
    /**
     * Update state vector and covariance matrix using given predicted sigma points in measurement space,
     * measurement data and measurement noise
     *
     * @param z_sig metrix of predicted sigma points in measurement space
     * @param z measurement data
     * @param R measurement noise data
     * @param sensor_type type of sensor which data is being processed
     */
    void Update(const Eigen::MatrixXd& z_sig, const Eigen::VectorXd& z, const Eigen::MatrixXd& R,
                MeasurementPackage::SensorType sensor_type);

    /**
     * Generates sigma points matrix
     *
     *  @param x state vector
     *  @param P covariance matrix
     *  @param lambda sigma points spreading parameter
     *  @param n_sig number of sigma points
     */
    Eigen::MatrixXd GenerateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, double lambda, int n_sig);

    /**
     * Predict sigma points
     *
     * @param Xsig matrix with generated sigma points
     * @param delta_t Time between k and k+1 in s
     * @param n_x state vector dimension.
     * @param n_sig number of sigma points
     */
    Eigen::MatrixXd PredictSigmaPoints(const Eigen::MatrixXd& Xsig, double delta_t, int n_x, int n_sig);
};

#endif  // UKF_H