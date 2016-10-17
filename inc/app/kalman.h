#ifndef __KALMAN_H__
#define __KALMAN_H__

//#define KALMAN_FUSION

#ifndef KALMAN_FUSION

#define KALMAN_3

#ifndef KALMAN_3
typedef struct kalman_s {
        double Q; // 输入噪声协方差
        double R; // 观测噪声协方差
        double P; 
        double last_angular_vel; 
}kalman_t;
double kalman_filter(kalman_t *kalman, double angular_vel);
double kalman_filter_self_adj(kalman_t *kalman, double angular_vel);
#else
/* Kalman filter variables */
typedef struct kalman_s {

	double Q_rate; // Process noise variance for the accelerometer
	double Q_bias; // Process noise variance for the gyro bias
	double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double P[2][2]; // Error covariance matrix - This is a 2x2 matrix

}kalman_t;
double kalman_filter(kalman_t *kalman, double angular_vel);
#endif
#else
/* Kalman filter variables */
typedef struct kalman_s {

	double Q_angle; // Process noise variance for the accelerometer
	double Q_bias; // Process noise variance for the gyro bias
	double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	double angle; 
	double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double P[2][2]; // Error covariance matrix - This is a 2x2 matrix

}kalman_t;
double kalman_get_angle(kalman_t *kalman, double encoder_angle, double gyr_rate, double dt);
#endif

void kalman_init(kalman_t *kalman);

extern kalman_t kalman;
extern kalman_t fkalman;

#endif
