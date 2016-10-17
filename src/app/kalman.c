#include "kalman.h"

kalman_t kalman;
kalman_t fkalman;

void kalman_init(kalman_t *kalman)
{
#ifndef KALMAN_FUSION
#ifndef KALMAN_3
	// 输入噪声协方差
	kalman->Q = 0.001;
	// 观测噪声协方差
	/*kalman->R = range_variance; //0.0288;*/
	kalman->R = 0.03;
	/*kalman->P = kalman->R;*/
	kalman->P = 0;
#else
	kalman->Q_rate = 0.001f; // Process noise variance for the accelerometer
	kalman->Q_bias = 0.003f; // Process noise variance for the gyro bias
	kalman->R_measure = 0.03f; // Measurement noise variance - this is actually the variance of the measurement noise

	kalman->bias = 0.0f; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	kalman->rate = 0.0f; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	// Error covariance matrix - This is a 2x2 matrix
	// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - 
	// see: http://en.wikipedia.org/wiki/Ka  lman_filter#Example_application.2C_technical
	kalman->P[0][0] = 0.0f; 
	kalman->P[0][1] = 0.0f;
	kalman->P[1][0] = 0.0f;
	kalman->P[1][1] = 0.0f;
#endif
#else
	/* We will set the variables like so, these can also be tuned by the user */
	kalman->Q_angle = 0.001f; // Process noise variance for the accelerometer
	kalman->Q_bias = 0.003f; // Process noise variance for the gyro bias
	kalman->R_measure = 0.03f; // Measurement noise variance - this is actually the variance of the measurement noise

	kalman->angle = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	kalman->bias = 0.0f; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	kalman->rate = 0.0f; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	// Error covariance matrix - This is a 2x2 matrix
	// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - 
	// see: http://en.wikipedia.org/wiki/Ka  lman_filter#Example_application.2C_technical
	kalman->P[0][0] = 0.0f; 
	kalman->P[0][1] = 0.0f;
	kalman->P[1][0] = 0.0f;
	kalman->P[1][1] = 0.0f;
#endif
}

#ifndef KALMAN_FUSION
#ifndef KALMAN_3
double kalman_filter(kalman_t *kalman, double angular_vel) 
{
	// KF - estimate - prediction
	// F H都为1
	kalman->P = kalman->P + kalman->Q;
	
	// KF - correction
	double z = angular_vel - kalman->last_angular_vel;
	double Z = kalman->R + kalman->P;
	
	double K = kalman->P/Z;
	
	// 先验估计，G为0?
	// Xk- = last_delta
	// 后验估计
	double filted_angular_vel = kalman->last_angular_vel + K * z;

	// 更新P，P+
	/*kalman->P = kalman->P - K * kalman->P;*/
	kalman->P = (1.0 - K) * kalman->P;
	
	// KF - collect data
	kalman->last_angular_vel = angular_vel;
	return filted_angular_vel;
}
// 自适应卡尔曼滤波
double kalman_filter_self_adj(kalman_t *kalman, double angular_vel)
{
	static double alpha = 0.5;
	// KF - estimate - prediction
	// F H都为1
	kalman->P = kalman->P + alpha * kalman->Q;
	
	// KF - correction
	double z = angular_vel - kalman->last_angular_vel;
	double Z = alpha * (kalman->R + kalman->P);
	
	double K = kalman->P/Z;
	
	// 先验估计，G为0?
	// Xk- = last_delta
	// 后验估计
	double filted_angular_vel = kalman->last_angular_vel + K * z;

	// 更新P，P+
	/*kalman->P = kalman->P - K * kalman->P;*/
	kalman->P = 1.0 / alpha * (1.0 - K) * kalman->P + K * kalman->R;
	
	// KF - collect data
	kalman->last_angular_vel = angular_vel;
	return filted_angular_vel;
}
#else
double kalman_filter(kalman_t *kalman, double angular_vel) 
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information:
	// http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	kalman->P[0][0] += kalman->Q_rate;
	/*kalman->P[0][1] -= kalman->P[1][1];*/
	/*kalman->P[1][0] -= kalman->P[1][1];*/
	kalman->P[1][1] += kalman->Q_bias;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	double S = kalman->P[0][0] + kalman->R_measure; // Estimate error
	/* Step 5 */
	double K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = kalman->P[0][0] / S;
	K[1] = kalman->P[1][0] / S;

	// Calculate rate and bias - Update estimate with measurement zk (encoder_angle)
	/* Step 3 */
	double y = angular_vel - kalman->rate; // Rate difference
	/* Step 6 */
	kalman->rate += K[0] * y;
	kalman->bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance

	/* Step 7 */                                                                                                                                    
	double P00_temp = kalman->P[0][0];
	double P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;
	return kalman->rate;
}
#endif
#else
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double kalman_get_angle(kalman_t *kalman, double encoder_theta, double gyr_rate, double dt) 
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information:
	// http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	kalman->rate   = gyr_rate - kalman->bias;
	kalman->angle += dt * kalman->rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += dt * kalman->Q_bias;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	double S = kalman->P[0][0] + kalman->R_measure; // Estimate error
	/* Step 5 */
	double K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = kalman->P[0][0] / S;
	K[1] = kalman->P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (encoder_angle)
	/* Step 3 */
	double y = encoder_theta - kalman->angle; // Angle difference
	/* Step 6 */
	kalman->angle += K[0] * y;
	kalman->bias  += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance

	/* Step 7 */                                                                                                                                    
	double P00_temp = kalman->P[0][0];
	double P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;
	return kalman->angle;
}
#endif

