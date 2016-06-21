using System;
public class Kalman {
    /* Kalman filter variables */
    private float Q_angle; // Process noise variance for the accelerometer
    private float Q_bias; // Process noise variance for the gyro bias
    private float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    private float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    private float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    private float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    private float[,] P = new float[2,2]; // Error covariance matrix - This is a 2x2 matrix

    public Kalman()
    {
        /* We will set the variables like so, these can also be tuned by the user */
        Q_angle = 0.01f;
        Q_bias = 0.03f;
        R_measure = 0.3f;

        angle = 0.0f; // Reset the angle
        bias = 0.0f; // Reset bias

        P[0,0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0,1] = 0.0f;
        P[1,0] = 0.0f;
        P[1,1] = 0.0f;
    }

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    public float getAngle(float newAngle, float newRate, float dt)
    {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0,0] += dt * (dt*P[1,1] - P[0,1] - P[1,0] + Q_angle);
        P[0,1] -= dt * P[1,1];
        P[1,0] -= dt * P[1,1];
        P[1,1] += Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        float S = P[0,0] + R_measure; // Estimate error
        /* Step 5 */
        float[] K = new float[2]; // Kalman gain - This is a 2x1 vector
        K[0] = P[0,0] / S;
        K[1] = P[1,0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        float y = newAngle - angle; // Angle difference
        /* Step 6 */
        angle += K[0] * y;
        bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        float P00_temp = P[0,0];
        float P01_temp = P[0,1];

        P[0,0] -= K[0] * P00_temp;
        P[0,1] -= K[0] * P01_temp;
        P[1,0] -= K[1] * P00_temp;
        P[1,1] -= K[1] * P01_temp;

        return angle;
    }
    public void setAngle(float a) {
        angle = a;
    }
    public float getRate() {
        return rate;
    }

    /* These are used to tune the Kalman filter */
    public void setQangle(float Q_a) {
        Q_angle = Q_a;
    }
    public void setQbias(float Q_b) {
        Q_bias = Q_b;
    }
    public void setRmeasure(float R_m) {
        R_measure = R_m;
    }
    public float getQangle() {
        return Q_angle;
    }
    public float getQbias() {
        return Q_bias;
    }
    public float getRmeasure() {
        return R_measure;
    }
}


