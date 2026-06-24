#ifndef KALMAN_H
#define KALMAN_H

class KalmanVelocityFilter{
    private:
        float Q_gyro;
        float R_encoder;
        float omega_est;
        float P;
    public:
        KalmanVelocityFilter(float Q, float R);
        float update(float gyro_z_deg_s, float encoder_omega_deg_s);
        float getEstimate();
};

#endif