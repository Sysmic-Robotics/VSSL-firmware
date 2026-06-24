#include "kalman.h"

KalmanVelocityFilter::KalmanVelocityFilter(float Q, float R){
    Q_gyro = Q;
    R_encoder = R;
    omega_est = 0.0f;
    P = 1.0f; // Inicialización de la incertidumbre
}

float KalmanVelocityFilter::update(float gyro_z_deg_s, float encoder_omega_deg_s){

    omega_est = gyro_z_deg_s; // Predicción usando la velocidad del giroscopio
    P = P + Q_gyro; // Actualización de la incertidumbre

    float K = P / (P + R_encoder); // Ganancia de Kalman
    omega_est = omega_est + K * (encoder_omega_deg_s - omega_est); // Actualización de la estimación
    P = (1.0f - K) * P; // Actualización de la incertidumbre

    return omega_est;
}

float KalmanVelocityFilter::getEstimate(){
    return omega_est;
}