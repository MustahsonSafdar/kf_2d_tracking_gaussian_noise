
% Sensor Measurement Model : H_Sensor_
% Sensor Measurement Noise : R_k
% Current Measurement : z_k
% Predicted State : x_k_1
% Predicted Process Covariance Matrix : P_k_1

function [x_k_k, P_k_k, y_k_k] = update(x_k_1, z_k, P_k_1, R_k, H_Sensor_)
    if (length(z_k) == 3)
        z_pred = RadarCartesianToPolar(x_k_1);
        y_k = z_k' - z_pred;        
        S_k = R_k + H_Sensor_*P_k_1*transpose(H_Sensor_);
        % Kalman Gain calculation
        K_k = P_k_1*transpose(H_Sensor_)*inv(S_k);
        % Posteriro Estimation
        x_k_k = x_k_1 + K_k*y_k;
        % State Covariance Matrix Calculation
        P_k_k = (eye(4,4) - K_k*H_Sensor_) * P_k_1 * transpose((eye(4,4)-K_k*H_Sensor_)) + K_k*R_k*transpose(K_k);
        y_k_k = z_k' - H_Sensor_*x_k_k;
    else
        y_k = z_k' - H_Sensor_*x_k_1;
        S_k = R_k + H_Sensor_*P_k_1*transpose(H_Sensor_);
        % Kalman Gain calculation
        K_k = P_k_1*transpose(H_Sensor_)*inv(S_k);
        % Posteriro Estimation
        x_k_k = x_k_1 + K_k*y_k;
        % State Covariance Matrix Calculation
        P_k_k = (eye(4,4) - K_k*H_Sensor_) * P_k_1 * transpose((eye(4,4)-K_k*H_Sensor_)) + K_k*R_k*transpose(K_k);
        y_k_k = z_k' - H_Sensor_*x_k_k;
    end
end
