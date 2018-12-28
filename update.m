
% Sensor Measurement Model : H_Sensor_
% Sensor Measurement Noise : R_k
% Current Measurement : z_k
% Predicted State : x_k_1
% Predicted Process Covariance Matrix : P_k_1

function [x_k_k, P_k_k, y_k_k] = update(x_k_1, z_k, R_k, P_k_1, H_Sensor_)
    
    y_k = z_k' - H_Sensor_*x_k_1;
    S_k = R_k + H_Sensor_*P_k_1*transpose(H_Sensor_);
    K_k = P_k_1*transpose(H_Sensor_)*inv(S_k);
    x_k_k = x_k_1 + K_k*y_k;
    P_k_k = (eye(4,4) - K_k*H_Sensor_) * P_k_1 * transpose((eye(4,4)-K_k*H_Sensor_)) + K_k*R_k*transpose(K_k);
    y_k_k = z_k' - H_Sensor_*x_k_k;
    
end
