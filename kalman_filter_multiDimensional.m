clc
clear all
close all

LIDAR_MODE = false;
RADAR_MODE = ~LIDAR_MODE;

% Frequency of radar measurement
f_radar = 16;
% Time between two measurement
dt = 1/f_radar; % assuming constant radar measurement rate during whole period

% State Transition Model (Constat velocity Model)
F = [1 0 dt 0; 
     0 1 0 dt;
     0 0 1 0; 
     0 0 0 1];

 % Generate simulated ground truth and noise realtime data
Generate_Measurement
 
%set the process covariance matrix Q
% Note: This is not sure, where to use it, or should we use it 
Q_ = zeros(4, 4);
dt_2 = dt * dt;
dt_3 = dt_2 * dt;
dt_4 = dt_3 * dt;

Q_ = [ dt_4/4*noise_std_x,   0,                    dt_3/2*noise_std_x,  0; 
       0,                    dt_4/4*noise_std_y,   0,                   dt_3/2*noise_std_y;
       dt_3/2*noise_std_x,   0,                    dt_2*noise_std_x,    0; 
       0,                    dt_3/2*noise_std_y,   0,                   dt_2*noise_std_y];
       
% Error Covairance Matrix
P_k_1 = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];

% Measurement Model
H_Lidar_ = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, 0
            0, 0, 0, 1];
            
%measurement covariance matrix - laser
R_laser_ = [noise_std_x, 0,           noise_std_x,           0;
            0,           noise_std_y, 0,           noise_std_y;
            noise_std_x,           0,           2*noise_std_x, 0
            0,           noise_std_y,           0,           2*noise_std_y];

x_k_1 = [14; 20; 0; 15];

estimated = zeros(length(raw_measurement),4);
x_minus_1 = birth_mean(1);
y_minus_1 = birth_mean(2);

if (LIDAR_MODE)
    
  for n = 1: length(raw_measurement)
      x_pos = raw_measurement(n,2);
      y_pos = raw_measurement(n,3);
      % Velocity component estimation from position
      raw_measurement(n,4) = (x_pos - x_minus_1)/dt;
      raw_measurement(n,5) = (y_pos - y_minus_1)/dt;
      % Update Step
      [x_k_k, P_k_k, y_k_k] = update(x_k_1, raw_measurement(n,2:5), R_laser_, P_k_1, H_Lidar_);
      % Saving Estimated state for later qualitative analysis
      estimated(n,:) = x_k_k;
      % Prediction Step
      [x_k_1, P_k_1] = predict(x_k_k, P_k_k, F);  
      % Saved previous measurement to calculate velocity in next iteration
      x_minus_1 = x_pos;
      y_minus_1 = y_pos;
  end
  
% Radar Measurement Case
else
  
  for n = 1: length(raw_measurement)
      [x_k_k, P_k_k, y_k_k] = update(x_k_1, raw_measurement(n,2:5), R_k, P_k_1, H_Lidar_);
      
      disp (['update: ', num2str(x_k_k(1,1)), '  ',num2str(x_k_k(2,1))])
      
      [x_k_1, P_k_1] = predict(x_k_k, P_k_k,F);
      
      disp (['predict: ', num2str(x_k_1(1,1)), '  ',num2str(x_k_1(2,1))])
  end

end

plot(estimated(:,1),estimated(:,2),'k.')

% Root Mean Square Calculation
rmse_estimation = CalculateRMSE(estimated, ground_truth(:,2:5));
rmse_measurement = CalculateRMSE(raw_measurement(:,2:5), ground_truth(:,2:5));
