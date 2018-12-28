clc
clear all
close all

LIDAR_MODE = true;
RADAR_MODE = ~LIDAR_MODE;

% State Transition Model
dt = 1/16;

F = [1 0 dt 0; 
     0 1 0 dt;
     0 0 1 0; 
     0 0 0 1];
 
%set the process covariance matrix Q
Q_ = zeros(4, 4);
dt_2 = dt * dt;
dt_3 = dt_2 * dt;
dt_4 = dt_3 * dt;

% Generate simulated ground truth and noise realtime data
Generate_Measurement

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
            0, 1, 0, 0];
            
%measurement covariance matrix - laser
R_laser_ = [noise_std_x, 0;
            0, noise_std_y];

x_k_1 = [14; 20; 0; 15];

estimated = zeros(length(raw_measurement),2);

if (LIDAR_MODE)
    
  for n = 1: length(raw_measurement)
      [x_k_k, P_k_k, y_k_k] = update(x_k_1, raw_measurement(n,2:3), R_laser_, P_k_1, H_Lidar_);
      estimated(n,1:2) = x_k_k(1:2);

      [x_k_1, P_k_1] = predict(x_k_k, P_k_k, F);
      
  end
  
else
  
  for n = 1: length(raw_measurement)
      [x_k_k, P_k_k, y_k_k] = update(x_k_1, raw_measurement(n,2:5), R_k, P_k_1, H_Lidar_);
      
      disp (['update: ', num2str(x_k_k(1,1)), '  ',num2str(x_k_k(2,1))])
      
      [x_k_1, P_k_1] = predict(x_k_k, P_k_k,F);
      
      disp (['predict: ', num2str(x_k_1(1,1)), '  ',num2str(x_k_1(2,1))])
  end

end

plot(estimated(:,1),estimated(:,2),'k.')

rmse_estimation = CalculateRMSE(estimated, ground_truth(:,2:3));
rmse_measurement = CalculateRMSE(raw_measurement(:,2:3), ground_truth(:,2:3));
