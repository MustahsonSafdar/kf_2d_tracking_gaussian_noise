%% 
% Simulate Direct Radar Measurement
VERBOSE = true;
% The noise would be uniform throught the plane
UNIFORM_NOISE = true;   % Else noise would be added as in radar, more noise for further the object is
% 15 mete per second or 54 km per hour
speed_vehicle = 15; 
% [x_position, y_position_, velocity_x, velcoity_y]
birth_mean = [14;20; 0; 15];
% Number of measurement during simulation
num_measurement = 100;

%ground_truth = [timeStamp px py vx vy range angle vel];
ground_truth = zeros(num_measurement, 8);

current_time = posixtime(datetime);
% State of the system before prediction
current_state_minus_1 = birth_mean;

for i = 1:num_measurement
    current_state = F*current_state_minus_1;
    ground_truth(i,1) = current_time;
    ground_truth(i,2:5) = current_state;
    ground_truth(i,6) = sqrt(current_state(1)^2+current_state(2)^2);    % range
    ground_truth(i,7) = atan(current_state(1)/current_state(2));    % azimuth angle
    ground_truth(i,8) = birth_mean(4) * cos(ground_truth(i,7));                % radial velcoity
    current_time = current_time + dt;                               % increment in time measurement
    current_state_minus_1 = current_state;
end

%% Addition of noise
range_noise_std = 0.40; % 40cm standard deviation 
angle_noise_std = 0.052359877559830;    % 3 degree standard deviation 
velocity_noise_std = 0.25; % 0.25 meter per second standard deviation  

%Noise in X direction
noise_std_x = 2; % one meter
%Noise in Y direction
noise_std_y = 2; % one meter

if UNIFORM_NOISE
    raw_x = noise_std_x.*randn(num_measurement,1)+ground_truth(:,2);
    raw_y = noise_std_y.*randn(num_measurement,1)+ground_truth(:,3);
    raw_range = sqrt(raw_x.^2+raw_y.^2);
    raw_angle = atan(raw_x./raw_y);
    raw_velocity = velocity_noise_std*randn(num_measurement,1)+ground_truth(:,8);
    raw_measurement = [ground_truth(:,1) raw_x raw_y raw_range raw_angle raw_velocity];
    clear raw_range raw_angle raw_x raw_y raw_velocity
else
    raw_range = range_noise_std*randn(num_measurement,1)+ground_truth(:,6);
    raw_angle = angle_noise_std*randn(num_measurement,1)+ground_truth(:,7);
    raw_velocity = velocity_noise_std*randn(num_measurement,1)+ground_truth(:,8);

    raw_x = raw_range.*sin(raw_angle);
    raw_y = raw_range.*cos(raw_angle);
    raw_measurement = [ground_truth(:,1) raw_x raw_y raw_range raw_angle raw_velocity];
    clear raw_range raw_angle raw_x raw_y raw_velocity
end

%% Plotting Simulated Data 
if (VERBOSE)
    hFig = figure;
    %set(hFig, 'Position', [675 924 570 450])
    axis equal 
    grid on 
    hold on 
    xlim([0 20])
    ylim([0 120])
    xlabel('X axis')
    ylabel('Y-Axis')
    title('Raw Measurement')
    ax = gca;
    c = ax.Color;

    plot(ground_truth(:,2),ground_truth(:,3),'g.')
    plot(raw_measurement(:,2),raw_measurement(:,3),'r.')
    
end 
