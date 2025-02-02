%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

%% Serial Port Init
serialPort = "COM4";
baudRate = 115200;
serialObj = serialport(serialPort,baudRate);

% Parameters
bufferSize = 100;  % Number of data points to display at a time
gyr = zeros(bufferSize, 3); % Buffer for gyroscope data
acc = zeros(bufferSize, 3); % Buffer for accelerometer data
tcAcc = zeros(bufferSize, 3); % Buffer for tilt-compensated accelerometer data
linAcc = zeros(bufferSize, 3); % Buffer for linear acceleration in Earth frame
linVel = zeros(bufferSize, 3); % Buffer for linear velocity
linVelHP = zeros(bufferSize, 3); % Buffer for high-pass filtered velocity
linPos = zeros(bufferSize, 3); % Buffer for linear position
linPosHP = zeros(bufferSize, 3); % Buffer for high-pass filtered linear position
R = zeros(3, 3, bufferSize); % Buffer for rotation matrices
timeStamps = zeros(bufferSize, 1); % Timestamps for calculating samplePeriod
samplePeriod = 0.01; % Default sample period (in seconds)

% High-pass filter parameters
order = 1; % Filter order
filtCutOff = 0.1; % Cutoff frequency in Hz
[b, a] = butter(order, (2 * filtCutOff) / (1 / samplePeriod), 'high'); % High-pass filter coefficients

% MahonyAHRS initialization
ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

% Create figure and subplots
figure('Name', 'Real-Time Gyroscope and Accelerometer Data');
subplot(8, 1, 1); % Gyroscope subplot
hold on;
gyrPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
gyrPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
gyrPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Gyroscope (dps)');
title('Gyroscope Data');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 2); % Accelerometer subplot
hold on;
accPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
accPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
accPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Accelerometer (g)');
title('Accelerometer Data');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 3); % Tilt-compensated accelerometer subplot
hold on;
tcAccPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
tcAccPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
tcAccPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Tilt-Compensated Acceleration (m/s^2)');
title('Tilt-Compensated Accelerometer');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 4); % Linear acceleration subplot
hold on;
linAccPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
linAccPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
linAccPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Linear Acceleration (m/s^2)');
title('Linear Acceleration in Earth Frame');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 5); % Linear velocity subplot
hold on;
linVelPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
linVelPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
linVelPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Linear Velocity (m/s)');
title('Linear Velocity');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 6); % High-pass filtered linear velocity subplot
hold on;
linVelHPPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
linVelHPPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
linVelHPPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Filtered Linear Velocity (m/s)');
title('High-pass Filtered Linear Velocity');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 7); % Linear position subplot
hold on;
linPosPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
linPosPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
linPosPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
ylabel('Linear Position (m)');
title('Linear Position');
legend('X', 'Y', 'Z');
grid on;

subplot(8, 1, 8); % High-pass filtered linear position subplot
hold on;
linPosHPPlot(1) = plot(nan(bufferSize, 1), 'r', 'LineWidth', 1.5); % X-axis
linPosHPPlot(2) = plot(nan(bufferSize, 1), 'g', 'LineWidth', 1.5); % Y-axis
linPosHPPlot(3) = plot(nan(bufferSize, 1), 'b', 'LineWidth', 1.5); % Z-axis
xlabel('Sample');
ylabel('Filtered Position (m)');
title('High-pass Filtered Linear Position');
legend('X', 'Y', 'Z');
grid on;

%% Add 3D Position Plot
anim = [];


%% Real-Time Data Reading and Plotting
dataCount = 0;
while isvalid(gcf)
    rawLine = readline(serialObj);
    
    if isempty(rawLine)
        disp("Warning: No data received. Retrying...");
        continue; % Skip this loop iteration if no data is received
    end
    
    rawValues = sscanf(rawLine, '%f,%f,%f,%f,%f,%f'); % Parse the comma-separated values

    if length(rawValues) == 6
            % Increment data count
            dataCount = dataCount + 1;

            % Get timestamp and update sample period
            currentTime = now; % Get current timestamp
            if dataCount > 1
                samplePeriod = (currentTime - timeStamps(mod(dataCount - 2, bufferSize) + 1)) * 24 * 3600;
                ahrs.SamplePeriod = samplePeriod; % Update Mahony sample period
            end
            timeStamps(mod(dataCount - 1, bufferSize) + 1) = currentTime;

            % Update buffers
            gyr(mod(dataCount - 1, bufferSize) + 1, :) = rawValues(1:3) * (pi / 180); % Convert dps to rad/s
            acc(mod(dataCount - 1, bufferSize) + 1, :) = rawValues(4:6); % Accel in g

            % Update AHRS
            ahrs.UpdateIMU(gyr(mod(dataCount - 1, bufferSize) + 1, :), acc(mod(dataCount - 1, bufferSize) + 1, :));

            % Calculate rotation matrix and tilt-compensated accelerometer
            R(:, :, mod(dataCount - 1, bufferSize) + 1) = quatern2rotMat(ahrs.Quaternion)';
            tcAcc(mod(dataCount - 1, bufferSize) + 1, :) = ...
                (R(:, :, mod(dataCount - 1, bufferSize) + 1) * acc(mod(dataCount - 1, bufferSize) + 1, :)')';

            % Calculate linear acceleration in Earth frame
            linAcc(mod(dataCount - 1, bufferSize) + 1, :) = ...
                tcAcc(mod(dataCount - 1, bufferSize) + 1, :) - [0, 0, 1];
            linAcc(mod(dataCount - 1, bufferSize) + 1, :) = linAcc(mod(dataCount - 1, bufferSize) + 1, :) * 9.81;

            % Calculate linear velocity (integrate acceleration)
            if dataCount > 1
                linVel(mod(dataCount - 1, bufferSize) + 1, :) = ...
                    linVel(mod(dataCount - 2, bufferSize) + 1, :) + linAcc(mod(dataCount - 1, bufferSize) + 1, :) * samplePeriod;
            end

            % Apply high-pass filter to linear velocity
            if dataCount > bufferSize
                linVelHP = filtfilt(b, a, linVel);
            end

            % Calculate linear position (integrate velocity)
            if dataCount > 1
                linPos(mod(dataCount - 1, bufferSize) + 1, :) = ...
                    linPos(mod(dataCount - 2, bufferSize) + 1, :) + linVelHP(mod(dataCount - 1, bufferSize) + 1, :) * samplePeriod;
            end

            % High-pass filter linear position
            if dataCount > bufferSize
                linPosHP = filtfilt(b, a, linPos);
            end

            % Update plots
            set(gyrPlot(1), 'YData', gyr(:, 1));
            set(gyrPlot(2), 'YData', gyr(:, 2));
            set(gyrPlot(3), 'YData', gyr(:, 3));

            set(accPlot(1), 'YData', acc(:, 1));
            set(accPlot(2), 'YData', acc(:, 2));
            set(accPlot(3), 'YData', acc(:, 3));

            set(tcAccPlot(1), 'YData', tcAcc(:, 1));
            set(tcAccPlot(2), 'YData', tcAcc(:, 2));
            set(tcAccPlot(3), 'YData', tcAcc(:, 3));

            set(linAccPlot(1), 'YData', linAcc(:, 1));
            set(linAccPlot(2), 'YData', linAcc(:, 2));
            set(linAccPlot(3), 'YData', linAcc(:, 3));

            set(linVelPlot(1), 'YData', linVel(:, 1));
            set(linVelPlot(2), 'YData', linVel(:, 2));
            set(linVelPlot(3), 'YData', linVel(:, 3));

            set(linVelHPPlot(1), 'YData', linVelHP(:, 1));
            set(linVelHPPlot(2), 'YData', linVelHP(:, 2));
            set(linVelHPPlot(3), 'YData', linVelHP(:, 3));

            set(linPosPlot(1), 'YData', linPos(:, 1));
            set(linPosPlot(2), 'YData', linPos(:, 2));
            set(linPosPlot(3), 'YData', linPos(:, 3));

            set(linPosHPPlot(1), 'YData', linPosHP(:, 1));
            set(linPosHPPlot(2), 'YData', linPosHP(:, 2));
            set(linPosHPPlot(3), 'YData', linPosHP(:, 3));

            % Initialize the 3D plot animation after 100 samples
            if dataCount == 100
                anim = PlotAnimation('Title', 'Real-Time 3D Animation', 'AxisLength', 0.1);
                anim.init();
            end

            % Update the 3D plot dynamically
            if dataCount > 100
                anim.update(linPosHP(mod(dataCount - 1, bufferSize) + 1, :), ...
                            R(:, :, mod(dataCount - 1, bufferSize) + 1));
            end

            % Refresh the plot
            drawnow;
    end
end
