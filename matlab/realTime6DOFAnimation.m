function realTime6DOFAnimation(serialObj, batchSize, samplePeriod)
    % Initialize figure
    fig = figure('Name', '6DOF Real-Time Animation', 'NumberTitle', 'off');
    hold on;
    axis equal;
    grid on;
    view(3);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('6DOF Real-Time Animation');
    legend('Position', 'X-Axis', 'Y-Axis', 'Z-Axis');
    
    % Parameters
    axisLength = 1;
    R = eye(3);  % Initial rotation matrix
    p = zeros(batchSize, 3); % Position buffer
    
    % Initialize graphics
    positionPlot = plot3(0, 0, 0, 'ko', 'MarkerSize', 5);
    quivX = quiver3(0, 0, 0, 1, 0, 0, 'r', 'AutoScale', 'off');
    quivY = quiver3(0, 0, 0, 0, 1, 0, 'g', 'AutoScale', 'off');
    quivZ = quiver3(0, 0, 0, 0, 0, 1, 'b', 'AutoScale', 'off');
    
    % Buffers for incoming data
    gyr = zeros(batchSize, 3);
    acc = zeros(batchSize, 3);
    
    % MahonyAHRS initialization
    ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
    
    % Main loop
    while isvalid(fig)
        % Collect a batch of data
        for i = 1:batchSize
            if serialObj.NumBytesAvailable > 0
                rawLine = readline(serialObj);
                rawValues = sscanf(rawLine, '%f,%f,%f,%f,%f,%f');
                
                if length(rawValues) == 6
                    gyr(i, :) = rawValues(1:3) * (pi / 180); % Gyroscope in rad/s
                    acc(i, :) = rawValues(4:6); % Accelerometer in g
                end
            end
        end
        
        % Update AHRS and calculate new positions
        for i = 1:batchSize
            % Update AHRS with gyroscope and accelerometer data
            ahrs.UpdateIMU(gyr(i, :), acc(i, :));
            R = quatern2rotMat(ahrs.Quaternion)'; % Rotation matrix
            
            % Update position (basic example with dummy data)
            if i > 1
                p(i, :) = p(i - 1, :) + (R(1:3, 1:3) * [1; 0; 0])'; % Dummy increment
            end
        end
        
        % Update plot
        set(positionPlot, 'XData', p(:, 1), 'YData', p(:, 2), 'ZData', p(:, 3));
        set(quivX, 'XData', p(end, 1), 'YData', p(end, 2), 'ZData', p(end, 3), ...
            'UData', R(1, 1), 'VData', R(2, 1), 'WData', R(3, 1));
        set(quivY, 'XData', p(end, 1), 'YData', p(end, 2), 'ZData', p(end, 3), ...
            'UData', R(1, 2), 'VData', R(2, 2), 'WData', R(3, 2));
        set(quivZ, 'XData', p(end, 1), 'YData', p(end, 2), 'ZData', p(end, 3), ...
            'UData', R(1, 3), 'VData', R(2, 3), 'WData', R(3, 3));
        
        % Adjust axis limits dynamically
        axis([min(p(:, 1)) - axisLength, max(p(:, 1)) + axisLength, ...
              min(p(:, 2)) - axisLength, max(p(:, 2)) + axisLength, ...
              min(p(:, 3)) - axisLength, max(p(:, 3)) + axisLength]);
        
        drawnow;
    end
end