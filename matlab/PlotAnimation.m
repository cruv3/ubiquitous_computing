classdef PlotAnimation < handle
    properties (Access = private)
        FigureHandle
        PlotHandle
        QuiverHandles % Handles for the quiver plots (arrows)
        AxisLimits = [-10, 10; -10, 10; -10, 10];
        ViewAngle = [30, 20];
    end
    
    methods
        %% Constructor
        function obj = PlotAnimation(varargin)
            % Parse optional input arguments
            for i = 1:2:numel(varargin)
                switch varargin{i}
                    case 'Title', obj.ViewAngle = varargin{i+1};
                    case 'AxisLimits', obj.AxisLimits = varargin{i+1};
                end
            end
        end

        %% Initialization
        function init(obj)
            % Create figure
            obj.FigureHandle = figure('Name', '3D Animation');
            obj.PlotHandle = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5);
            hold on;

            % Create quiver handles
            obj.QuiverHandles = gobjects(3, 1); % X, Y, Z axes
            colors = ['r', 'g', 'b'];
            for i = 1:3
                obj.QuiverHandles(i) = quiver3(0, 0, 0, 0, 0, 0, colors(i), 'LineWidth', 1.5);
            end

            % Set plot properties
            grid on;
            axis equal;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            xlim(obj.AxisLimits(1, :));
            ylim(obj.AxisLimits(2, :));
            zlim(obj.AxisLimits(3, :));
            view(3);
        end

        %% Update the Plot
        function update(obj, position, rotationMatrix)
            % Update 3D position plot
            currentXData = get(obj.PlotHandle, 'XData');
            currentYData = get(obj.PlotHandle, 'YData');
            currentZData = get(obj.PlotHandle, 'ZData');
            
            set(obj.PlotHandle, ...
                'XData', [currentXData, position(1)], ...
                'YData', [currentYData, position(2)], ...
                'ZData', [currentZData, position(3)]);

            % Update quiver arrows (axes representation)
            quiverBase = position;
            for i = 1:3
                direction = rotationMatrix(:, i);
                set(obj.QuiverHandles(i), ...
                    'XData', quiverBase(1), 'YData', quiverBase(2), 'ZData', quiverBase(3), ...
                    'UData', direction(1), 'VData', direction(2), 'WData', direction(3));
            end
            
            % Refresh the plot
            drawnow;
        end
    end
end