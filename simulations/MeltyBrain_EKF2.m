classdef MeltyBrain_EKF2 < handle
    properties
        dt %Time step
        A %Motion matrix
        B %Input matrix
        H %Jacobian matrix (for measurement)
        h %State to measurement function
        
        x %Predicted state
        predictions %Vector of all previous predicted states
        P %Covariance matrix
        
        R %Process noise covariance
        Q %Sensor noise covariance
        
        imus %Number of IMUs used
        mags %Number of magnetometers used
        beacon %Whether a beacon is used
        sensors %Total number of sensors used
        count %Internal counter to determine number of times updated
    end
    
    methods(Static)
        %% Evaluate IR beacon jacobian
        function [J] = evalIRJacobian(theta, epsilon)
            tolerance = 2e-2;
            theta = mod(theta, 2 * pi);
            if(abs(epsilon - theta) < tolerance)
                J = [-1 0];
            elseif(abs(2 * pi - epsilon - theta) < tolerance)
                J = [1 0];
            else
                J = [0 0];
            end
        end
    end
    
    methods
        %% Constructor
        %{
            Constructs a kalman filtering object with specified params
        
            dt: Time step
            Tsim: Total simulation time
            alpha: motor parameter
            beta: motor parameter
            accRad: radius from bot center to accelerometers
            wheelRad: radius of bot wheels
            imus: number of imus used by the robot
            mags: number magnetometers used by the robot
            maxBField: maximum value of the sensed magnetic field
            fieldOffset: offset of the 0 angle point from the earth's
            magnetic field
            beacon: whether or not to utilize an IR beacon
            beaconRange: range of angle in which beacon outputs a value
        %}
        function obj = MeltyBrain_EKF2(dt, Tsim, alpha, beta, dists, ...
                wheelRad, botRad, imus, mags, maxBField, ...
                fieldOffset, beacon, beaconRange)
            obj.dt = dt;
            %Define discrete time dynamics
            obj.A = [1 dt - 0.5 * alpha * dt ^ 2;
                     0 1 - alpha * dt];
            obj.B = [0.5 * dt ^ 2;
                     dt] * wheelRad * beta / botRad;
            %Define sensor counts
            obj.imus = imus;
            obj.mags = mags;
            obj.beacon = beacon;
            obj.sensors = imus + mags + beacon;
            %-angle = 2 * pi - angle
            beaconEdges = [beaconRange 2 * pi - beaconRange];
            %Define Jacobian
            obj.H = @(theta, omega) [zeros(imus, 1) 2.*dists.*omega; ...
                                     repmat([-maxBField * sin(theta - fieldOffset) 0], mags, 1); ... 
                                     repmat(obj.evalIRJacobian(theta, beaconRange), beacon, 1)];
            %Define state to observation transformation matrix
            obj.h = @(theta, omega) [dists .* (omega.^2); ...
                                     repmat([maxBField * cos(theta - fieldOffset)], mags, 1); ...
                                     repmat((mod(theta, 2 * pi) < beaconEdges(1) | mod(theta, 2 * pi) > beaconEdges(2)), beacon, 1)];
            %Initial state and covariances at 0
            obj.x = zeros(2, 1);
            obj.P = zeros(2);
            %Set process and sensor noise covariances
            obj.Q = 8e1 * eye(obj.sensors);
            obj.R = 1e2 * eye(2);
            %Preallocate predictions matrix for efficiency
            obj.predictions = zeros(2, ceil(Tsim / dt) + 2);
            obj.count = 1;
        end

        %% Update
        %{
            Executes one iteration of the kalman filter
        
            meas: measurement vector from sensors in form [meas1 ; meas2 ;
            ... ; measN]
            u: voltage input to the motor system
        %}
        function x = update(obj, meas, u)
            %Update counter
            obj.count = obj.count + 1;
            %Predict
            obj.x = obj.A * obj.x + obj.B * u;
            obj.P = obj.A * obj.P * obj.A' + obj.R;
            %Calculate Jacobian: evaluate H with angular velocity (x(2))
            theta = obj.x(1);
            omega = obj.x(2);
            H_t = obj.H(theta, omega);
            %Update
            K = obj.P * H_t' / (H_t * obj.P * H_t' + obj.Q);
            obj.x = obj.x + K * (meas - obj.h(theta, omega));
            obj.P = (eye(2) - K * H_t) * obj.P;
            %Set value for returning
            x = obj.x;
            %Append to predictions
            obj.predictions(:, obj.count) = x;
        end
        %% Plotting 
        %{
            Plots the predicted angular position of the robot
            
            figNum: the figure handle to plot on
            m, n, subplotIndex: subplot specifiers
        %}
        function [] = plotPos(obj, figNum, m, n, subplotIndex)
            T = (0 : length(obj.predictions) - 1) .* obj.dt;
            figure(figNum);
            subplot(m, n, subplotIndex);
            pos = obj.predictions(1, :) .* 180 / pi;
            plot(T, pos, 'g-', 'LineWidth', 1);
            title('Angular Position');
            xlabel('Time (s)');
            ylabel('Angular Position (deg)');
        end
        %{
            Plots the predicted angular velocity of the robot
            
            figNum: the figure handle to plot on
            m, n, subplotIndex: subplot specifiers
        %}
        function [] = plotVel(obj, figNum, m, n, subplotIndex)
            T = (0 : length(obj.predictions) - 1) .* obj.dt;
            figure(figNum);
            subplot(m, n, subplotIndex);
            plot(T, obj.predictions(2, :) .* 180 / pi, 'g-', 'LineWidth', 1);
            title('Angular Velocity');
            xlabel('Time (s)');
            ylabel('Angular Velocity (deg/s)');
        end
    end
end