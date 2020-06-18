classdef MeltyBrain_EKF < handle
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

        imus %Number of imus used
        count %Internal counter to determine number of times updated
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
        %}
        function obj = MeltyBrain_EKF(dt, Tsim, alpha, beta, accRad, wheelRad, imus)
            obj.dt = dt;
            %Define discrete time dynamics
            obj.A = [1 dt - 0.5 * alpha * dt ^ 2;
                     0 1 - alpha * dt];
            obj.B = [0.5 * dt ^ 2;
                     dt] * wheelRad * beta / accRad;
            %Create state to observation transformation matrix handles
            obj.imus = imus;
            obj.H = @(omega) repmat([0 2 * accRad * omega], imus, 1); 
            obj.h = @(omega) repmat([accRad * omega ^ 2], imus, 1);
            %Initial state and covariances at 0
            obj.x = zeros(2, 1);
            obj.P = zeros(2);
            %Set process and sensor noise covariances
            obj.Q = 1e-1 * eye(imus);
            obj.R = 1e3 * eye(2);
            %Preallocate predictions matrix for efficiency
            obj.predictions = zeros(2, floor(Tsim / dt));
            obj.count = 0;
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
            omega = obj.x(2);
            H_t = obj.H(omega);
            %Update
            K = obj.P * H_t' / (H_t * obj.P * H_t' + obj.Q);
            obj.x = obj.x + K * (meas - obj.h(omega));
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
            pos = mod(pos, 360);
            plot(T, pos, 'g--', 'LineWidth', 2);
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
            plot(T, obj.predictions(2, :) .* 180 / pi, 'g--', 'LineWidth', 2);
            title('Angular Velocity');
            xlabel('Time (s)');
            ylabel('Angular Velocity (deg/s)');
        end
    end
end