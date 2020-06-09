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
    end
    methods
        %% Constructor
        %{
            Constructs a kalman filtering object with specified params
        
            dt: Time step
            alpha: motor parameter
            beta: motor parameter
            botRad: radius from bot center to wheel
            wheelRad: radius of bot wheels
            imus: number of imus used by the robot
        %}
        function obj = MeltyBrain_EKF(dt, alpha, beta, botRad, wheelRad, imus)
            obj.dt = dt
            obj.A = [1 dt - 0.5 * alpha * dt ^ 2;
                     0 1 - alpha * dt];
            obj.B = [0.5 * dt ^ 2;
                     dt] * wheelRad * beta / botRad;
            obj.H = @(omega) repmat([0 2 * wheelRad * omega], imus, 1); 
            obj.h = @(omega) repmat([wheelRad * omega ^ 2], imus, 1);
            obj.x = zeros(2, 1);
            obj.P = zeros(2);
            obj.Q = 1e-6 * eye(2);
            obj.R = zeros(2);
            obj.imus = imus;
        end
        %% Update
        %{
            Executes one iteration of the kalman filter
        
            meas: measurement vector from sensors in form [meas1 ; meas2 ;
            ... ; measN]
            u: voltage input to the motor system
        %}
        function x = update(obj, meas, u)
            %Predict
            obj.x = obj.A * obj.x + obj.B * u;
            obj.P = obj.A * obj.P * obj.A' + obj.R;
            
            %Calculate Jacobian: evaluate H with angular velocity (x(2))
            omega = obj.x(2);
            H_t = obj.H(omega);
            
            %Update
            K = obj.P * H_t' * inv(H_t * obj.P * H_t' + obj.Q);
            obj.x = obj.x + K * (meas - obj.h(omega));
            obj.P = (eye(2) - K * H_t) * obj.P;
            
            %Set value for returning
            x = obj.x;
            
            %Append to predictions
            obj.predictions = [obj.predictions x];
        end
        %% Plotting 
        function [] = plotResults(obj)
            T = (0 : length(obj.predictions) - 1) .* obj.dt;
            figure(1);
            plot(T, obj.predictions(1, :));
            title('EKF Angular Position');
            xlabel('Time (s)');
            ylabel('Angular Position (rad)');
            figure(2);
            plot(T, obj.predictions(2, :));
            title('EKF Angular Velocity');
            xlabel('Time (s)');
            ylabel('Angular Velocity (rad/s)');
        end
    end
end