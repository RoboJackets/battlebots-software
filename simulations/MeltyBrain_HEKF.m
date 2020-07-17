%{
    Hybrid-Extended Kalman Filter: 
        Executes prediction simulation in continuous time
        Updates with measurements in discrete time
        (Hopefully) supports various sets of sensors updating at different
        rates
    Unless otherwise stated, all angle variables, constants, inputs, 
    outputs are in RADIANS
%}
classdef MeltyBrain_HEKF < handle
    %% Class Properties
    properties
        dt      %Continuous time simulation timestep
        tUpdate %Time of last update
        updates %Total number of updates
        
        A       %Motion Matrix (2x2)
        B       %Input Matrix (2x1)
        h       %Cell array of state to observation matricies for each sensor type
        H       %Cell array of observation jacobian matricies for each sensor type
        
        x       %Predicted state
        x_all   %Matrix of all predicted states
        t_all   %Matrix of update times
        
        P       %State covariance matrix
        Q       %Process noise covariance matrix
        R       %Cell array of sensor noise covariance matricies for each sensor type
        
        imus    %Total number of IMUs used
    end 
    
    %% Static methods
    methods(Static)
        %{ 
            integrate: Numerically integrates x using euler's method
           	
            xf: Final value of x
            x0: Initial value of x
            xdot: Anonymous function handle @(x) of dx/dt
            dt: Integration timestep
            T: total time to integrate over
        %}
   
        function [xf] = integrate(x0, xdot, dt, T)
            steps = round(T / dt, 0);
            x = x0;
            for i = 1 : steps
                x = x + xdot(x) * dt;
            end
            xf = x;
        end
    end
    
    %% Methods
    methods
        %{
            Constructor: Initializes the HEKF
            
            obj: returned HEKF object
            dt: Continuous time simulation time step
            alpha: motor parameter 1
            beta: motor parameter 2
            accR: radius of accelerometers
            wheelR: radius of wheels
            botR: radius of robot
            imus: number of imus used
        %}
        function [obj] = MeltyBrain_HEKF(dt, alpha, beta, accDists, wheelR, botR, imus, mag_dir)
            obj.dt = dt;        %Set time step
            obj.tUpdate = 0;    %First update is at t = 0
            obj.updates = 0;    %No updates to start
            

            obj.A = [0 1;           %Define motion matrix
                     0 -alpha];
            obj.B = [0;             %Define input matrix
                     wheelR * beta / botR];
            %Cell array of state to observation matricies for each sensor
            %h{1} corresponds to accelerometers (a = w^2 * r)
            %h{2} corresponds to beacon (theta = theta)
            %h{3} corresponds to magnetometer "y" (B_y = sqrt(B_y^2 +
            %B_x^2)*cos(theta - <m)
            %h{4} corresponds to magnetometer "x" (B_x = sqrt(B_y^2 +
            %B_x^2)*sin(theta - <m)
            obj.h = {@(x) x(2) ^ 2 .* accDists, ...    
                	 @(x) mod(x(1), 2 * pi), ...
                     @(x, Bmag) Bmag .* cos(x(1) - mag_dir), ...
                     @(x, Bmag) Bmag .* -sin(x(1) - mag_dir)};
            %Cell array of state to observation jacobians for each sensor
            %H{1} corresponds to accelerometers
            %H{2} corresponds to beacon
            obj.H = {@(x) [zeros(imus, 1) 2 * x(2) .* accDists], ...
                     @(x) [1 0], ...
                     @(x, Bmag) [Bmag .* -sin(x(1) - mag_dir) 0], ...
                     @(x, Bmag) [Bmag .* -cos(x(1) - mag_dir) 0]};

            obj.x = [0; 0];     %Init position = init velocity = 0
            obj.x_all = [0; 0]; %Initial predictions vector
            obj.t_all = 0;      %Initial times vector (need to store time since Ts not const.)
            
            angleVar = 0.1;     %Variance of the initial angle
            processVar = 0.1;   %Variance of the model
            imuVar = 80;        %Variance of imus
            beaconVar = 1e-3;    %Variance of beacon
            magVar = 0.1;       % Variance of magnetometer 
            
            obj.P = [angleVar 0;            %Initial state covariance
                     0 0];                  %Velocity variance is 0 since w is guarunteed to be 0
            obj.Q = processVar .* eye(2);   %Process covariance matrix
            %Cell array of sensor noise covariances for each sensor
            %R{1} corresponds to accelerometers
            %R{2} corresponds to beacon
            obj.R = {imuVar * eye(imus), beaconVar, magVar, magVar};

            obj.imus = imus;    %Set number of imus used
        end
        
        %{
            update: updates the HEKF with a sensor measurement
            
            x_pred: The predicted state after the update
            obj: the HEKF object to update
            meas: the measurement with which to perform the update
            u: the control input at the update
            t: the time at which the update is performed
            sensor: the sensor set from which measurements are drawn
                'acc': update with accelerometer data
                'beacon': update with beacon data
        %}
        function [x] = update(obj, meas, u, t, sensor, Bmag)
            %Determine the index to use when extracting sensor data from cell arrays
            switch sensor
                case 'acc'
                    sensorIdx = 1;  %Accelerometers are index 1
                case 'beacon'
                    sensorIdx = 2;  %beacon is index 2
                case 'mag_x'
                    sensorIdx = 3;  % x-axis mag is 3
                case 'mag_y'
                    sensorIdx = 4;  % y-axis mag is 4
                otherwise
                    %Otherwise, throw an error
                    error('Error using HEKF.update\n%s is not a valid sensor', sensor);
            end
            
            %Prediction step
            T = t - obj.tUpdate;                            %Time since the last update
            xDot = @(x) obj.A * x + obj.B * u;              %Derivative of state as anonymous function
            PDot = @(P) obj.A * P + P * obj.A' + obj.Q;     %Derivative of covariance as anonymous function
            
            obj.x = obj.integrate(obj.x, xDot, obj.dt, T);  %Integrate state estimate
            obj.x(1) = wrapToPi(obj.x(1));                  %Constrain angular position to [-pi, pi)
            obj.P = obj.integrate(obj.P, PDot, obj.dt, T);  %Integrate covariance

            %Measurement step
            if sensorIdx == 1 || sensorIdx == 2
                Hk = obj.H{sensorIdx}(obj.x);   %Evaluate jacobian at state estimate
                hk = obj.h{sensorIdx}(obj.x);   %Evaluate expected measurement of state estimate
            else
                Hk = obj.H{sensorIdx}(obj.x, Bmag);
                hk = obj.h{sensorIdx}(obj.x, Bmag);
            end
            
            Rk = obj.R{sensorIdx};          %Select corresponding sensor noise covariance
            K = obj.P * Hk' / (Hk * obj.P * Hk' + Rk);
            err = meas - hk;                %Error between expected and actual measurements
            if(sensorIdx == 2)              %If using the beacon or mags, measurement error needs to be
                err = wrapToPi(err);        %Constrained to the range [-pi, pi)
            end
            obj.x = obj.x + K * (err);
            obj.P = (eye(2) - K * Hk) * obj.P * (eye(2) - K * Hk)' + K * Rk * K';
            
            %Update values
            obj.x(1) = wrapToPi(obj.x(1));      %Constrain angular position to [-pi, pi)
            x = obj.x;                          %Set value for returning
            obj.tUpdate = t;                    %Update last update time
            obj.updates = obj.updates + 1;      %Increment update counter
            obj.t_all = [obj.t_all t];          %Append results to cumulative vectors
            obj.x_all = [obj.x_all x];
        end
        
        %{
            Plots the predicted angular position of the robot
            
            figNum: the figure handle to plot on
            m, n, subplotIndex: subplot specifiers
        %}
        function [] = plotPos(obj, figNum, m, n, subplotIndex)
            figure(figNum);
            subplot(m, n, subplotIndex);
            pos = obj.x_all(1, :) .* 180 / pi;
            plot(obj.t_all, pos, 'g-', 'LineWidth', 1);
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
            figure(figNum);
            subplot(m, n, subplotIndex);
            plot(obj.t_all, obj.x_all(2, :) .* 180 / pi, 'g-', 'LineWidth', 1);
            title('Angular Velocity');
            xlabel('Time (s)');
            ylabel('Angular Velocity (deg/s)');
        end
    end
end