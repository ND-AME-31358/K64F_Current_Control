function output_data = K64_Current_Control_matlab()
%K64_Current_Control_matlab Communicate to FRDM board to start current controller
%   See parameters below
    figure(1);  clf;       % Create an empty figure to update later
    subplot(411)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (rad)');
    
    subplot(412)
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity (rad/s)');
    
    subplot(413)
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Voltage (V)');
    
    subplot(414); hold on
    h4 = plot([0],[0]);
    h4.XData = []; h4.YData = [];
    h42 = plot([0],[0], 'r');
    h42.XData = []; h42.YData = [];
    ylabel('Current (A)');
    legend("Sensed", "Desired");
    
    % This function will get called any time there is new data from
    % the FRDM board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t       = new_data(:,1); % time
        angle   = new_data(:,2); % position
        vel     = new_data(:,3); % velocity
        volt    = new_data(:,4); % voltage
        current = new_data(:,5); % current
        N       = length(angle); % number of data points
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = angle;
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = vel;
        h3.XData(end+1:end+N) = t;   % Update subplot 3
        h3.YData(end+1:end+N) = volt;
        h4.XData(end+1:end+N) = t;   % Update subplot 4
        h4.YData(end+1:end+N) = current;
        h42.XData(end+1:end+N) = t;   % Draw desired current
        h42.YData(end+1:end+N) = current_des*ones(N,1);
    end
    
    % Setup the communication between PC and FRDM board
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout

    %% Set experiment parameters
    current_des = 1.0; % Desired current
    R_motor     = 3.8; % Winding resistance of motor
    kb          = 0.0; % Motor back-EMF constant
    Kp          = 3;   % Proportional controller gain
    ExpTime     = 3 ;  % Expriement time
    
    % Pack experiment parameters
    input = [current_des R_motor kb Kp ExpTime];
    
    % Number of report data from FRDM
    output_size = 5; % time, angle, veloticy, voltage, current
    
    % Send data to FRDM board, run experiment.
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    
end
