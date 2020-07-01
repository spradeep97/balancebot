function [x,y,u] = sim_bot(x0, u, saturate)

    % Simulates a discrete step in the states of the robot 
    % wrt to the provided control input u and initial state x0. 

    global Model
    
    if(saturate)
        % Saturate duty cycle for more accurate simulation
        u = max(-1,min(u,1));
        
        % Saturate theta for hitting the ground for more accurate simulation
        theta_lim = pi/2 + atan2(Model.radius, Model.bot_height/100); %Since the height in cm, convert back
    end
    x = Model.A_d*x0 + Model.B_d*u;
    y = Model.C_d*x0 + Model.D_d*u;
    
    if(saturate && abs(x(1)) >= theta_lim)
        x(2) = 0; % Body angle is zero when hitting ground
        x(1) = max(-theta_lim,min(theta_lim,x(1)));
    end
    
end

