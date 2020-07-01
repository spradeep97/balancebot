

%==============================================================================================================================
% SCRIPT FOR SIMULATING THE BALANCEBOT FOR ROB550: ROBOTICS LAB AT THE UNIVERSITY OF MICHIGAN
%
% THIS WILL RUN A SIMULATION USING PID CONTROL FOR THE TILT ANGLE (SEC4) FOLLOWED BY THE BODY ANGLE (SEC6).MAKES SURE TO CHECK 
% ANY SIMULATION VARIABLES DEFINED BEFORE EACH SECTION
%
% THIS SCRIPT IS DEPLOYED STARTING WINTER 2020. IT IS STILL IN BETA MODE AND SUSCEPTIBLE TO FIXES/CHANGES. YOU CAN MAKE CHANGES 
% AS YOU DEEM NECESSARY
%
%==============================================================================================================================

%% Section0 - Clear Terminal Output, Figures, and All Variables

clc
close all
clear 

global Model % We will use the Model global struct to store simulation data and avoid unecessarily large amount of arguments to
             % our simulation function
             
% Hint for new matlab users on windows: Ctrl+Enter to run a highlighted section. Home->Preferences->Keyboard->Shortcuts for more.

%% Section1 - Get physical constants

% TODO : FILL IN WITH PARAMETERS FROM YOUR ROBOT
%--------------------------------------------------------------------------------------------------------------------------------
DT      = .01;          % 100hz controller loop
m_w     = .0922;          % mass of one wheel in Kg MEASURED
m_b     = 1.0927;          % balancebot body mass without wheels (TO BE DETERMINED)
R       = .0413;          % radius of wheel in m MEASURED
L       = .100;         % center of wheel to Center of mass (TO BE DETERMINED)
I_r    = 0.00513;
%I_r     = 0.004;        % Inertia of body about center (not wheel axis) Kg*m^2 (TO BE DETERMINED)
g       = 9.81;         % gravity m/s^2
R_gb    = 20.4;         % gearbox ratio
tau_s   = 0.5740;         % Motor output stall Torque @ V_nominal (TO BE DETERMINED)
w_free  = 45.3042;           % Motor output free run speed @ V_nominal (TO BE DETERMINED)
V_n     = 12.0;         % motor nominal drive voltage
%I_gb = 100e-05;
I_gb = 5.5055e-06;     % inertial of motor armature and gearbox (TO BE DETERMINED)
bot_height  = 21;                % height of the robot in !![cm]!! (TO BE DETERMINED MORE ACCURATELY from wheel center to top)

% add inertia of wheels modeled as disks and times two for both sides
I_w = 2 * (I_gb+(m_w*R^2)/2);
%--------------------------------------------------------------------------------------------------------------------------------


%% Section2 - Get model constants and state space matrices 

% We will use the state space for simulation even if this is the PID script The constants are obtained from the lecture slides 
% on the Balancebot Model Derivation. 

a1 = I_w + (m_b + m_w)*R^2;
a2 = m_b * R * L;
a3 = I_r + m_b*L^2;
a4 = m_b * g * L;


% motor model equation used: t = e*u - f*w

b1 = 2 * tau_s;        % stall torque of two motors
b2  = b1 / (w_free);   % constant provides zero torque @ free run

%simplifications

c1 = 1 - (a2^2)/(a1*a3);
c2 = 1 + (a2/a3);
c3 = 1 + (a2/a1);

A = [0 1 0 0;
     a4/(a3*c1) -(b2*c3)/(a3*c1) 0 (b2*c3)/(a3*c1);
     0 0 0 1;
     -(a2*a4)/(a1*a3*c1) (b2*c2)/(a1*c1) 0 -(b2*c2)/(a1*c1)];
B = [0 -(b1*c3)/(a3*c1) 0 (b1*c2)/(a1*c1) ]';
C = [1 0 0 0;
     0 0 1 0];
D = [0 0]';

% Transfer continuous time system to discrete time system Discrete simulation allow us to factor in the effect of the control 
% freq under which we run our control on the Beaglebone.

sys = ss(A,B,C,D);
sys_d = c2d(sys,DT,'zoh');  % If interested, read on state transition from continuous to discrete
Model.A_d = sys_d.A;
Model.B_d = sys_d.B;
Model.C_d = sys_d.C;
Model.D_d = sys_d.D;
Model.DT = DT;
Model.bot_height = bot_height;
Model.radius = R;

%% Section3 -Inner loop plant including motor dynamics

% First transfer function of interest, G1(s). Controlling the output of this transfer function allows us to control tilt. Input
% is a reference body angle. The control loop will compose the inner loop.

numG1 = [-b1*(a1+a2), 0];
denG1 = [(a1*a3 - a2^2), b2*(a1+a3+2*a2), -a1*a4, -a4*b2];
% Make TF monic 
numG1 = (1/denG1(1))*numG1; 
denG1 = (1/denG1(1))*denG1;
% Make the TF
G1 = tf(numG1,denG1)
disp('G1 poles')
root_den_innerloop = roots(denG1)
disp('G1 zeros')
root_num_innerloop = roots(numG1)

% TODO : LOOK AT THE ROOT LOCUS OF THE INNER PLANT. THINK ABOUT THE POSITION OF POLES AND ZEROS. THIS HELPS IN MAKING THE TUNING 
% A BIT MORE EDUCATED. YOU MIGHT WANT TO TAKE A LOOK AGAIN WITH THE LOOP CLOSED AND YOUR PID INCLUDED TO SEE HOW YOU AFFECT IT 
%--------------------------------------------------------------------------------------------------------------------------------
figure(1)
rlocus(G1)
title('Root Locus of G1(s)')
%--------------------------------------------------------------------------------------------------------------------------------

%% Section4 -Inner Loop Body Angle Controller


tr1 = 0.05;       % desired rise time of inner loop
wc1 = 1.8/tr1;    % rule of thumb crossover freq

% TODO : TUNE !
%--------------------------------------------------------------------------------------------------------------------------------
gain1 = 1.0;      % Optional: You can use the gain to gain the net output of your PID. Equivalent to imposing one more P layer.
% trying to tune
% Kp1 = -2.2;
% Ki1 = -13.5;
% Kd1 = -0.05;
% original
% Kp1 = -3;
% Ki1 = -14;
% Kd1 = -0.05;
Kp1 = -4.2;
Ki1 = -21;
Kd1 = -0.21;
% Kp1 = -4.179;
% Ki1 = -23.35;
% Kd1 = -0.1187;
tf1   = 1/wc1;
%--------------------------------------------------------------------------------------------------------------------------------

D1  = gain1*pid(Kp1, Ki1, Kd1, 1/wc1);
OL1 = minreal(D1*G1);
CL1 = feedback(OL1,1); % This closes the loop. Optional: Look at its root locus

% Optional: Use "step(CL1)" to simulate the loop for a step input in continous time domain. May serve as a reference.

% Discrete-time simulation for inner loop

sim_time = 5;                   % sim time [s] Default = 5
theta_ref = 0.0;                % Reference Body angle value [rad]
theta_0 = 0;                    % Initial Theta [rad]
x0 = [theta_0;0;0;0];           % Initial States
theta = theta_0;                % Initializing Theta
y_vec = [];                     % Storing Y vector history (Det. by the C matrix in Sec2. Default: Y only has theta and phi
u_vec = [];                     % Storing control input (duty cycle) history
time_vec = [];                  % Storing time vector for plotting
ref_vec = [];                   % Store reference value at each step for plotting, here it will be a fixed line. More imp in sec5

saturate = false;               % Impose duty cycle staturation between -1 and 1, as well as simulate the ground. Setting to false
                                % should get you similar results to the optional continuous time simulation if you. For a more 
                                % accurate simulation however, you need it true

filter_derr = true;             % Filter the output of the derivative controller of your PID. This dampens sharp high frequency
                                % behaviour that results of calculating derivatives discretely
                                
draw_robot = true;              % Draw the robot to visually see the graphs you're getting in action. This option is here 
                                % in case drawing slows sim on some machines.

% Needed inits for recording history
prev_err = 0;                   % Error Value from pervious loop
int_err = 0;                    % Integral of the error
prev_derr = 0;                  % Need to store deriv error for filtering


for time = 0:DT:sim_time
    
    err = theta_ref - theta;                    % Error Val of current loop   
    d_err = (err-prev_err)/DT;                  % Derivative of error
    int_err = int_err + 0.5*(err+prev_err)*DT;  % Integrated error
    
    if filter_derr
        d_err = (1-DT/tf1)* prev_derr+ (DT/tf1)*d_err; % Discrete low-pass filtering for the derivative output
    end
    u = Kp1*err + Ki1*int_err + Kd1*d_err;      % Get control inpt
    [x,y,u] = sim_bot(x0, u, saturate);         % Need sim_bot.m in the same directory as this script. 
    theta = y(1);                               % Read theta, on the actual robot this is equivalent to a gyro reading of tilt     
    
    % Update initial and prev vars for the next loop
    x0 = x;
    prev_err = err; 
    prev_derr = d_err;
    % Build the plotting and drawing vectors
    time_vec = [time_vec time];
    y_vec = [y_vec, y];
    u_vec = [u_vec, u];
    ref_vec = [ref_vec, [theta_ref;0]];
    
    % Plot data and draw robot (if set to true). The number is the figure number
    plot_bot(y_vec,u_vec,ref_vec,time_vec,draw_robot,2)

end


%% Section5 - Outer Loop Plant

% Second transfer function of interest, G2(s). Controlling the output of this transfer function allows us to control wheel 
% position and consequently robot position. Input is a reference wheel pos angle, output is the reference angle of the inner loop
% which as before, will calculate the duty cycle and ultimately drive things. This should clear the intuition on why we refer 
% to them as inner and outer loops.

numG2 = [-(a2+a3), 0, a4];
denG2 = [a1+a2, 0,0];
% make monic
numG2 = (1/denG2(1))*numG2;
denG2 = (1/denG2(1))*denG2;
% make the TF
G2=tf(numG2,denG2)
disp('G2 poles')
roots(denG2)
disp('G2 zeros')
roots(numG2)

% TODO : AGAIN, WE LOOK AT THE ROOT LOCUS
%--------------------------------------------------------------------------------------------------------------------------------
figure(3)
rlocus(G2)
title('Root Locus of G2(s)')
%--------------------------------------------------------------------------------------------------------------------------------

%% Section6 - Outer Loop Position Controller for Full System Control

% NOTE: WE RE-INTRODUCE THE DEFS FOR CONTROLLER 1 TO ALLOW MORE CONVENIENCE OF TUNING IN A SINGLE SPOT. BY DEFAULT WE SET THEM TO
% WHAT YOU OBTAINED IN Section4, HOWEVER YOU CAN TUNE THEM HERE

tr2 = 0.35;     % desired rise time of outer loop
wc2 = 1.8/tr2;  % rule of thumb crossover freq


% TODO : TUNE !
%--------------------------------------------------------------------------------------------------------------------------------
% Inner - Keep results from Sec4 or tune again here if needed.
gain1 = gain1;
Kp1 = Kp1;
Ki1 = Ki1;
Kd1 = Kd1;
tf1 = tf1;

% Outer - Tune!
gain2 = 1.0; % Like before, optional.
% Kp2 = 0.04;
% Ki2 = 0.005;
% Kd2 = 0.015;
% Kp2 = 0.01;
% Ki2 = 0.001;
% Kd2 = 0.015;
Kp2 = 0.0084;
Ki2 = 0.0168;
Kd2 = 0.00105;
tf2 = 1/wc2;
%--------------------------------------------------------------------------------------------------------------------------------


D2  = gain2*pid(Kp2, Ki2, Kd2, 1/wc2)
OL2 = minreal(D2*CL1*G2);
CL2 = feedback(OL2,1); % Optional : Look at root-locus
figure(5)
rlocus(CL2)
title('Root Locus of CL2(s)')
step(CL2)   



% Discrete-time simulation for outer loop

sim_time = 5;                   % sim time [s] Default = 5
phi_ref = 0;                    % Reference wheel position [rad]
phi_0 = 0;                      % Initial wheel pos [rad]
theta_0 = 0;                    % Initial Theta [rad]
x0 = [theta_0;0;phi_0;0];       % Initial States
phi = phi_0;                    % Initialize Wheel pos
theta = theta_0;                % Initialize Body angle
y_vec = [];                     % Storing Y vector history (Det. by the C matrix in Sec2. Default: Y only has theta and phi
u_vec = [];                     % Storing control input (duty cycle) history
time_vec = [];                  % Storing time vector for plotting
ref_vec = [];                   % Store reference value at each step for plotting. Very important here to see ref body angle

saturate = true;                % Impose duty cycle staturation between -1 and 1, as well as simulate the ground. Setting to false
                                % should get you similar results to the optional continuous time simulation if you. For a more 
                                % accurate simulation however, you need it true
                                
filter_derr = true;             % Filter the output of the derivative controller of your PID. This dampens sharp high frequency
                                % behaviour that results of calculating derivatives discretely

draw_robot = true;              % Draw the robot to visually see the graphs you're getting in action. This option is here 
                                % in case drawing slows sim on some machines.
                                
% Needed inits for recording history
prev_phi_err = 0;               % Phi Error from pervious loop
int_phi_err = 0;                % Phi Error Integral
prev_theta_err = 0;             % Theta Error from pervious loop
int_theta_err = 0;              % Theta Error Integral

% Need to store deriv error for filtering
prev_phi_derr = 0;
prev_theta_derr = 0;

for time = 0:DT:sim_time
    
    % Outer Controller
    err_phi = phi_ref - phi;
    d_err_phi = (err_phi-prev_phi_err)/DT;
    int_phi_err = int_phi_err + 0.5*(err_phi+prev_phi_err)*DT;
    if filter_derr
        d_err_phi = (1-DT/tf2)* prev_phi_derr+ (DT/tf2)*d_err_phi;          %Filtering
    end    
    theta_ref = Kp2*err_phi + Ki2*int_phi_err + Kd2*d_err_phi;
    prev_phi_derr = d_err_phi;
    
    % Inner Controller
    err_theta = theta_ref - theta;                   
    d_err_theta = (err_theta-prev_theta_err)/DT;      
    int_theta_err = int_theta_err + 0.5*(err_theta+prev_theta_err)*DT;
    if filter_derr
        d_err_theta = (1-DT/tf1)* prev_theta_derr+ (DT/tf1)*d_err_theta;    %Filtering
    end   
    u = Kp1*err_theta + Ki1*int_theta_err + Kd1*d_err_theta;
    prev_theta_err = d_err_theta;
    
    [x,y,u] = sim_bot(x0, u, saturate);   % Need the sim_bot.m in the directory
    theta = y(1);     
    phi = y(2);
    x0 = x;
    prev_phi_err = err_phi;
    prev_theta_err = err_theta;
    
    time_vec = [time_vec time];
    y_vec = [y_vec, y];
    u_vec = [u_vec, u];
    ref_vec = [ref_vec, [theta_ref; phi_ref]];
    plot_bot(y_vec,u_vec,ref_vec,time_vec,draw_robot,2)

end

