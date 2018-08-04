%% CLF-CBF-QP geometric control of a single 3D-moving quadrotor
%Yu Yushu
% ------------------------------------------------
% The corresponding data of system trajectory
% is stored in mat file 
% ------------------------------------------------
% T: the time duration of simulation 
%
% 

function unicycle_sim(T)
clc;
close all;
if nargin < 1
    T = 20; 
end


%% Simulation based on ODE function
% control function handles
ctrl_hdl1 = @virtual_Control;
current_hdl = ctrl_hdl1;
ctrl_hdl_str = func2str(current_hdl);

% initial condition of different trials
% -------------------------------------------------
    % x(1:3): position
    % x(4:6): velocity
    % x(7:15): rotation matrix
    % x(16:18): body angular velocity
% -------------------------------------------------
% initial condition
% r0=[0;0;0];
% dr0=[1;0;0];
% ddr0=[0;0;0];
% dddr0=[0;0;0];

y0=[8;0;0;0.0; 0; 0];

% option of ode function 
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

% simulation process
disp('The simulation process has started.');
disp(strcat('Controller: ', ctrl_hdl_str));
disp('---------------------------------------------');
tspan = [0 T];
% [t1, y1] = ode15s(@quad_3d_ode, tspan, y0, options, current_hdl);
[t1, y1] = ode45(@quad_3d_ode, tspan, y0, options, current_hdl);

% [t1, y1] = self_solverdynamics(@quad_3d_ode, tspan, y0, options, current_hdl);

% save all the data into .mat file  
save('sim_data.mat');
disp('Data successfully stored!');


end


% geometric backstepping by Taeyoung Lee
function [u] = virtual_Control(t, y, trajd)
%commands
% rhat=[trajd(1);trajd(2);trajd(3)];
 
%sensed 
% x_feedback = [y(1);y(2);y(3)];

%time horizon: 
horizon = 1; 

input = [t; y];

coef_.A = zeros(10,2);
coef_.B = zeros(10,1);

% %using the m-file: 
% coef_ = cbf_seperate(y);   %single constraints 
% 
% % using the mex-file: (should run unicycle_c_seperate.m firstly)
% out = unicycle_input_RUN(t, t+horizon, y(1), y(2), y(3), y(4), ...
%     coef_(1), coef_(2), coef_(3), ... %row 1 
%     coef_(1), coef_(2), coef_(3), ... %row 2 
%     coef_(1), coef_(2), coef_(3), ... %row 3 
%     coef_(1), coef_(2), coef_(3), ... %row 4 
%     coef_(1), coef_(2), coef_(3), ...  %row 5 
%     coef_(1), coef_(2), coef_(3), ... %row 6 
%     coef_(1), coef_(2), coef_(3), ... %row 7
%     coef_(1), coef_(2), coef_(3), ... %row 8 
%     coef_(1), coef_(2), coef_(3), ... %row 9
%     coef_(1), coef_(2), coef_(3));

% only static obstacles: 
% coef_ = cbf_seperate_mult_constraints(y); 

% can address dynamics obstacles: 
% coef_ = cbf_seperate_mult_dynamic_constraints([y; t]); 

% if (coef_.C == 1)
%     %the feasible control space is empty
%     u = [-4; 0]; 
% else
% %     using the mex-file: (should run unicycle_c_seperate.m firstly)
    out = bicycle_input_RUN(t, t+horizon, ...
        y(1), y(2), y(3), y(4), y(5), y(6),  ...
        coef_.A(1,1), coef_.A(1,2), coef_.B(1), ... %row 1 
        coef_.A(2,1), coef_.A(2,2), coef_.B(2),  ... %row 2 
        coef_.A(3,1), coef_.A(3,2), coef_.B(3),  ... %row 3 
        coef_.A(4,1), coef_.A(4,2), coef_.B(4),  ... %row 4 
        coef_.A(5,1), coef_.A(5,2), coef_.B(5),  ...  %row 5 
        coef_.A(6,1), coef_.A(6,2), coef_.B(6), ... %row 6 
        coef_.A(7,1), coef_.A(7,2), coef_.B(7),  ... %row 7
        coef_.A(8,1), coef_.A(8,2), coef_.B(8), ... %row 8 
        coef_.A(9,1), coef_.A(9,2), coef_.B(9),  ... %row 9
        coef_.A(10,1), coef_.A(10,2), coef_.B(10));

 
    if (out.CONVERGENCE_ACHIEVED ==1)
        u = out.CONTROLS(1,2:end)';   %the MPC control 
    else
        u = [0; -4];  %the MPC solver does not converge
    end
% end
 
    
end


%% Ode Function of this vehicle
function [dy] = quad_3d_ode(t, y, ctrl_hdl)

dy =zeros(6,1);

% get the current reference and control input 
trajd = traj_gen(t);
u = feval(ctrl_hdl, t, y, trajd);

%% unicycle model: 
% % convert the current state 
% px = y(1); 
% py = y(2); 
% v = y(3); 
% psi = y(4);
% 
% %update the system dynamics 
% dy(1) = v*cos(psi); 
% dy(2) = v*sin(psi);
% dy(3) = u(1);
% dy(4) = u(2);  


%% 2018-08-04, bicycle model: 
%input signal 
delta_f = u(1);   %steering angle 
a_x = u(2);    %acc 

if (y(1)<= 0)
    y(1) = 1e-4;
end 

%states:
xp_dot = y(1);  %lateral speed
yp_dot = y(2);  %longitudinal speed
psi_dot = y(3); 
epsi = y(4);
ey= y(5);  %lateral position
s = y(6);  %logitudinal position 

%constants: 
a = 1.41; 
b = 1.576; 
mu =0.5; 
Fzf = 21940/2; 
Fzr = 21940/2; 
cf = 65000; 
cr = 65000; 
m = 2194; 
Iz = 4770; 
psi_dot_com = 0;
p =Iz/(m*b);

%state equation: 
f_x = [ yp_dot*psi_dot;... 
    -2*(cf+cr)/(m*xp_dot)*yp_dot-2*(a*cf-b*cr)/m/xp_dot*psi_dot-xp_dot*psi_dot; ...
     -2*(a*cf-b*cr)/Iz/xp_dot*yp_dot-2*(a*a*cf+b*b*cr)/Iz/xp_dot*psi_dot;...
     psi_dot - psi_dot_com;...
     yp_dot*cos(epsi) + xp_dot*sin(epsi); ...
     xp_dot*cos(epsi)-yp_dot*cos(epsi)];
 
g_x = [0, 1; ...
    2*cf/m, 0; ...
    2*a*cf/Iz, 0;...
    0, 0;...
    0, 0;...
    0, 0];

dy = f_x + g_x*u;

% -----------------------------------------------
% check simulation time for stability property
debug = 1;
if debug == 1
    disp(['The current time is ', num2str(t)]);
end
% -----------------------------------------------
end


function out = traj_gen(t)

% r=7.5;
% pos = [r*cos(1/r*t); r*sin(1/r*t); 0];
% vel = [-sin(1/r*t); cos(1/r*t); 0];
% acc = [-1/r*cos(1/r*t); -1/r*sin(1/r*t); 0];
% dacc =[1/r^2*sin(1/r*t); -1/r^2*cos(1/r*t); 0]; 
% d2acc = [1/r^3*cos(1/r*t); 1/r^3*sin(1/r*t); 0];


v=10;
p = [v*t; 0]; 
psi = 0;

out=[p; v; psi];
end


 
