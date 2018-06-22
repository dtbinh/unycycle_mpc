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
    T = 15; 
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

y0=[0;0;0;0.1];

% option of ode function 
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3);

% simulation process
disp('The simulation process has started.');
disp(strcat('Controller: ', ctrl_hdl_str));
disp('---------------------------------------------');
tspan = [0 T];
% [t1, y1] = ode15s(@quad_3d_ode, tspan, y0, options, current_hdl);
[t1, y1] = ode45(@quad_3d_ode, tspan, y0, options, current_hdl);

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

%using the m-file: 
% u = unicycle_c(input);    


%using the mex-file: (should run unicycle_c_seperate.m firstly)
out = unicycle_input_RUN(t, t+horizon, y(1), y(2), y(3), y(4));
u = out.CONTROLS(1,2:end)'; 
 
    
end


%% Ode Function of this vehicle
function [dy] = quad_3d_ode(t, y, ctrl_hdl)
dy = zeros(4, 1);

 
% convert the current state 
px = y(1); 
py = y(2); 
v = y(3); 
psi = y(4);



% get the current reference and control input 
trajd = traj_gen(t);
u = feval(ctrl_hdl, t, y, trajd);


%update the system dynamics 
dy(1) = v*cos(psi); 
dy(2) = v*sin(psi);
dy(3) = u(1);
dy(4) = u(2);  

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


 
