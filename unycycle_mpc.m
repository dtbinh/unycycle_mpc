clear;

BEGIN_ACADO;
    
    acadoSet('problemname', 'unicycle_mpc');

    %% Differential Equation
    DifferentialState p_x p_y vel psi_;
    Control           a psi_dot;
    Disturbance       w1 w2 w3 w4;
    

    % Differential Equation
    f = acado.DifferentialEquation();
    f.linkCFunction('unicycle_dynamics.cpp', 'dynamics_forcontrol');
    
    f2 = acado.DifferentialEquation();      % Use a different differential equation for simulation vs optimization
    f2.linkCFunction('unicycle_dynamics_plant.cpp', 'dynamics_forplant');
    
%% optimal control problem
% Set up the Optimal Control Problem (OCP)
                                            
    h={p_x p_y vel psi_ a psi_dot};
    Q = eye(6, 6);
    Q(1,1) = 5; 
    Q(2,3) = 5; 
    Q(3,3) = 5; 
    Q(4,4) = 5; 
    Q(5,5) = 1; 
    Q(6,6) = 1; 
    
    r = 0* ones(1,6);  % The reference
    
%     r = ref_gen();
%     r = [r, zeros(size(r,1), 2)]; 
%     r =rand(56200,30);
    
    ocp = acado.OCP(0.0, 1.0, 10);  % Start at 0s, control in 10 
                                            % intervals upto 1s (time horizon?)
    ocp.minimizeLSQ( Q, h, r );  % Minimize this Least Squares Term
    
    ocp.subjectTo( f );
    ocp.subjectTo( -400 <= a <= 400 );
    ocp.subjectTo( -4 <= psi_dot <= 4 );
    %ocp.subjectTo( w == 0.0 );
 
    
    %% SETTING UP THE (SIMULATED) PROCESS
    % SETTING UP THE (SIMULATED) PROCESS:
    identity = acado.OutputFcn();
    dynamicSystem = acado.DynamicSystem(f2, identity);    
    process = acado.Process(dynamicSystem, 'INT_RK45');% Simulates the process to be controlled 
                                                         % based on a dynamic model.
                                                         % The class Process is one of the two main 
                                                         % building-blocks within the SimulationEnvironment 
                                                         % and complements the Controller. It simulates the 
                                                         % process to be controlled based on a dynamic model.
    disturbance = [
        0.0       0.00    0.00    0.00    0.00   
        0.5       0.00    0.00    0.00    0.00 
        1.0       0.00    0.00    0.00    0.00 
        1.5       0.00     0.00    0.00    0.00 
        2.0       0.00    0.00    0.00    0.00 
        2.5       0.00    0.00    0.00    0.00 
        3.0       0.00    0.00    0.00    0.00 
        3.5       0.00   0.00    0.00    0.00 
        4.0       0.00   0.00    0.00    0.00  
        10.0      0.00   0.00    0.00    0.00 ];
%         30.0      0.00   0.00    0.00    0.00 ];
    process.setProcessDisturbance(disturbance);  %what is this? 
    
    %% SETTING UP THE MPC CONTROLLER:
    % SETUP OF THE ALGORITHM AND THE TUNING OPTIONS:
    algo = acado.RealTimeAlgorithm(ocp, 0.1);  % The class RealTimeAlgorithm serves as a user-interface 
                                                        % to formulate and
                                                        % solve model
                                                        % predictive
                                                        % control problems.
                                                        % 0.5 is the
                                                        % sampling time 
    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    algo.set('MAX_NUM_ITERATIONS', 2 );
    
    
    % SETTING UP THE NMPC CONTROLLER:
%     ref = [0.0       0.00       0.00            % Set up a given reference trajectory
%         0.5       0.00       0.00               % This trajectory is PERIODIC!
%         1.0       0.00       0.00
%         1.25      0.00       0.00
%         1.5       0.00       0.00
%         1.75      0.00       0.00
%         2.0       0.00       0.00
%         2.5       0.00       0.00
%         3.0       0.00       -0.50
%         3.5       0.00       -0.50
%         4.0       0.00       0.00];
%     %   TIME      X_REF      U_REF
    
    ref = ref_gen();
    
    reference = acado.StaticReferenceTrajectory(ref);    
    controller = acado.Controller( algo,reference );
    
    %% SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE..
    % SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE..
    sim = acado.SimulationEnvironment( 0.0,10.0,process,controller );  % Setup the closed-loop simulations of dynamic systems. 
                                                                     % Simulate from 0 to 15  sec
    
    r = zeros(1,4);  % Initilize the states
    sim.init( r );

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = unicycle_mpc_RUN();

draw;