 %This file should be run first, in order to obtain the mex file 

%input: 
%u: 5-by-1, including: 
%time: 1-by-1, the current time, 
%x_feedback: 4-by-1, the current feedback state of the system 

close all; clear;




BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'unicycle_input'); 
    
    DifferentialState px p_y v psi_ L;  %sometimes, the name of the variables induces errors, I don't know why. 
    Control a psi_dot;
    
    input1 = acado.MexInput;   %start time   
    input2 = acado.MexInput;  %final time 
    
    input3 = acado.MexInput;   %state 1 
    input4 = acado.MexInput;   %state 2 
    input5 = acado.MexInput;   %state 3 
    input6 = acado.MexInput;   %state 3 


 
%     input_ = acado.MexInputMatrix;      % initializeControls. Initializations are always matrices, also when they contain only one row
%     
%     %input: 
%     time = input_(1); 
%     x_feedback = [input_(2); input_(3); input_(4); input_(5)];
    
%     time = input1; 
%     x_feedback = [input2; input3; input4; input5];

 
    
    % Set default objects
    f = acado.DifferentialEquation();
    f.linkCFunction('unicycle_dynamics_L.cpp', 'dynamics');
    
    %Set up optimal control problem, 
    %start at t0, control in 40 intervals to tf
    %parameters: 
%     time_horizon = 0.5;
%     t0 = time;  
%     tf= time+time_horizon; 

%     state_tf = terminal_state(tf);
    
    ocp = acado.OCP(input1, input2, 20);
    
    ocp.minimizeMayerTerm(L);  % minimizeLagrange is not yet implemented for matlab ode calls!
                               % but you can define another differential
                               % state to get the same effect (L)
    
    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', px ==  input3);
    ocp.subjectTo( 'AT_START', p_y ==  input4);
    ocp.subjectTo( 'AT_START', v ==  input5 );
    ocp.subjectTo( 'AT_START', psi_ ==  input6);
%     ocp.subjectTo( 'AT_START', px == 0);
%     ocp.subjectTo( 'AT_START', p_y == 0);
%     ocp.subjectTo( 'AT_START', v ==  0 );
%     ocp.subjectTo( 'AT_START', psi_ ==  0);
    ocp.subjectTo( 'AT_START', L ==  0.0 );
%     ocp.subjectTo( 'AT_END'  , px ==  state_tf(1) );
%     ocp.subjectTo( 'AT_END'  , p_y ==  state_tf(2) );
%     ocp.subjectTo( 'AT_END'  , v ==  state_tf(3) );
%     ocp.subjectTo( 'AT_END'  , psi_ ==  state_tf(4) ); %sometimes, the name of the variables induces errors, I don't know why. 
    ocp.subjectTo( -100 <= a <= 100);   
    ocp.subjectTo( -40 <= psi_dot <= 40); 
    
    
    algo = acado.OptimizationAlgorithm(ocp);
    % !!
    % algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!
    
    algo.set( 'KKT_TOLERANCE', 1e-4 );

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
% out = unicycle_RUN(0, 0, 0, 0, 0);

out = unicycle_input_RUN(0, 1, 0, 0, 0, 0);


% u_k = out.CONTROLS(1,2:end)'; 

draw_oponly;

% end