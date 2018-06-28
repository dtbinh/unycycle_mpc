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
    
%     cc = 1; 
%     if (v>cc)
%         matrix_line = [cos(psi_), -v*sin(psi_); sin(psi_),  v*cos(psi_)]; 
%     else
%         matrix_line = [cos(psi_), -cc*sin(psi_); sin(psi_),  cc*cos(psi_)]; 
%     end
    
    
    input1 = acado.MexInput;   %start time   
    input2 = acado.MexInput;  %final time 
    
    input3 = acado.MexInput;   %feedback state 1 
    input4 = acado.MexInput;   %feedback state 2 
    input5 = acado.MexInput;   %feedback state 3 
    input6 = acado.MexInput;   %feedback state 4 

    %coe_cbf*u <= remaining
    coe_cbf1 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbf2 = acado.MexInput;  %coeffecients of the cbf constraints
    remaining = acado.MexInput;  %coeffecients of the cbf constraints
    
%      coe_cbf = acado.MexInputMatrix;  %coeffecients of the cbf constraints

 
%     input_ = acado.MexInputMatrix;      % initializeControls. Initializations are always matrices, also when they contain only one row
%     
%     %input: 
%     time = input_(1); 
%     x_feedback = [input_(2); input_(3); input_(4); input_(5)];
    
    time = input1; 
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
% 
%     state_tf = terminal_state(tf);
    
%     test = cbf_seperate([px; p_y; v; psi_]);

    i=1; 
    
    if(i>=0)
        i = i + 2 -2 *3*i;
    end
    
    teatea =v; 
    

    testtt = test_mex([v, 2, 3]);  %test only
 
 
    ocp = acado.OCP(input1, input2, 10);
    
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

%input bounded: 
    ocp.subjectTo( -4 <= a <= 4);       
    ocp.subjectTo( -4 <= psi_dot  <= 4); 
    ocp.subjectTo( a^2+ 20^2*psi_dot^2 - (0.7*9.8)^2  <= 0); 
%      ocp.subjectTo(  (px - 20)^2  + p_y^2 -1  >= 0); 


    
%CBF constraints: 
%very important for this problem, because there may be singular 
%sometimes, this constant should be big enough, in order to let the solver
%works 
 
 
     ocp.subjectTo(  coe_cbf1*a + coe_cbf2*psi_dot -remaining  <= 0 ); 
    
    
    algo = acado.OptimizationAlgorithm(ocp);
    
     
    % !!
%     algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!
    
    algo.set( 'KKT_TOLERANCE', 1e-10);

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
% out = unicycle_RUN(0, 0, 0, 0, 0);

out = unicycle_input_RUN(0, 1, 0, 0, 0, 0, 1, 1, 0);


% u_k = out.CONTROLS(1,2:end)'; 

draw_oponly;

% end