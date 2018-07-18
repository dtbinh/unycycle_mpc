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
    
    %%feedback states: 
    input3 = acado.MexInput;   %feedback state 1 
    input4 = acado.MexInput;   %feedback state 2 
    input5 = acado.MexInput;   %feedback state 3 
    input6 = acado.MexInput;   %feedback state 4 

    %%maximum 5 obstacles, each has 2 constraints, then Au<b, A is 10-by-2,
    %b is 2-by-1, totally 30 inputs 
    %coe_cbf*u <= remaining
    coe_cbfqp_a11 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a12 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b1 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a21 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a22 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b2 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a31 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a32 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b3 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a41 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a42 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b4 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a51 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a52 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b5 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a61 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a62 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b6 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a71 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a72 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b7 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a81 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a82 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b8 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a91 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a92 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b9 = acado.MexInput;  %coeffecients of the cbf constraints
    
    coe_cbfqp_a101 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_a102 = acado.MexInput;  %coeffecients of the cbf constraints
    coe_cbfqp_b10 = acado.MexInput;  %coeffecients of the cbf constraints
    
 
 
    
    % Set default objects
    f = acado.DifferentialEquation();
    f.linkCFunction('unicycle_dynamics_L.cpp', 'dynamics');
    
    %Set up optimal control problem, 
    %start at t0, control in 40 intervals to tf
    %parameters: 
 
 
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
%     ocp.subjectTo(  sqrt((px - 80)^2  + (p_y -0.5)^2)  -1  >= 0); 
%     ocp.subjectTo( a^2+ 10^2*psi_dot^2 - (0.7*9.8)^2  <= 0); %slip constraints 
%      ocp.subjectTo(  sqrt((px - 80)^2  + (p_y -0.5)^2)  -1  >= 0); 
     
%      ocp.subjectTo(   sqrt(2*4*(sqrt((px - 80)^2  + (p_y -0.5)^2)-1)) + [px-80; p_y-0.5]'/sqrt((px - 80)^2  + (p_y -0.5)^2)*[v*cos(psi_); v*sin(psi_)] >=0 );
%      sqrt(2*alpha*(norm_deltap-Ds)) + rel_pos'/norm_deltap*rel_vel


    
%CBF constraints: 
%very important for this problem, because there may be singular 
%sometimes, this constant should be big enough, in order to let the solver
%works 

%% if do not use CBF to generate constraints, uncomment the following: 
% pos_ob_array_pre(:,1) = [50; -1.1];
% pos_ob_array_pre(:,2) = [50; 1.1];
% pos_ob_array_pre(:,3) = [51.8; -2.8];
% pos_ob_array_pre(:,4) = [51.8;  0.1];
% pos_ob_array_pre(:,5) = [51.8;  2.8];
%     ocp.subjectTo(  sqrt((px - 50)^2  + (p_y +1.1)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((px - 50)^2  + (p_y- 1.1)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((px - 51.8)^2  + (p_y + 2.8)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((px - 51.8)^2  + (p_y - 0.0)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((px - 51.8)^2  + (p_y - 2.8)^2)  -1  >= 0); 
%       
    ocp.subjectTo(  -3.7<= p_y <= 3.7); 
 
 %% if use CBF generate constraints, uncomment the following: 
     ocp.subjectTo(  coe_cbfqp_a11*a + coe_cbfqp_a12*psi_dot -coe_cbfqp_b1  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a21*a + coe_cbfqp_a22*psi_dot -coe_cbfqp_b2  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a31*a + coe_cbfqp_a32*psi_dot -coe_cbfqp_b3  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a41*a + coe_cbfqp_a42*psi_dot -coe_cbfqp_b4  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a51*a + coe_cbfqp_a52*psi_dot -coe_cbfqp_b5  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a61*a + coe_cbfqp_a62*psi_dot -coe_cbfqp_b6  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a71*a + coe_cbfqp_a72*psi_dot -coe_cbfqp_b7  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a81*a + coe_cbfqp_a82*psi_dot -coe_cbfqp_b8  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a91*a + coe_cbfqp_a92*psi_dot -coe_cbfqp_b9  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a101*a + coe_cbfqp_a102*psi_dot -coe_cbfqp_b10  <= 0 ); 

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

out = unicycle_input_RUN(0, 1, ...  %start time and final time 
    0, 0, 0, 0, ... %feedback state
    1, 1, 0, ... %first row of cbf qp constraints 
    1, 1, 0, ... %2 row of cbf qp constraints 
    1, 1, 0, ... %3 row of cbf qp constraints 
    1, 1, 0, ... %4 row of cbf qp constraints 
    1, 1, 0, ... %5 row of cbf qp constraints 
    1, 1, 0, ... %6 row of cbf qp constraints 
    1, 1, 0, ... %7 row of cbf qp constraints 
    1, 1, 0, ... %8 row of cbf qp constraints 
    1, 1, 0, ... %9 row of cbf qp constraints 
    1, 1, 0 ); %10 row of cbf qp constraints 
    


% u_k = out.CONTROLS(1,2:end)'; 

draw_oponly;

% end