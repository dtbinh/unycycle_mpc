 %This file should be run first, in order to obtain the mex file 

%input: 
%u: 5-by-1, including: 
%time: 1-by-1, the current time, 
%x_feedback: 4-by-1, the current feedback state of the system 

close all; clear;




BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'bicycle_input'); 
    
    DifferentialState xp_dot yp_dot  psi_dot epsi  ey s L;  %sometimes, the name of the variables induces errors, I don't know why. 
    Control delta_f a_x;    
    
    %constants: 
    a = 1.41; 
    b = 1.576; 

    input1 = acado.MexInput;   %start time   
    input2 = acado.MexInput;  %final time   
    
    %%feedback states: 
    input3 = acado.MexInput;   %feedback state 1 
    input4 = acado.MexInput;   %feedback state 2 
    input5 = acado.MexInput;   %feedback state 3 
    input6 = acado.MexInput;   %feedback state 4 
    input7 = acado.MexInput;   %feedback state 5 
    input8 = acado.MexInput;   %feedback state 6 

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
    f.linkCFunction('bicycle_dynamics_L.cpp', 'dynamics');
    
    %Set up optimal control problem, 
    %start at t0, control in 40 intervals to tf
    %parameters: 
 
 
    ocp = acado.OCP(input1, input2, 50);    
    ocp.minimizeMayerTerm(L);  % minimizeLagrange is not yet implemented for matlab ode calls!
                               % but you can define another differential
                               % state to get the same effect (L)
    
    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', xp_dot ==  input3);
    ocp.subjectTo( 'AT_START', yp_dot ==  input4);
    ocp.subjectTo( 'AT_START', psi_dot ==  input5 );
    ocp.subjectTo( 'AT_START', epsi ==  input6);
    ocp.subjectTo( 'AT_START', ey ==  input7);
    ocp.subjectTo( 'AT_START', s ==  input8);
%     ocp.subjectTo( 'AT_START', px == 0);
%     ocp.subjectTo( 'AT_START', p_y == 0);
%     ocp.subjectTo( 'AT_START', v ==  0 );
%     ocp.subjectTo( 'AT_START', psi_ ==  0);
    ocp.subjectTo( 'AT_START', L ==  0.0 );
%     ocp.subjectTo( 'AT_END'  , px ==  state_tf(1) );
%     ocp.subjectTo( 'AT_END'  , p_y ==  state_tf(2) );
%     ocp.subjectTo( 'AT_END'  , v ==  state_tf(3) );
%     ocp.subjectTo( 'AT_END'  , psi_ ==  state_tf(4) ); %sometimes, the name of the variables induces errors, I don't know why. 

% %input bounded: 
    ocp.subjectTo( -1 <= delta_f <= 1);       
    ocp.subjectTo( -4 <= a_x  <= 4); 
%     ocp.subjectTo(  sqrt((s - 40)^2  + (ey -0)^2)  -1  >= 0); 
%     ocp.subjectTo( a^2+ 10^2*psi_dot^2 - (0.7*9.8)^2  <= 0); %slip constraints 
%      ocp.subjectTo(  sqrt((px - 80)^2  + (p_y -0.5)^2)  -1  >= 0); 
     
%      ocp.subjectTo(   sqrt(2*4*(sqrt((px - 80)^2  + (p_y -0.5)^2)-1)) + [px-80; p_y-0.5]'/sqrt((px - 80)^2  + (p_y -0.5)^2)*[v*cos(psi_); v*sin(psi_)] >=0 );
%      sqrt(2*alpha*(norm_deltap-Ds)) + rel_pos'/norm_deltap*rel_vel


%% slip ratio constraints: 
    ocp.subjectTo(  (yp_dot+a*psi_dot)/xp_dot - delta_f >= -0.5 );
    ocp.subjectTo(  (yp_dot+a*psi_dot)/xp_dot - delta_f <=  0.5 );
    ocp.subjectTo(  (yp_dot -b *psi_dot)/xp_dot >= -0.5 );
    ocp.subjectTo(  (yp_dot -b*psi_dot)/xp_dot <= 0.5 );


    
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
%     ocp.subjectTo(  -3.7<= ey <= 3.7); 
 
 %% if use CBF generate constraints, uncomment the following: 
     ocp.subjectTo(  coe_cbfqp_a11*delta_f + coe_cbfqp_a12*a_x -coe_cbfqp_b1  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a21*delta_f + coe_cbfqp_a22*a_x -coe_cbfqp_b2  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a31*delta_f + coe_cbfqp_a32*a_x -coe_cbfqp_b3  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a41*delta_f + coe_cbfqp_a42*a_x -coe_cbfqp_b4  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a51*delta_f + coe_cbfqp_a52*a_x -coe_cbfqp_b5  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a61*delta_f + coe_cbfqp_a62*a_x -coe_cbfqp_b6  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a71*delta_f + coe_cbfqp_a72*a_x -coe_cbfqp_b7  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a81*delta_f + coe_cbfqp_a82*a_x -coe_cbfqp_b8  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a91*delta_f + coe_cbfqp_a92*a_x -coe_cbfqp_b9  <= 0 ); 
     ocp.subjectTo(  coe_cbfqp_a101*delta_f + coe_cbfqp_a102*a_x -coe_cbfqp_b10  <= 0 ); 

    algo = acado.OptimizationAlgorithm(ocp);
    
     
    % !!
%     algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!
    
%     algo.set( 'KKT_TOLERANCE', 1e-4);

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
% out = unicycle_RUN(0, 0, 0, 0, 0);

out = bicycle_input_RUN(0, 1, ...  %start time and final time 
    1, 0, 0, 0, 0, 0,... %feedback state
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