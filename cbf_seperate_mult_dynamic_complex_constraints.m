function out = cbf_seperate_mult_dynamic_complex_constraints(u)
%calculate the safet_certificate of the system

%u: the state of the system, used to calculates the constraints in the mpc 

%feedback states:
xp_dot = u(1);  %lateral speed
yp_dot = u(2);  %longitudinal speed
psi_dot = u(3); 
epsi = u(4);
ey= u(5);  %lateral position
s = u(6);  %logitudinal position 

time = u(7); 

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



%in MPC, set nominal input as zeros
u_nom = [0; 0];
%bound for control
alpha=[1.0; 4];
%flag to determin if use the bounded input:
flag_bound = 0; 



%% for dynamic obstacles:  
global results_2; 
% results_2 = constraint_obstacles_dynamics([p_x; p_y; v; psi], time); 
results_2 = constraint_obstacles_dynamics_complex([xp_dot; yp_dot; psi_dot; epsi; ey; s], time ); 

no_ob = size(results_2,2); 
global beta_2;   %initial value is 0; 

Ds0 = 1;
Ds_angle = 1.2; 

for i_ob = 1:no_ob
    %justify when jump: 
%     Ds = 2; 
    theta_d_big = asin((Ds_angle)/results_2(i_ob).norm_relpos) - asin( Ds0 /results_2(i_ob).norm_relpos);
%     theta_d_big =0.1;
    theta_d_small = theta_d_big/200000; 
%     theta_d_small = -0.01; 
    if (beta_2(i_ob) == 0 ) && (results_2(i_ob).h_angle_fix > -theta_d_small)
        beta_2(i_ob) = 1;
    elseif (beta_2(i_ob) == 1 ) && (results_2(i_ob).h_angle_fix <=  -theta_d_big/10000000)
        beta_2(i_ob) = 0;
    end
end
  
shreshold_movingangle = 1e-20; 

%the variable determine which CBF is active now 
slack_mult = zeros(2,no_ob);
%number of the possible conditions: 
nu_combine =  1; 
order = [];
for aa = 1:no_ob
    if(beta_2(aa)==1) && (results_2(aa).h_angle_moving<=shreshold_movingangle)
%         Ds = 2; 
        theta_d_big = asin((Ds_angle)/results_2(aa).norm_relpos) - asin( Ds0 /results_2(aa).norm_relpos);
%         theta_d_big = 0.1;
        theta_d_small = theta_d_big/20;
        %if does not point to the obstacle:
    
        if  (results_2(aa).h_angle_fix>= -theta_d_big)   %pointing constraint
            slack_mult(1, no_ob) = 1;   %active
        else
            slack_mult(1, no_ob) = 0;
        end

        if (  results_2(aa).h_dis >= 0)   %distance constraint
            slack_mult(2, no_ob) = 1;   %active
        else
            slack_mult(2, no_ob) = 0 ;
        end 
        if (  slack_mult(1, no_ob) == 0) && (  slack_mult(2, no_ob) == 0)
            slack_mult(2, no_ob) =1; %at least one should be 1 
        end
    
        nu_combine = nu_combine*sum(slack_mult(:,no_ob));   %number of the possible conditions
    
        row_order = size(order,1);
        if (row_order == 0)
            row_order = 1;
        end
        
        if(slack_mult(1, no_ob) == 1) && (slack_mult(2, no_ob) ==1)
            order = [order, ones(row_order, 1); order, 2*ones(row_order, 1)];  %record the place 
        elseif(slack_mult(1, no_ob) == 1) && (slack_mult(2, no_ob) == 0)
            order = [order, ones(row_order, 1)];  %record the place 
        elseif(slack_mult(1, no_ob) == 0) && (slack_mult(2, no_ob) == 1)
            order = [order, 2*ones(row_order, 1)];  %record the place 
        end
    else
        row_order = size(order,1);
        if (row_order == 0)
            row_order = 1;
        end
        order = [order, zeros(row_order, 1)];
    end
end

%the minmal value and the corresponding solution, notice there may be no
%solution:
value_min = 100000000;
x_min = [0;0];

for i_combine = 1:nu_combine
    A_n_and = [];
    b_n_and = [];
    A_n_or = [];
    b_n_or = [];
    
    for aa = 1:no_ob
        if (beta_2(aa) == 0 ) 
% %             if pointing to the obstacle, both conditions should be satisfied 
            A_n_and = [A_n_and;  results_2(aa).A_n_angle_fix; results_2(aa).A_n_dis];
            b_n_and = [b_n_and;   results_2(aa).B_n_angle_fix;  results_2(aa).B_n_dis ]; 
        elseif (results_2(aa).h_angle_moving > shreshold_movingangle)
            %if the vehicle and the obstacle have been in the opposite
            %direction
            A_n_and = [A_n_and;  results_2(aa).A_n_angle_fix; results_2(aa).A_n_angle_moving];
            b_n_and = [b_n_and;   results_2(aa).B_n_angle_fix;  results_2(aa).B_n_angle_moving ];
            
        else            
            if(order(i_combine, aa) ==1)  %pointing constraint
                 A_n_or = [A_n_or;  results_2(aa).A_n_angle_fix;   ]; 
                 b_n_or = [b_n_or;   results_2(aa).B_n_angle_fix;  ];
            elseif (order(i_combine, aa) == 2)    %distance constraint
                 A_n_or = [A_n_or;  results_2(aa).A_n_dis;   ]; 
                 b_n_or = [b_n_or;   results_2(aa).B_n_dis; ];        
            end 
        end
 
    end
   
     %solve QP at the end, see if the angle constraints for multiple
     %obstacles solvable 
     H= diag([1;1]);
     f2 = -2* u_nom;  %the optimal goal is for the entire control
     optoption_1 = optimset('Display', 'off', 'TolFun', 1e-10);
     if (size(A_n_and,1)>0)
        if(flag_bound ==0)
             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n_and, [], [], -alpha, alpha, [], optoption_1);
        else
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n_and, [], [], [], [], [], optoption_1);
        end        
%         delta_just = width_control(A_n_and, b_n_and);   %calucate the width of the feasible control  
        if (EXITFLAG<0)  
            %qp  has no solution 
            A_n_and = [];
            b_n_and = [];
            for aa = 1:no_ob
                if (beta_2(aa) == 0 ) && (aa == 1)
        % %             if angle CBF for multiple obstacles does not solvable,
        % then only consider the angle constraints for the main obstacle 
                    A_n_and = [A_n_and;  results_2(aa).A_n_angle_fix;  results_2(aa).A_n_dis];
                    b_n_and = [b_n_and;   results_2(aa).B_n_angle_fix;  results_2(aa).B_n_dis ];
                elseif (beta_2(aa) == 0 ) && (aa ~= 1) 
                    A_n_and = [A_n_and;  results_2(aa).A_n_dis; ];
                    b_n_and = [b_n_and;   results_2(aa).B_n_dis; ];
                elseif (results_2(aa).h_angle_moving > shreshold_movingangle) 
                    %if the vehicle and the obstacle have been in the opposite
                    %direction
                    A_n_and = [A_n_and;  results_2(aa).A_n_angle_fix; results_2(aa).A_n_angle_moving];
                    b_n_and = [b_n_and;   results_2(aa).B_n_angle_fix;  results_2(aa).B_n_angle_moving ];
                end
            end

        end
     end
     
    global A_n b_n;    
    A_n = [A_n_and; A_n_or];
    b_n = [b_n_and; b_n_or];
    
     %solve QP at the end

%     delta_just2 = width_control(A_n, b_n);   %calucate the width of the feasible control 
    
    %see if solvable 
%     if(delta_just2.max< 0 )
        %if the QP is solvable, then solve it 
        if(flag_bound ==0)
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha-u_nom, alpha-u_nom, [], optoption_1);
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
        else
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], [], optoption_1);
        end
        if (EXITFLAG < 0)  
            %qp  has no solution 
            FVAL = 100000000; %no solution, set a very big value 
        end

        if (FVAL < value_min)
            value_min = FVAL; %update
            x_min = x; 
            A_min = A_n;  %the coefficient 
            b_min = b_n;  %the coefficient 
        end   
 
%     end
    
%     if(delta_just2.max < max_delta_lb) 
%         max_delta_lb = delta_just2.max; 
%         x_min = x; 
%     end
end

 
%assume the output of the A is 10-by-2 matrix, B is 10-by-1 vector: 
%pre allocation the value of A and B, with the pre-allocated value, it does
%not affect the solution 
out.A = zeros(10,2);
out.B = 100*ones(10,1);   
out.C=0;
% % the output: 
if (value_min~=100000000) 
% if (max_delta_lb<0)
    %select the minimal solution 
    %output the coefficient  
     out.A(1:size(A_min,1),:) = A_min; 
     out.B(1:size(A_min,1)) = b_min; 
else
%     %no solution exsits, brake  
%     out = [-alpha(1); 0];
    out.A(1:4,:) = [-1, 0; 1, 0; 0, 1; 0, -1];
    out.B(1:4) = [alpha(1); -alpha(1); 0; 0];
    out.C = 1; %warning, no feasible solution 
end


