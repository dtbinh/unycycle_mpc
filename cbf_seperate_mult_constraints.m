function out = cbf_seperate_mult_constraints(u)
%calculate the safet_certificate of the system

%u: the feedback state of the system and the obstacles 
 
p_x = u(1);
p_y = u(2);
v = u(3); 
psi = u(4); 
 
% global results; 
results = constraint_obstacles([p_x; p_y; v; psi]); 

%             h1: 1.0355
%        coef_u1: [0.9839 0.1787]
%     remaining1: 6.3702
%             h2: 1.4358
%        coef_u2: [0 1.0000]
%     remaining2: 25.0294

no_ob = size(results,2);
h = results(1).h1; 
coef_u = results(1).coef_u1; 
remaining = results(1).remaining1; 
h3_1 = results(1).h2; 
coef_u_31 = results(1).coef_u2; 
remaining_31 = results(1).remaining2; 


%%QP problem: 

%nominal input
% %very important for this problem, because there may be singular 
% %sometimes, this constant should be big enough, in order to let the solver
% %works 
% cc = 0.1; 
% if (abs(v)>cc)
%     matrix_line = [cos(psi), -v*sin(psi); sin(psi),  v*cos(psi)]; 
% else
%     matrix_line = [cos(psi), -cc*sin(psi); sin(psi),  cc*cos(psi)]; 
% end
% k1=0.8;
% k2=2*1.414*sqrt(k1);
% u_nom_lin=p_com_ddot-k1*([p_x; p_y]-p_com)-k2*([v*cos(psi); v*sin(psi)]-p_com_dot);
% 
% u_nom = u_nom_lin; 
% u_nom = inv(matrix_line)*u_nom_lin;   %feedback linearization

%in mpc, nominal input is given
u_nom = [0; 0];

%bound for control
alpha=[4; 4];

%flag to determin if use the bounded input:
flag_bound = 0; 

global beta;   %initial value is 0; 

for i_ob = 1:no_ob
    if (beta(i_ob) == 0 ) && (results(i_ob).h2 >= -0.01)
        beta(i_ob) = 1;
    elseif (beta(i_ob) == 1 ) && (results(i_ob).h2 <=  -0.1)
        beta(i_ob) = 0;
    end
end
 

%the variable determine which CBF is active now 
slack_mult = zeros(2,no_ob);
%number of the possible conditions: 
nu_combine =  1; 
order = [];
for aa = 1:no_ob
    if(beta(aa)==1)
    %if does not point to the obstacle:
        if  (results(aa).h2>= -0.01)   %pointing constraint
            slack_mult(1, no_ob) = 1;   %active
        else
            slack_mult(1, no_ob) = 0 ;
        end

        if (  results(aa).h1 >= 0)   %distance constraint
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


%if pointing to this obstacal, the distance constraint and angle constraint should be satisfied together 
% index_poining = (beta==1);
% A_n = [-results(index_poining).coef_u1; -results(index_poining).coef_u2];
% b_n = [   results(index_poining).remaining1;  results(index_poining).remaining2 ];
 

pointing_turn = slack_mult; %the turn flag determine which constraint will be added: 1-angle,  2-distance, 3-distance(no cbf is active condition)

%the minmal value and the corresponding solution, notice there may be no
%solution:
value_min = 100000000;
max_delta_lb = 1; 
x_min = [0;0];


pointing_turn = ones(2,no_ob);
global A_n_and b_n_and A_n_or b_n_or;
for i_combine = 1:nu_combine
    A_n_and = [];
    b_n_and = [];
    A_n_or = [];
    b_n_or = [];
    
    for aa = 1:no_ob
        if (beta(aa) == 0 )
% %             if pointing to the obstacle, both conditions should be satisfied 
            A_n_and = [A_n_and; -results(aa).coef_u1; -results(aa).coef_u2];
            b_n_and = [b_n_and;   results(aa).remaining1;  results(aa).remaining2 ];
        else            
            if(order(i_combine, aa) ==1)  %pointing constraint
                 A_n_or = [A_n_or; -results(aa).coef_u2; ]; 
                 b_n_or = [b_n_or;   results(aa).remaining2;  ];
            elseif (order(i_combine, aa) == 2)    %distance constraint
                 A_n_or = [A_n_or; -results(aa).coef_u1; ]; 
                 b_n_or = [b_n_or;   results(aa).remaining1 ];         
            end 
        end
 
    end
 
    
     %solve QP at the end, see if the angle constraints for multiple
     %obstacles solvable 
     H= diag([1;1]);
%     f2 = zeros(2, 1);
    f2 = -2* u_nom;  %the optimal goal is for the entire control
    optoption_1 = optimset('Display', 'off', 'TolFun', 1e-10);   %if the resulution is too high, may make the MPC not solvable, notice  
     if (size(A_n_and,1)>0)
     
%          b_n = b_n_and - A_n_and*u_nom; %the optimal goal is for the entire control
% X = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB,NONLCON,OPTIONS)

        if(flag_bound ==0)
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n, [], [], -alpha-u_nom, alpha-u_nom, [], optoption_1);
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n_and, [], [], -alpha, alpha, [], optoption_1);
%             [x, FVAL, EXITFLAG] = fmincon('fun_cos',[0;0], A_n_and, b_n_and, [], [], -alpha, alpha, 'nonlinearcondition'); 
        else
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n_and, [], [], [], [], [], optoption_1);
        end
        
%         delta_just = width_control(A_n_and, b_n_and);   %calucate the width of the feasible control  
        
        
        if (EXITFLAG<0)  
            %qp  has no solution 
            A_n_and = [];
            b_n_and = [];
            for aa = 1:no_ob
                if (beta(aa) == 0 ) && (aa == 1)
        % %             if angle CBF for multiple obstacles does not solvable,
        % then only consider the angle constraints for the main obstacle 
                    A_n_and = [A_n_and; -results(aa).coef_u1; -results(aa).coef_u2];
                    b_n_and = [b_n_and;   results(aa).remaining1;  results(aa).remaining2 ];
                elseif (beta(aa) == 0 ) && (aa ~= 1) 
                    A_n_and = [A_n_and; -results(aa).coef_u1; ];
                    b_n_and = [b_n_and;   results(aa).remaining1; ];
                end
            end

        end
     end
     
    global A_n b_n;    
    A_n = [A_n_and; A_n_or];
    b_n = [b_n_and; b_n_or];
    
     %solve QP at the end
%      H= diag([1;1]);
%      f2 = zeros(2, 1);

%     delta_just2 = width_control(A_n, b_n);   %calucate the width of the feasible control 
    
    %see if solvable 
%     if(delta_just2.max< 0 )
        %if the QP is solvable, then solve it 

%          b_n = b_n - A_n*u_nom; %the optimal goal is for the entire control
         f2 = -2* u_nom;  %the optimal goal is for the entire control
 
        if(flag_bound ==0)
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha-u_nom, alpha-u_nom, [], optoption_1);
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
%             [x, FVAL, EXITFLAG] = fmincon('fun_cos',[0;0],A_n,b_n, [], [], -alpha, alpha, 'nonlinearcondition'); 
        else
            [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], [], optoption_1);
        end
        if (EXITFLAG ~=1 )  
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

 

 

