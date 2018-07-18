function out = cbf_seperate_mult_dynamic_constraints(u)
%calculate the safet_certificate of the system

%u: the state of the system, used to calculates the constraints in the mpc 

 

p_x = u(1);
p_y = u(2);
v = u(3); 
psi = u(4); 
time = u(5); 

%reference trajectory
% p_com = [u(13);u(14)];
% p_com_dot = [u(15);u(16)];
% p_com_ddot = [u(17);u(18)]; 
% time = u(19); 

%modified on July, 4th, 2018, put the constraints computation into a
%function 

% %%relative to obstacles:
% pos_ob = [u(7); u(8)];
% vel_ob = [u(9); u(10)];
% acc_ob = [u(11); u(12)];
%  
% p_x_dot = v*cos(psi);
% p_y_dot = v*sin(psi); 
% 
% rel_pos = [p_x; p_y] - pos_ob;
% rel_vel = [p_x_dot; p_y_dot] - vel_ob; 
%  
% Ds = 1; 
% 
% %%only distance constraints, not good: 
% K = [1, 2*1.414*1];
% %  K = [1.2,3];
%  
% h_disonly = (norm(rel_pos))^2 - Ds^2; 
% h_dot = 2*rel_pos'*rel_vel;
%  
%  %coefficient of the input 
% coef_u_ponly = [2*rel_pos(1)*cos(psi)+2*rel_pos(2)*sin(psi), -2*rel_pos(1)*v*sin(psi)+2*rel_pos(2)*v*cos(psi)]; 
% %remaining: 
% remaining_ponly    = 2*(norm(rel_vel))^2 - 2* rel_pos'*acc_ob + K*[h_disonly; h_dot];
% 
% %very important for this problem, because there may be singular 
% %sometimes, this constant should be big enough, in order to let the solver
% %works 
% cc = 1; 
% if (abs(v)>cc)
%     matrix_line = [cos(psi), -v*sin(psi); sin(psi),  v*cos(psi)]; 
% else
%     matrix_line = [cos(psi), -cc*sin(psi); sin(psi),  cc*cos(psi)]; 
% end
%     
% 
% %%consider velocity: 
% Ds = 1; 
% alpha = 4; 
% 
% gamma = 5; 
% 
% norm_deltap = (norm(rel_pos)); 
% if (norm_deltap < Ds) 
%     norm_deltap = Ds; 
% end
% 
% norm_rel_p_dot = 0.5/norm_deltap*2*rel_pos'*rel_vel; 
% 
% h = sqrt(2*alpha*(norm_deltap-Ds)) + rel_pos'/norm_deltap*rel_vel; 
% coef_u =  rel_pos'/norm_deltap* matrix_line;
% % coef_u =  rel_pos'/norm_deltap; 
% 
% remaining   = 0.5*1/sqrt(2*alpha*norm_deltap-Ds) *2*alpha* norm_rel_p_dot +  (rel_vel'*norm_deltap - rel_pos'*norm_rel_p_dot)/norm_deltap^2*rel_vel...
%     + gamma*h; 
% 
% 
% %%velocity constraints: 
% h2= v; 
% coef_u2 = [1, 0];
% remaining2   = 2*h2; 
%  
% 
% %%road side constraints 
% Ds2 = 2; 
% K2 = [1, 2*1.414*1]; 
% K2 = [1.2, 3.5];
%  
% h2 = -(p_y)^2 + Ds2^2; 
% h2_dot = -2*p_y'*v*sin(psi);
%  
% %coefficient of the input 
% coef_u2 = [0,  -2*p_y]; 
% %remaining: 
% remaining2   =   - 2* (v*sin(psi))^2  + K2*[h2; h2_dot];
% % coef_u2 = zeros(1, 2);
% % remaining2 = -10;
% 
% 
%  
% %%CBF 3, for fixed obstacle: 
% %1st constraint, the angle constraint 
% Ds = 1.2;
% angle_ob =  atan(rel_pos(2)/rel_pos(1));  %the angle of the vector from vehicle to obstacle 
% angle_rel = psi - angle_ob; 
% h3_1 =  norm(rel_pos)* abs(sin(angle_rel)) - Ds; 
% %another way to define the relative angle: 
% angle_rel = acos((-rel_pos'*[cos(psi); sin(psi)]) /norm(rel_pos));
% 
% % if (angle_rel ==0)
% %     angle_rel = 1e-5;
% % end
% 
% h3_1 =  norm(rel_pos)* abs(sin(angle_rel)) - Ds;
% 
% pos_ob_x = pos_ob(1);
% pos_ob_y = pos_ob(2); 
% 
% if(pos_ob_y==0)    
%     pos_ob_y = 1e-4;
% end
% 
% %compute from the symbols: 
% % L_f_h3_1 = v*sin(psi)*((abs(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) - (sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))/((p_x - pos_ob_x)*((p_y - pos_ob_y)^2/(p_x - pos_ob_x)^2 + 1))) + v*cos(psi)*((abs(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(p_y - pos_ob_y))/((p_x - pos_ob_x)^2*((p_y - pos_ob_y)^2/(p_x - pos_ob_x)^2 + 1))); 
% % L_g_h3_1 =[ 0, sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)]; 
% %compute from the symbols: 
% L_f_h3_1 = v*sin(psi)*((abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*((2*sin(psi)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2*(2*p_y - 2*pos_ob_y))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))/(2*abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2))) + v*cos(psi)*((abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*((2*cos(psi)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2*(2*p_x - 2*pos_ob_x))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^2))/(2*abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)));
% L_g_h3_1 = [ 0, (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x)))/(abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))]; 
% 
% ratio = Ds/norm(rel_pos);
% if ratio>=1
%     ratio = 0.999;
% end
% h3_1 = angle_rel - asin(ratio); 
% L_f_h3_1 = v*cos(psi)*((cos(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2) + (Ds*(2*p_x - 2*pos_ob_x))/(2*(1 - Ds^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2))^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))) + v*sin(psi)*((sin(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2) + (Ds*(2*p_y - 2*pos_ob_y))/(2*(1 - Ds^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2))^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))); 
% L_g_h3_1 = [ 0, (cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x))/(((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2))]; 
% 
% ratio_dot = -Ds/ (norm(rel_pos))^2 *norm_rel_p_dot; 
% asin_dot = 1/(1 - ratio^2)^(1/2); 
% 
% L_f_h3_1 = (v*cos(psi)*(cos(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2) + (v*sin(psi)*(sin(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2); 
% L_g_h3_1 = [ 0, (cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x))/(((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2))]; 
% L_f_h3_1 = L_f_h3_1 -asin_dot*ratio_dot; 
%  
% % if(angle_rel<=  pi/2)
%     coef_u_31 = L_g_h3_1; 
%     gamma_31 = 3; %constant 
%     remaining_31 = L_f_h3_1 + gamma_31*h3_1; 
% % else %if the relative angle is more than pi/2, do not need to constraint 
% %     h3_1 = pi; 
% %     coef_u_31 = [0, 0]; 
% %     remaining_31 = 100;
% % end
% 
% %2nd constraint, the distance constraint 
% a_m = 4;  %maximum acc 
% l = v^2/(2*a_m);
% h3_2 =  norm(rel_pos)-Ds - l; 
% 
% %compute from the symbols: 
% L_f_h3_2 = (v*cos(psi)*(p_x - pos_ob_x))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (v*sin(psi)*(p_y - pos_ob_y))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2); 
% L_g_h3_2 = [ -v/a_m, 0]; 
% 
% coef_u_32 = L_g_h3_2; 
% gamma_32= 3; %constant 
% remaining_32 = L_f_h3_2 + gamma_32*h3_2; 
% %from manual calculation: 
% % remaining_32 = norm_rel_p_dot + gamma_32*h3_2; 
% 
% 
% %compute from the symbols: 
% h3_3 =  sqrt(2*a_m*(norm_deltap-Ds)) + rel_pos'/norm_deltap*[v*cos(psi); v*sin(psi)]; 
% L_f_h3_3 = v*cos(psi)*((v*cos(psi))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (2^(1/2)*a_m*(2*p_x - 2*pos_ob_x))/(4*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(-a_m*(Ds - ((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)))^(1/2)) - (v*sin(psi)*(conj(p_y) - conj(pos_ob_y))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)) + (v*cos(psi)*(p_x*abs(pos_ob_x)^2 - pos_ob_x*abs(p_x)^2)*(2*p_x - 2*pos_ob_x))/(2*p_x*pos_ob_x*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))) + v*sin(psi)*((v*sin(psi))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (2^(1/2)*a_m*(2*p_y - 2*pos_ob_y))/(4*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(-a_m*(Ds - ((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)))^(1/2)) - (v*cos(psi)*(conj(p_x) - conj(pos_ob_x))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)) + (v*sin(psi)*(p_y*abs(pos_ob_y)^2 - pos_ob_y*abs(p_y)^2)*(2*p_y - 2*pos_ob_y))/(2*p_y*pos_ob_y*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))); 
% L_g_h3_3 = [ (cos(psi)*(conj(p_x) - conj(pos_ob_x)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (sin(psi)*(conj(p_y) - conj(pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2), (v*cos(psi)*(conj(p_y) - conj(pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - (v*sin(psi)*(conj(p_x) - conj(pos_ob_x)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)]; 
% 
% coef_u_33 = L_g_h3_3; 
% gamma_33= 2; %constant 
% remaining_33 = L_f_h3_3 + gamma_33*h3_3; 



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
u_nom = [0; 0];

%bound for control
alpha=[4; 4];

%flag to determin if use the bounded input:
flag_bound = 0; 



%% static obstacles: 

% global results; 
% results = constraint_obstacles([p_x; p_y; v; psi]); 

%             h1: 1.0355
%        coef_u1: [0.9839 0.1787]
%     remaining1: 6.3702
%             h2: 1.4358
%        coef_u2: [0 1.0000]
%     remaining2: 25.0294

% no_ob = size(results,2);
% h = results(1).h1; 
% coef_u = results(1).coef_u1; 
% remaining = results(1).remaining1; 
% h3_1 = results(1).h2; 
% coef_u_31 = results(1).coef_u2; 
% remaining_31 = results(1).remaining2; 
% 
% 
% %%QP problem: 
% 
% 
% 
% global beta;   %initial value is 0; 
% 
% for i_ob = 1:no_ob
%     if (beta(i_ob) == 0 ) && (results(i_ob).h2 >= -0.01)
%         beta(i_ob) = 1;
%     elseif (beta(i_ob) == 1 ) && (results(i_ob).h2 <=  -0.1)
%         beta(i_ob) = 0;
%     end
% end
% 
% %the variable determine which CBF is active now 
% % slack = zeros(2,1);
% % 
% % if (h3_1>= -0.01) 
% %     slack(1) = 1;
% % else
% %     slack(1) = 0 ;
% % end
% % 
% % if (h >= 0) 
% %     slack(2) = 1;
% % else
% %     slack(2) = 0 ;
% % end 
% 
% 
% %the variable determine which CBF is active now 
% slack_mult = zeros(2,no_ob);
% %number of the possible conditions: 
% nu_combine =  1; 
% order = [];
% for aa = 1:no_ob
%     if(beta(aa)==1)
%     %if does not point to the obstacle:
%         if  (results(aa).h2>= -0.01)   %pointing constraint
%             slack_mult(1, no_ob) = 1;   %active
%         else
%             slack_mult(1, no_ob) = 0 ;
%         end
% 
%         if (  results(aa).h1 >= 0)   %distance constraint
%             slack_mult(2, no_ob) = 1;   %active
%         else
%             slack_mult(2, no_ob) = 0 ;
%         end 
%         if (  slack_mult(1, no_ob) == 0) && (  slack_mult(2, no_ob) == 0)
%             slack_mult(2, no_ob) =1; %at least one should be 1 
%         end
%     
%         nu_combine = nu_combine*sum(slack_mult(:,no_ob));   %number of the possible conditions
%     
%         row_order = size(order,1);
%         if (row_order == 0)
%             row_order = 1;
%         end
%         
%         if(slack_mult(1, no_ob) == 1) && (slack_mult(2, no_ob) ==1)
%             order = [order, ones(row_order, 1); order, 2*ones(row_order, 1)];  %record the place 
%         elseif(slack_mult(1, no_ob) == 1) && (slack_mult(2, no_ob) == 0)
%             order = [order, ones(row_order, 1)];  %record the place 
%         elseif(slack_mult(1, no_ob) == 0) && (slack_mult(2, no_ob) == 1)
%             order = [order, 2*ones(row_order, 1)];  %record the place 
%         end
%     else
%         row_order = size(order,1);
%         if (row_order == 0)
%             row_order = 1;
%         end
%         order = [order, zeros(row_order, 1)];
%     end
% end
% 
% 
% %if pointing to this obstacal, the distance constraint and angle constraint should be satisfied together 
% % index_poining = (beta==1);
% % A_n = [-results(index_poining).coef_u1; -results(index_poining).coef_u2];
% % b_n = [   results(index_poining).remaining1;  results(index_poining).remaining2 ];
%  
% 
% pointing_turn = slack_mult; %the turn flag determine which constraint will be added: 1-angle,  2-distance, 3-distance(no cbf is active condition)
% 
% %the minmal value and the corresponding solution, notice there may be no
% %solution:
% value_min = 100000000;
% max_delta_lb = 1; 
% x_min = [0;0];
% 
% pointing_turn = ones(2,no_ob);
% global A_n_and b_n_and A_n_or b_n_or;
% for i_combine = 1:nu_combine
%     A_n_and = [];
%     b_n_and = [];
%     A_n_or = [];
%     b_n_or = [];
%     
%     for aa = 1:no_ob
%         if (beta(aa) == 0 )
% % %             if pointing to the obstacle, both conditions should be satisfied 
%             A_n_and = [A_n_and; -results(aa).coef_u1; -results(aa).coef_u2];
%             b_n_and = [b_n_and;   results(aa).remaining1;  results(aa).remaining2 ];
%         else            
%             if(order(i_combine, aa) ==1)  %pointing constraint
%                  A_n_or = [A_n_or; -results(aa).coef_u2; ]; 
%                  b_n_or = [b_n_or;   results(aa).remaining2;  ];
%             elseif (order(i_combine, aa) == 2)    %distance constraint
%                  A_n_or = [A_n_or; -results(aa).coef_u1; ]; 
%                  b_n_or = [b_n_or;   results(aa).remaining1 ];         
%             end 
%         end
%  
%     end
%  
%     
%      %solve QP at the end, see if the angle constraints for multiple
%      %obstacles solvable 
%      H= diag([1;1]);
% %     f2 = zeros(2, 1);
%     f2 = -2* u_nom;  %the optimal goal is for the entire control
%     optoption_1 = optimset('Display', 'off', 'TolFun', 1e-20);
%      if (size(A_n_and,1)>0)
%      
% %          b_n = b_n_and - A_n_and*u_nom; %the optimal goal is for the entire control
% 
%         if(flag_bound ==0)
% %             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n, [], [], -alpha-u_nom, alpha-u_nom, [], optoption_1);
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n_and, [], [], -alpha, alpha, [], optoption_1);
%         else
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n_and, [], [], [], [], [], optoption_1);
%         end
%         
% %         delta_just = width_control(A_n_and, b_n_and);   %calucate the width of the feasible control  
%         
%         
%         if (EXITFLAG<0)  
%             %qp  has no solution 
%             A_n_and = [];
%             b_n_and = [];
%             for aa = 1:no_ob
%                 if (beta(aa) == 0 ) && (aa == 1)
%         % %             if angle CBF for multiple obstacles does not solvable,
%         % then only consider the angle constraints for the main obstacle 
%                     A_n_and = [A_n_and; -results(aa).coef_u1; -results(aa).coef_u2];
%                     b_n_and = [b_n_and;   results(aa).remaining1;  results(aa).remaining2 ];
%                 elseif (beta(aa) == 0 ) && (aa ~= 1) 
%                     A_n_and = [A_n_and; -results(aa).coef_u1; ];
%                     b_n_and = [b_n_and;   results(aa).remaining1; ];
%                 end
%             end
% 
%         end
%      end
%      
%     global A_n b_n;    
%     A_n = [A_n_and; A_n_or];
%     b_n = [b_n_and; b_n_or];
%     
%      %solve QP at the end
% %      H= diag([1;1]);
% %      f2 = zeros(2, 1);
% 
% %     delta_just2 = width_control(A_n, b_n);   %calucate the width of the feasible control 
%     
%     %see if solvable 
% %     if(delta_just2.max< 0 )
%         %if the QP is solvable, then solve it 
% 
% %          b_n = b_n - A_n*u_nom; %the optimal goal is for the entire control
%          f2 = -2* u_nom;  %the optimal goal is for the entire control
%  
%         if(flag_bound ==0)
% %             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha-u_nom, alpha-u_nom, [], optoption_1);
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
%         else
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], [], optoption_1);
%         end
%         if (EXITFLAG < 0)  
%             %qp  has no solution 
%             FVAL = 100000000; %no solution, set a very big value 
%         end
% 
%         if (FVAL < value_min)
%             value_min = FVAL; %update
%             x_min = x; 
%         end   
%  
% %     end
%     
% %     if(delta_just2.max < max_delta_lb) 
% %         max_delta_lb = delta_just2.max; 
% %         x_min = x; 
% %     end
% end
%  
% % % the output: 
% if (value_min~=100000000) 
% % if (max_delta_lb<0)
%     %select the minimal solution 
% %     out= x_min + u_nom; 
%      out= x_min; 
% else
%     %no solution exsits, brake  
%     out = [-alpha(1); 0];
% end
% 
%  
% 
% 

%% for dynamic obstacles: 
 
global results_2; 
results_2 = constraint_obstacles_dynamics([p_x; p_y; v; psi], time); 

no_ob = size(results_2,2);
 
global beta_2;   %initial value is 0; 



for i_ob = 1:no_ob
    %justify when jump: 
    Ds = 1.2; 
    theta_d_big = asin((Ds)/results_2(i_ob).norm_relpos) - asin( (Ds -0.2) /results_2(i_ob).norm_relpos);
%     theta_d_big =0.1;
    theta_d_small = theta_d_big/10; 
    if (beta_2(i_ob) == 0 ) && (results_2(i_ob).h_angle_fix > -theta_d_small)
        beta_2(i_ob) = 1;
    elseif (beta_2(i_ob) == 1 ) && (results_2(i_ob).h_angle_fix <=  -theta_d_big)
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
        Ds = 1.2; 
        theta_d_big = asin((Ds)/results_2(aa).norm_relpos) - asin( (Ds-0.2) /results_2(aa).norm_relpos);
%         theta_d_big = 0.1;
        theta_d_small = theta_d_big/2;
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


%if pointing to this obstacal, the distance constraint and angle constraint should be satisfied together 
% index_poining = (beta==1);
% A_n = [-results(index_poining).coef_u1; -results(index_poining).coef_u2];
% b_n = [   results(index_poining).remaining1;  results(index_poining).remaining2 ];
 

% pointing_turn = slack_mult; %the turn flag determine which constraint will be added: 1-angle,  2-distance, 3-distance(no cbf is active condition)

%the minmal value and the corresponding solution, notice there may be no
%solution:
value_min = 100000000;
% max_delta_lb = 1; 
x_min = [0;0];

% pointing_turn = ones(2,no_ob);
% global A_n_and b_n_and A_n_or b_n_or;
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
                 A_n_or = [A_n_or;  results_2(aa).A_n_angle_fix; ]; 
                 b_n_or = [b_n_or;   results_2(aa).B_n_angle_fix;  ];
            elseif (order(i_combine, aa) == 2)    %distance constraint
                 A_n_or = [A_n_or;  results_2(aa).A_n_dis; ]; 
                 b_n_or = [b_n_or;   results_2(aa).B_n_dis ];         
            end 
        end
 
    end
 
    
     %solve QP at the end, see if the angle constraints for multiple
     %obstacles solvable 
     H= diag([1;1]);
%     f2 = zeros(2, 1);
     f2 = -2* u_nom;  %the optimal goal is for the entire control
     optoption_1 = optimset('Display', 'off', 'TolFun', 1e-6); %if the resulution is too high, may make the MPC not solvable, notice  
     if (size(A_n_and,1)>0)
     
%          b_n = b_n_and - A_n_and*u_nom; %the optimal goal is for the entire control

        if(flag_bound ==0)
%             [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n_and, b_n, [], [], -alpha-u_nom, alpha-u_nom, [], optoption_1);
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

%          b_n = b_n - A_n*u_nom; %the optimal goal is for the entire control
        f2 = -2* u_nom;  %the optimal goal is for the entire control
 
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


