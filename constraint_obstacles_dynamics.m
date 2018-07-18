function out = constraint_obstacles_dynamics(u, t)
%calculate the coefficients from the multiple static obstacles 

%carefully check this file

%feedback states:
p_x = u(1);
p_y = u(2);
v = u(3); 
psi = u(4); 
 
%static obstacles: 
no_ob = 5; 

global pos_ob_array_pre;
pos_ob_array_pre = zeros(2,no_ob);
vel_ob_array_pre = zeros(2,no_ob);
acc_ob_array = zeros(2,no_ob);

%%note that if two of the obstacles have the same x-coordinates, then the
%%solver will let the vehicle go into the gap between the two obstacles,
%%and if the gap is two small or even negative, then the QP problem will
%%become non solvable. 
%in this case, we can modify the center of the obstacle a little, so that
%the solver will not let the vehicle go to the gap between the two
%obstacles, but go to another side which is not between the
%two obstacles. 
pos_ob_array_pre(:,1) = [60-5*t; 0.0];
pos_ob_array_pre(:,2) = [500; -0.0];
pos_ob_array_pre(:,3) = [800; -0.1];
pos_ob_array_pre(:,4) = [800;  1.5];
pos_ob_array_pre(:,5) = [1000;  0.1];
% pos_ob_array_pre(:,6) = [1200; -0.1];
% pos_ob_array_pre(:,7) = [1400;  0.1];
% pos_ob_array_pre(:,8) = [1400; -0.5];
% pos_ob_array_pre(:,9) = [1700; -0.1];
% pos_ob_array_pre(:,10) = [1900; -0.1];

vel_ob_array_pre(:,1) = [ -5; 0];
vel_ob_array_pre(:,2) = [0; 0];


%the size of the output depends on the number of the obstacles and the
%number of the constraints:
 
%select the active obstacles, which are in the front of the vehicle, and the
%distance is less than a shreshold. 
dis_shresh = 600;
pos_ob_array = [];
vel_ob_array =[];
for i = 1:no_ob 
    if (pos_ob_array_pre(1,i)>=(p_x-10)) && (abs(p_x-pos_ob_array_pre(1,i))<=dis_shresh)
        pos_ob_array = [pos_ob_array, pos_ob_array_pre(:,i)];
        vel_ob_array = [vel_ob_array, vel_ob_array_pre(:,i)];
    end
end
no_ob = size(pos_ob_array,2); 
if(no_ob == 0)
    pos_ob_array = pos_ob_array_pre(:,end);
    vel_ob_array = vel_ob_array_pre(:,end);  %very important, cause a lot of errors 
    no_ob=1;
end

if (no_ob >=2)
    %if the x-coordinates of the first two obstacles are the same, then
    %modified the second obstacles a little, see the comments previously 
    if(pos_ob_array(1,1) == pos_ob_array(1,2))
       if(abs(pos_ob_array(2,1)) >= abs(pos_ob_array(2,2)))
            pos_ob_array(1,2) = pos_ob_array(1,2) - 0.01;
       else
           pos_ob_array(1,2) = pos_ob_array(1,2) + 0.01;
       end
    end
end


for i_ob =1:no_ob
    %%relative to obstacles:
    pos_ob = pos_ob_array(:,i_ob);  
    vel_ob = vel_ob_array(:,i_ob);  
    acc_ob = acc_ob_array(:,i_ob);  

    p_x_dot = v*cos(psi);
    p_y_dot = v*sin(psi); 
    vel_vehicle = [p_x_dot; p_y_dot];     

    rel_pos = [p_x; p_y] - pos_ob;
    rel_vel = [p_x_dot; p_y_dot] - vel_ob; 
    
    Ds = 1; 
    a_m = 4;  %maximum acc 
    
    norm_v_vehicle = norm([p_x_dot; p_y_dot]);
    norm_v_ob = norm(vel_ob);
    norm_rel_pos = norm(rel_pos); 
    
%% July 10, add the consstraints that the velocity of the obstacle and the vehicle should be in the opposite side of the vector of relative position: 
    beta = 4; 
    rel_pos_norm =  [-rel_pos(2); rel_pos(1)];  %normal to rel_pos    
    rel_pos_norm_dot = [-(v*sin(psi)-vel_ob(2)); v*cos(psi)-vel_ob(1)];  %dot of rel_pos_norm
    
    h_move_angle = -rel_pos_norm'* [cos(psi); sin(psi)] * rel_pos_norm'*vel_ob;
    Lgh = [0, -rel_pos_norm'*[-sin(psi); cos(psi)]*rel_pos_norm'*vel_ob];
    Lfth = -rel_pos_norm_dot'*[cos(psi); sin(psi)]*rel_pos_norm'*vel_ob...
        -rel_pos_norm'* [cos(psi); sin(psi)] * rel_pos_norm_dot'*vel_ob...
        -rel_pos_norm'* [cos(psi); sin(psi)] * rel_pos_norm_dot'*acc_ob; 
    
    A_n_anglemoving = -Lgh;
    B_n_anglemoving  = Lfth+beta*h_move_angle; 
    
    
  
    
 %% consider velocity: 
     %very important for this problem, because there may be singular 
    %sometimes, this constant should be big enough, in order to let the solver
    %works 
    cc = 1; 
    if (abs(v)>cc)
        matrix_line = [cos(psi), -v*sin(psi); sin(psi),  v*cos(psi)]; 
    else
        matrix_line = [cos(psi), -cc*sin(psi); sin(psi),  cc*cos(psi)]; 
    end
    Ds = 1; 
    alpha = 4; 
    gamma = 5; 

    norm_deltap = (norm(rel_pos)); 
    if (norm_deltap < Ds) 
        norm_deltap = Ds; 
    end

    norm_rel_p_dot = 0.5/norm_deltap*2*rel_pos'*rel_vel; 

    h_dis = sqrt(2*alpha*(norm_deltap-Ds)) + rel_pos'/norm_deltap*rel_vel; 
    coef_u =  rel_pos'/norm_deltap* matrix_line;
    % coef_u =  rel_pos'/norm_deltap; 

    remaining   = 0.5*1/sqrt(2*alpha*norm_deltap-Ds) *2*alpha* norm_rel_p_dot +  (rel_vel'*norm_deltap - rel_pos'*norm_rel_p_dot)/norm_deltap^2*rel_vel...
        + gamma*h_dis; 
    
    A_n_dis = -coef_u; 
    B_n_dis = remaining; 
    
    
%% the relax constraints         
%     angle_rel = acos(( rel_pos'*[cos(psi); sin(psi)]) /norm(rel_pos));
%     angle_rel_ob = acos(( rel_pos'*vel_ob) /norm(rel_pos)/norm(norm_v_ob));
%      
%     AB_sqare = (norm_v_vehicle^2/4/a_m*cos(angle_rel) - norm_v_ob^2/4/a_m *cos(angle_rel_ob) + norm_rel_pos)^2 + (norm_v_vehicle^2/4/a_m*sin(angle_rel) - norm_v_ob^2/ 4/a_m*sin(angle_rel))^2;
%         
%     %%for moving obstacles: 
%     gamma = 20; 
%     h_moving =  AB_sqare - (Ds+ norm_v_vehicle^2/4/a_m + norm_v_ob^2/4/a_m)^2;
%     A_n = norm_v_vehicle*norm_v_ob/(8*a_m*a_m)*vel_ob + ...
%         vel_vehicle'*vel_ob*norm_v_ob/(8*a_m*a_m*norm_v_vehicle)*vel_vehicle - ...
%         rel_pos'*vel_vehicle/(2*a_m*vel_vehicle)*vel_vehicle - ...
%         norm_v_vehicle*rel_pos/(2*a_m) + ...
%         Ds*vel_vehicle/a_m + norm_v_ob^2*vel_vehicle/(4*a_m*a_m); 
%     A_n = A_n'; %should be 1-by-2 matrix 
%     B_n = rel_pos'*rel_vel + norm_v_vehicle/(2*a_m) * rel_vel'*vel_vehicle + 0.5*gamma*h_moving; 



%% angle constraints, the same to the fixed obstacle: 
    %1st constraint, the angle constraint 
    
    %there is a belt to prevent jump, so Ds should be carefully treated in
    %this case, see notebook for carefully: 
    
    %two possibilities: 
%     delta_theta = 0.1;     
%     Ds = norm(rel_pos) * sin(delta_theta + asin(Ds/norm(rel_pos))); 
    
    Ds = Ds + 0.2; 
    
    
    
    
    angle_ob =  atan(rel_pos(2)/rel_pos(1));  %the angle of the vector from vehicle to obstacle 
    angle_rel = psi - angle_ob; 
    h3_1 =  norm(rel_pos)* abs(sin(angle_rel)) - Ds; 
    %another way to define the relative angle: 
    angle_rel = acos((-rel_pos'*[cos(psi); sin(psi)]) /norm(rel_pos));

%     if (angle_rel ==0)
%         angle_rel = 1e-3;
%     end

    h3_1 =  norm(rel_pos)* abs(sin(angle_rel)) - Ds;

    pos_ob_x = pos_ob(1);
    pos_ob_y = pos_ob(2); 

    %very important, if this is zero, may make the qp infeasible due to the
    %coefficient matrix be zeros, but the right matrix is negative 
    %sometimes, it may be NaN of inf, so this constraint actually do not work
    %noticed on July, 18th, 2018
    if(pos_ob_y==0)    
        pos_ob_y = 1e-4;
    end
    if (psi ==0)
        psi = 1e-5;
    end

    %compute from the symbols: 
    % L_f_h3_1 = v*sin(psi)*((abs(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) - (sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))/((p_x - pos_ob_x)*((p_y - pos_ob_y)^2/(p_x - pos_ob_x)^2 + 1))) + v*cos(psi)*((abs(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(p_y - pos_ob_y))/((p_x - pos_ob_x)^2*((p_y - pos_ob_y)^2/(p_x - pos_ob_x)^2 + 1))); 
    % L_g_h3_1 =[ 0, sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)]; 
    %compute from the symbols: 
    L_f_h3_1 = v*sin(psi)*((abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*((2*sin(psi)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2*(2*p_y - 2*pos_ob_y))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))/(2*abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2))) + v*cos(psi)*((abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*((2*cos(psi)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2*(2*p_x - 2*pos_ob_x))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^2))/(2*abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)));
    L_g_h3_1 = [ 0, (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x)))/(abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))]; 

    ratio = Ds/norm(rel_pos);
    
    %notice if do not deal carefully, there maybe virtual number appears. 
    if ratio>=1
        ratio = 0.9999;
    end    
    
    h3_1 = angle_rel - asin(ratio); 
     

    L_f_h3_1 = v*cos(psi)*((cos(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2) + (Ds*(2*p_x - 2*pos_ob_x))/(2*(1 - Ds^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2))^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))) + v*sin(psi)*((sin(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2) + (Ds*(2*p_y - 2*pos_ob_y))/(2*(1 - Ds^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2))^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))); 
    L_g_h3_1 = [ 0, (cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x))/(((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2))]; 

    ratio_dot = -Ds/ (norm(rel_pos))^2 *norm_rel_p_dot; 
    asin_dot = 1/(1 - ratio^2)^(1/2); 

    L_f_h3_1 = (v*cos(psi)*(cos(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2) + (v*sin(psi)*(sin(psi)/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))))/(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2); 
    L_g_h3_1 = [ 0, (cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x))/(((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(- (cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) + 1)^(1/2))]; 
    L_f_h3_1 = L_f_h3_1 -asin_dot*ratio_dot; 
    
%     msg = 'Error occurred.';
%     error(msg);
    %modified on July, 17, 2018, not computed from symbolic calculation,
    %but handly:
        %very important, if this is zero, may make the qp infeasible due to the
    %coefficient matrix be zeros, but the right matrix is negative 
%     if(rel_pos(2)==0)    
%         rel_pos(2) = 1e-4;
%     end
%     
%     L_g_h3_1 = [0, -rel_pos'*[-sin(psi); cos(psi)]/norm_deltap];
%     L_f_h3_1 = ((-rel_vel'*[cos(psi); sin(psi)])*norm_deltap+rel_pos'*[cos(psi);sin(psi)]*norm_rel_p_dot)/norm_deltap/norm_deltap -asin_dot*ratio_dot; 
    
    gamma_31 = 3; %constant 
    
%     if( abs(L_g_h3_1(2))<1e-5) && (L_f_h3_1 + gamma_31*h3_1<0)
%         L_g_h3_1(2) = -1e-5;
%     end

    % if(angle_rel<=  pi/2)
    coef_u_31 = L_g_h3_1;     
    remaining_31 = L_f_h3_1 + gamma_31*h3_1; 
    
    

        
%% output, notice the define of the output variables: 
    out(i_ob).norm_relpos = norm(rel_pos);
    out(i_ob).h_angle_moving= h_move_angle; 
    out(i_ob).A_n_angle_moving  = A_n_anglemoving; 
    out(i_ob).B_n_angle_moving = B_n_anglemoving;    
    out(i_ob).h_angle_fix =  h3_1; 
    out(i_ob).A_n_angle_fix=  -coef_u_31; 
    out(i_ob).B_n_angle_fix =   remaining_31;
    out(i_ob).h_dis = h_dis; 
    out(i_ob).A_n_dis = A_n_dis; 
    out(i_ob).B_n_dis = B_n_dis; 
end

