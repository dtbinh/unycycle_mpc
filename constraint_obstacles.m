function out = constraint_obstacles(u)
%calculate the coefficients from the multiple static obstacles 


%feedback states:
p_x = u(1);
p_y = u(2);
v = u(3); 
psi = u(4); 
 
%static obstacles: 
no_ob = 5; 

global pos_ob_array_pre;
pos_ob_array_pre = zeros(2,no_ob);
vel_ob_array = zeros(2,no_ob);
acc_ob_array = zeros(2,no_ob);

%%note that if two of the obstacles have the same x-coordinates, then the
%%solver will let the vehicle go into the gap between the two obstacles,
%%and if the gap is two small or even negative, then the QP problem will
%%become non solvable. 
%in this case, we can modify the center of the obstacle a little, so that
%the solver will not let the vehicle go to the gap between the two
%obstacles, but go to another side which is not between the
%two obstacles. 
pos_ob_array_pre(:,1) = [50; -1.1];
pos_ob_array_pre(:,2) = [50; 1.1];
pos_ob_array_pre(:,3) = [51.8; -2.8];
pos_ob_array_pre(:,4) = [51.8;  0.0];
pos_ob_array_pre(:,5) = [51.8;  2.8];
% pos_ob_array_pre(:,6) = [1200; -0.1];
% pos_ob_array_pre(:,7) = [1400;  0.1];
% pos_ob_array_pre(:,8) = [1400; -0.5];
% pos_ob_array_pre(:,9) = [1700; -0.1];
% pos_ob_array_pre(:,10) = [1900; -0.1];


%the size of the output depends on the number of the obstacles and the
%number of the constraints:
 



%select the active obstacles, which are in the front of the vehicle, and the
%distance is less than a shreshold. 
dis_shresh = 200;
pos_ob_array = [];
for i = 1:no_ob 
    if (pos_ob_array_pre(1,i)>=p_x) && (abs(p_x-pos_ob_array_pre(1,i))<=dis_shresh)
        pos_ob_array = [pos_ob_array, pos_ob_array_pre(:,i)];
    end
end
no_ob = size(pos_ob_array,2); 
if(no_ob == 0)
    pos_ob_array = pos_ob_array_pre(:,end);
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

    rel_pos = [p_x; p_y] - pos_ob;
    rel_vel = [p_x_dot; p_y_dot] - vel_ob; 

    Ds = 1; 

    %%only distance constraints, not good: 
    K = [1, 2*1.414*1];
    %  K = [1.2,3];

    h_disonly = (norm(rel_pos))^2 - Ds^2; 
    h_dot = 2*rel_pos'*rel_vel;

     %coefficient of the input 
    coef_u_ponly = [2*rel_pos(1)*cos(psi)+2*rel_pos(2)*sin(psi), -2*rel_pos(1)*v*sin(psi)+2*rel_pos(2)*v*cos(psi)]; 
    %remaining: 
    remaining_ponly    = 2*(norm(rel_vel))^2 - 2* rel_pos'*acc_ob + K*[h_disonly; h_dot];

    %very important for this problem, because there may be singular 
    %sometimes, this constant should be big enough, in order to let the solver
    %works 
    cc = 1; 
    if (abs(v)>cc)
        matrix_line = [cos(psi), -v*sin(psi); sin(psi),  v*cos(psi)]; 
    else
        matrix_line = [cos(psi), -cc*sin(psi); sin(psi),  cc*cos(psi)]; 
    end


    %%consider velocity: 
    Ds = 1; 
    alpha = 4; 

    gamma = 5; 

    norm_deltap = (norm(rel_pos)); 
    if (norm_deltap < Ds) 
        norm_deltap = Ds; 
    end

    norm_rel_p_dot = 0.5/norm_deltap*2*rel_pos'*rel_vel; 

    h = sqrt(2*alpha*(norm_deltap-Ds)) + rel_pos'/norm_deltap*rel_vel; 
    coef_u =  rel_pos'/norm_deltap* matrix_line;
    % coef_u =  rel_pos'/norm_deltap; 

    remaining   = 0.5*1/sqrt(2*alpha*norm_deltap-Ds) *2*alpha* norm_rel_p_dot +  (rel_vel'*norm_deltap - rel_pos'*norm_rel_p_dot)/norm_deltap^2*rel_vel...
        + gamma*h; 


    %%velocity constraints: 
    h2= v; 
    coef_u2 = [1, 0];
    remaining2   = 2*h2; 


    %%road side constraints 
    Ds2 = 2; 
    K2 = [1, 2*1.414*1]; 
    K2 = [1.2, 3.5];

    h2 = -(p_y)^2 + Ds2^2; 
    h2_dot = -2*p_y'*v*sin(psi);

    %coefficient of the input 
    coef_u2 = [0,  -2*p_y]; 
    %remaining: 
    remaining2   =   - 2* (v*sin(psi))^2  + K2*[h2; h2_dot];
    % coef_u2 = zeros(1, 2);
    % remaining2 = -10;



    %%CBF 3, for fixed obstacle: 
    %1st constraint, the angle constraint 
    Ds = 1.1;
    angle_ob =  atan(rel_pos(2)/rel_pos(1));  %the angle of the vector from vehicle to obstacle 
    angle_rel = psi - angle_ob; 
    h3_1 =  norm(rel_pos)* abs(sin(angle_rel)) - Ds; 
    %another way to define the relative angle: 
    angle_rel = acos((-rel_pos'*[cos(psi); sin(psi)]) /norm(rel_pos));

    % if (angle_rel ==0)
    %     angle_rel = 1e-5;
    % end

    h3_1 =  norm(rel_pos)* abs(sin(angle_rel)) - Ds;

    pos_ob_x = pos_ob(1);
    pos_ob_y = pos_ob(2); 
    
    if(pos_ob_y==0)    
        pos_ob_y = 1e-2;
    end


    %compute from the symbols: 
    % L_f_h3_1 = v*sin(psi)*((abs(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) - (sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))/((p_x - pos_ob_x)*((p_y - pos_ob_y)^2/(p_x - pos_ob_x)^2 + 1))) + v*cos(psi)*((abs(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(p_y - pos_ob_y))/((p_x - pos_ob_x)^2*((p_y - pos_ob_y)^2/(p_x - pos_ob_x)^2 + 1))); 
    % L_g_h3_1 =[ 0, sign(sin(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x))))*cos(psi - atan((p_y - pos_ob_y)/(p_x - pos_ob_x)))*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)]; 
    %compute from the symbols: 
    L_f_h3_1 = v*sin(psi)*((abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*((2*sin(psi)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2*(2*p_y - 2*pos_ob_y))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))/(2*abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2))) + v*cos(psi)*((abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)) + (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*((2*cos(psi)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - ((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2*(2*p_x - 2*pos_ob_x))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^2))/(2*abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)));
    L_g_h3_1 = [ 0, (sign((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)*(cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))*(cos(psi)*(p_y - pos_ob_y) - sin(psi)*(p_x - pos_ob_x)))/(abs((cos(psi)*(p_x - pos_ob_x) + sin(psi)*(p_y - pos_ob_y))^2/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2) - 1)^(1/2)*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2))]; 

    ratio = Ds/norm(rel_pos);
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

    % if(angle_rel<=  pi/2)
        coef_u_31 = L_g_h3_1; 
        gamma_31 = 3; %constant 
        remaining_31 = L_f_h3_1 + gamma_31*h3_1; 
    % else %if the relative angle is more than pi/2, do not need to constraint 
    %     h3_1 = pi; 
    %     coef_u_31 = [0, 0]; 
    %     remaining_31 = 100;
    % end

    %2nd constraint, the distance constraint 
    a_m = 4;  %maximum acc 
    l = v^2/(2*a_m);
    h3_2 =  norm(rel_pos)-Ds - l; 

    %compute from the symbols: 
    L_f_h3_2 = (v*cos(psi)*(p_x - pos_ob_x))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (v*sin(psi)*(p_y - pos_ob_y))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2); 
    L_g_h3_2 = [ -v/a_m, 0]; 

    coef_u_32 = L_g_h3_2; 
    gamma_32= 3; %constant 
    remaining_32 = L_f_h3_2 + gamma_32*h3_2; 
    %from manual calculation: 
    % remaining_32 = norm_rel_p_dot + gamma_32*h3_2; 


    %compute from the symbols: 
    h3_3 =  sqrt(2*a_m*(norm_deltap-Ds)) + rel_pos'/norm_deltap*[v*cos(psi); v*sin(psi)]; 
    L_f_h3_3 = v*cos(psi)*((v*cos(psi))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (2^(1/2)*a_m*(2*p_x - 2*pos_ob_x))/(4*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(-a_m*(Ds - ((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)))^(1/2)) - (v*sin(psi)*(conj(p_y) - conj(pos_ob_y))*(2*p_x - 2*pos_ob_x))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)) + (v*cos(psi)*(p_x*abs(pos_ob_x)^2 - pos_ob_x*abs(p_x)^2)*(2*p_x - 2*pos_ob_x))/(2*p_x*pos_ob_x*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))) + v*sin(psi)*((v*sin(psi))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (2^(1/2)*a_m*(2*p_y - 2*pos_ob_y))/(4*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)*(-a_m*(Ds - ((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)))^(1/2)) - (v*cos(psi)*(conj(p_x) - conj(pos_ob_x))*(2*p_y - 2*pos_ob_y))/(2*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2)) + (v*sin(psi)*(p_y*abs(pos_ob_y)^2 - pos_ob_y*abs(p_y)^2)*(2*p_y - 2*pos_ob_y))/(2*p_y*pos_ob_y*((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(3/2))); 
    L_g_h3_3 = [ (cos(psi)*(conj(p_x) - conj(pos_ob_x)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) + (sin(psi)*(conj(p_y) - conj(pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2), (v*cos(psi)*(conj(p_y) - conj(pos_ob_y)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2) - (v*sin(psi)*(conj(p_x) - conj(pos_ob_x)))/((p_x - pos_ob_x)^2 + (p_y - pos_ob_y)^2)^(1/2)]; 

    coef_u_33 = L_g_h3_3; 
    gamma_33= 2; %constant 
    remaining_33 = L_f_h3_3 + gamma_33*h3_3; 
    
    %output, notice the define of the output variables: 
    out(i_ob).h1= h; 
    out(i_ob).coef_u1 = coef_u; 
    out(i_ob).remaining1 = remaining; 
    out(i_ob).h2 = h3_1; 
    out(i_ob).coef_u2 = coef_u_31;
    out(i_ob).remaining2 = remaining_31; 

end

