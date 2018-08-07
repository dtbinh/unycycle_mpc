function out = constraint_obstacles_dynamics_complex(u, t)
%calculate the coefficients from the multiple static obstacles 

%carefully check this file

%feedback states:
xp_dot = u(1);  %longitudinal speed
yp_dot = u(2);  %lateral speed
psi_dot = u(3); 
epsi = u(4);
ey= u(5);  %lateral position
s = u(6);  %logitudinal position 
 
%static obstacles: 
no_ob = 3; 

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
pos_ob_array_pre(:,1) = [40+0*t; 0.0];
pos_ob_array_pre(:,2) = [42; 0.8];
pos_ob_array_pre(:,3) = [150; -0.1];
% pos_ob_array_pre(:,4) = [800;  1.5];
% pos_ob_array_pre(:,5) = [1000;  0.1];
% pos_ob_array_pre(:,6) = [1200; -0.1];
% pos_ob_array_pre(:,7) = [1400;  0.1];
% pos_ob_array_pre(:,8) = [1400; -0.5];
% pos_ob_array_pre(:,9) = [1700; -0.1];
% pos_ob_array_pre(:,10) = [1900; -0.1];

vel_ob_array_pre(:,1) = [ 0; 0];
vel_ob_array_pre(:,2) = [0; 0];
vel_ob_array_pre(:,3) = [0; 0];


%the size of the output depends on the number of the obstacles and the
%number of the constraints:
 
%select the active obstacles, which are in the front of the vehicle, and the
%distance is less than a shreshold. 
dis_shresh = 600;
pos_ob_array = [];
vel_ob_array =[];
for i = 1:no_ob 
    if (pos_ob_array_pre(1,i)>=(s-10)) && (abs(s-pos_ob_array_pre(1,i))<=dis_shresh)
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
%% states of the vehicle and each obstacle:
    pos_ob = pos_ob_array(:,i_ob);  
    vel_ob = vel_ob_array(:,i_ob);  
    acc_ob = acc_ob_array(:,i_ob);  
    
    pos_ob_x = pos_ob(1);
    pos_ob_y = pos_ob(2);
    vel_ob_x = vel_ob(1);
    vel_ob_y = vel_ob(2);
    acc_ob_x = acc_ob(1);
    acc_ob_y = acc_ob(2);

    v_vehicle = [xp_dot*cos(epsi)-yp_dot*cos(epsi); yp_dot*cos(epsi) + xp_dot*sin(epsi)];
    rel_pos = [s; ey] - pos_ob;
    rel_vel = v_vehicle - [vel_ob_x; vel_ob_y]; %assume epsi is small, 
    
%     Ds = 1.2;  %the radius of obstacle 
    a_m = 4;  %maximum acc 
    
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
        
 %% August 3th, angle constraint for obstacles, 
        %constraint 1, angle constraint, test:  
    %notice there may be virtual number, nan, of inf, you should aoid these
    %conditions: 
    Ds = 1.2;
    cos_rel_ang = (-rel_pos.'*rel_vel) /norm(rel_pos)/norm(rel_vel); 
    
    if (cos_rel_ang>=-0.99) 
        if (cos_rel_ang>=1)
            cos_rel_ang = 1;
        end
        rel_ang = acos(cos_rel_ang); 
        % h_ang = rel_ang-asin(Ds/norm(rel_pos));

        ratio = Ds/norm(rel_pos);
        %notice if do not deal carefully, there maybe virtual number appears. 
        if ratio>=1
            ratio = 0.9999;
        end    
        h_ang = rel_ang - asin(ratio); 

        %very important, if this is zero, may make the qp infeasible due to the
        %coefficient matrix be zeros, but the right matrix is negative 
        %sometimes, it may be NaN of inf, so this constraint actually do not work
        %noticed on July, 18th, 2018
        if(abs(pos_ob_y)<1e-4)    
            pos_ob_y = sign(pos_ob_y)*1e-4;
        end
        if (abs(epsi) <= 1e-5)
            epsi = sign(epsi)*1e-5;
        end

        %very important, due to calculation errors, there maybe sometimes the
        %results are virtual, should be treated carefully. 

        %notice, the longitudinal velocity is not be considered as state here 
        % L_f_h_ang = (yp_dot*cos(epsi) + xp_dot*sin(epsi))*((Ds*(ey - pos_ob_y))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - ((pos_ob_x - s)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) - cos(epsi)*(xp_dot - yp_dot)*((Ds*(pos_ob_x - s))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) + ((ey - pos_ob_y)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) + ((psi_dot - psi_dot_com)*(xp_dot*yp_dot - xp_dot^2 + vel_ob_x*xp_dot*cos(epsi) + vel_ob_y*xp_dot*sin(epsi) - vel_ob_x*yp_dot*sin(epsi) - vel_ob_y*yp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
        % L_g_h_ang = (2*cf*cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 

        L_f_h_ang_part1 = ((psi_dot - psi_dot_com)*(xp_dot*yp_dot - xp_dot^2 + vel_ob_x*xp_dot*cos(epsi) + vel_ob_y*xp_dot*sin(epsi) - vel_ob_x*yp_dot*sin(epsi) - vel_ob_y*yp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - ((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
        L_t_h_ang_part1 = -((ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi))*(pos_ob_y*vel_ob_x^3 - pos_ob_x*vel_ob_y^3 - ey*vel_ob_x^3 + s*vel_ob_y^3 - acc_ob_x*pos_ob_x^2*vel_ob_y + acc_ob_y*pos_ob_x^2*vel_ob_x - acc_ob_x*pos_ob_y^2*vel_ob_y + acc_ob_y*pos_ob_y^2*vel_ob_x - acc_ob_x*s^2*vel_ob_y + acc_ob_y*s^2*vel_ob_x - ey*vel_ob_x*vel_ob_y^2 - ey*vel_ob_x*xp_dot^2 - pos_ob_x*vel_ob_x^2*vel_ob_y + pos_ob_y*vel_ob_x*vel_ob_y^2 - pos_ob_x*vel_ob_y*xp_dot^2 + pos_ob_y*vel_ob_x*xp_dot^2 + s*vel_ob_x^2*vel_ob_y + s*vel_ob_y*xp_dot^2 - acc_ob_x*ey^2*vel_ob_y + acc_ob_y*ey^2*vel_ob_x + 2*acc_ob_x*ey*pos_ob_y*vel_ob_y - 2*acc_ob_y*ey*pos_ob_y*vel_ob_x + 2*acc_ob_x*pos_ob_x*s*vel_ob_y - 2*acc_ob_y*pos_ob_x*s*vel_ob_x - 2*ey*vel_ob_x*yp_dot^2*cos(epsi)^2 - 2*pos_ob_x*vel_ob_y*yp_dot^2*cos(epsi)^2 + 2*pos_ob_y*vel_ob_x*yp_dot^2*cos(epsi)^2 + 2*s*vel_ob_y*yp_dot^2*cos(epsi)^2 - acc_ob_y*ey^2*xp_dot*cos(epsi) + acc_ob_x*ey^2*yp_dot*cos(epsi) + acc_ob_y*ey^2*yp_dot*cos(epsi) - acc_ob_y*pos_ob_x^2*xp_dot*cos(epsi) - acc_ob_y*pos_ob_y^2*xp_dot*cos(epsi) + acc_ob_x*pos_ob_x^2*yp_dot*cos(epsi) + acc_ob_x*pos_ob_y^2*yp_dot*cos(epsi) + acc_ob_y*pos_ob_x^2*yp_dot*cos(epsi) + acc_ob_y*pos_ob_y^2*yp_dot*cos(epsi) - acc_ob_y*s^2*xp_dot*cos(epsi) + acc_ob_x*s^2*yp_dot*cos(epsi) + acc_ob_y*s^2*yp_dot*cos(epsi) + acc_ob_x*ey^2*xp_dot*sin(epsi) + 2*ey*vel_ob_x^2*xp_dot*cos(epsi) - 2*ey*vel_ob_x^2*yp_dot*cos(epsi) + acc_ob_x*pos_ob_x^2*xp_dot*sin(epsi) + acc_ob_x*pos_ob_y^2*xp_dot*sin(epsi) + acc_ob_x*s^2*xp_dot*sin(epsi) - 2*pos_ob_y*vel_ob_x^2*xp_dot*cos(epsi) + 2*pos_ob_x*vel_ob_y^2*yp_dot*cos(epsi) + 2*pos_ob_y*vel_ob_x^2*yp_dot*cos(epsi) - 2*s*vel_ob_y^2*yp_dot*cos(epsi) + 2*pos_ob_x*vel_ob_y^2*xp_dot*sin(epsi) - 2*s*vel_ob_y^2*xp_dot*sin(epsi) - 2*s*vel_ob_x*vel_ob_y*xp_dot*cos(epsi) + 2*s*vel_ob_x*vel_ob_y*yp_dot*cos(epsi) + 2*ey*vel_ob_x*vel_ob_y*xp_dot*sin(epsi) - 2*pos_ob_y*vel_ob_x*vel_ob_y*xp_dot*sin(epsi) + 2*ey*vel_ob_x*xp_dot*yp_dot*cos(epsi)^2 + 2*pos_ob_x*vel_ob_y*xp_dot*yp_dot*cos(epsi)^2 - 2*pos_ob_y*vel_ob_x*xp_dot*yp_dot*cos(epsi)^2 - 2*s*vel_ob_y*xp_dot*yp_dot*cos(epsi)^2 - ey*vel_ob_x*xp_dot*yp_dot*sin(2*epsi) - pos_ob_x*vel_ob_y*xp_dot*yp_dot*sin(2*epsi) + pos_ob_y*vel_ob_x*xp_dot*yp_dot*sin(2*epsi) + s*vel_ob_y*xp_dot*yp_dot*sin(2*epsi) + 2*acc_ob_y*ey*pos_ob_y*xp_dot*cos(epsi) - 2*acc_ob_x*ey*pos_ob_y*yp_dot*cos(epsi) - 2*acc_ob_y*ey*pos_ob_y*yp_dot*cos(epsi) + 2*acc_ob_y*pos_ob_x*s*xp_dot*cos(epsi) - 2*acc_ob_x*pos_ob_x*s*yp_dot*cos(epsi) - 2*acc_ob_y*pos_ob_x*s*yp_dot*cos(epsi) - 2*acc_ob_x*ey*pos_ob_y*xp_dot*sin(epsi) + 2*ey*vel_ob_x*vel_ob_y*yp_dot*cos(epsi) - 2*acc_ob_x*pos_ob_x*s*xp_dot*sin(epsi) + 2*pos_ob_x*vel_ob_x*vel_ob_y*xp_dot*cos(epsi) - 2*pos_ob_x*vel_ob_x*vel_ob_y*yp_dot*cos(epsi) - 2*pos_ob_y*vel_ob_x*vel_ob_y*yp_dot*cos(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2));
        L_g_h_ang_part1 = (2*cf*cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 

        L_f_h_ang_part2 = ((ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - ((xp_dot*cos(epsi) - yp_dot*cos(epsi))*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
        L_g_h_ang_part2 = 0;
        L_t_h_ang_part2 = -(ey*vel_ob_y - pos_ob_x*vel_ob_x - pos_ob_y*vel_ob_y + s*vel_ob_x)/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);

        ratio_pf = -Ds/ (norm(rel_pos))^2 *L_f_h_ang_part2; 
        ratio_pg = -Ds/ (norm(rel_pos))^2 *L_g_h_ang_part2; 
        ratio_pt = -Ds/ (norm(rel_pos))^2 *L_t_h_ang_part2; 
        asin_dot = 1/(1 - ratio^2)^(1/2); 
        L_f_h_ang=  L_f_h_ang_part1   -asin_dot*ratio_pf;
        L_g_h_ang=  L_g_h_ang_part1   -asin_dot*ratio_pg;
        L_t_h_ang=  L_t_h_ang_part1   -asin_dot*ratio_pt;

        A_n_angle_fix = [-L_g_h_ang, 0];
        b_n_angle_fix = L_f_h_ang + L_t_h_ang +3*h_ang;
    elseif (cos_rel_ang<=-0.99)
        h_ang = 3; 
        A_n_angle_fix = [0, 0];
        b_n_angle_fix = 1; 
    end
 
%%  August 3rd, constraint 2, velocity constraint, test:  
    %notice there may be virtual number, nan, of inf, you should aoid these
    %conditions: 
    Ds = 1;    
    dis_maxacc_sqr = 2*a_m*(norm(rel_pos)-Ds); 
    if (dis_maxacc_sqr<=0)
        dis_maxacc_sqr = 1e-4;
    end
    h_vel  = sqrt(dis_maxacc_sqr) + rel_pos'/norm(rel_pos)*rel_vel; 

    %very important, if this is zero, may make the qp infeasible due to the
    %coefficient matrix be zeros, but the right matrix is negative 
    %sometimes, it may be NaN of inf, so this constraint actually do not work
    %noticed on July, 18th, 2018
    if(abs(pos_ob_y)<1e-4)    
        pos_ob_y = sign(pos_ob_y)*1e-4;
    end
    if (abs(epsi) <= 1e-5)
        epsi = sign(epsi)*1e-5;
    end

    % L_f_h_ang = (yp_dot*cos(epsi) + xp_dot*sin(epsi))*((Ds*(ey - pos_ob_y))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - ((pos_ob_x - s)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) - cos(epsi)*(xp_dot - yp_dot)*((Ds*(pos_ob_x - s))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) + ((ey - pos_ob_y)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) + ((psi_dot - psi_dot_com)*(xp_dot*yp_dot - xp_dot^2 + vel_ob_x*xp_dot*cos(epsi) + vel_ob_y*xp_dot*sin(epsi) - vel_ob_x*yp_dot*sin(epsi) - vel_ob_y*yp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
    % L_g_h_ang = (2*cf*cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 

    L_f_h_norm = ((ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - ((xp_dot*cos(epsi) - yp_dot*cos(epsi))*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
    L_g_h_norm = [ 0, 0];
    L_t_h_norm = -(ey*vel_ob_y - pos_ob_x*vel_ob_x - pos_ob_y*vel_ob_y + s*vel_ob_x)/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);

    L_f_h_vel = (((ey - pos_ob_y)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((pos_ob_x - s)*(xp_dot*sin(epsi) - yp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2))*(psi_dot - psi_dot_com) + (xp_dot*cos(epsi) - yp_dot*cos(epsi))*(((2*pos_ob_x - 2*s)*(ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((2*pos_ob_x - 2*s)*(pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - (yp_dot*cos(epsi) + xp_dot*sin(epsi))*(((ey - pos_ob_y)*(2*ey - 2*pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((pos_ob_x - s)*(2*ey - 2*pos_ob_y)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - psi_dot*yp_dot*((cos(epsi)*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - (sin(epsi)*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)) - (cos(epsi)*(psi_dot*xp_dot + (psi_dot*(2*a*cf - 2*b*cr))/(m*xp_dot) + (yp_dot*(2*cf + 2*cr))/(m*xp_dot))*(ey + pos_ob_x - pos_ob_y - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
    L_g_h_vel = [ (2*cf*cos(epsi)*(ey + pos_ob_x - pos_ob_y - s))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)), (sin(epsi)*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - (cos(epsi)*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)];
    L_t_h_vel = vel_ob_y*(((ey - pos_ob_y)*(2*ey - 2*pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((pos_ob_x - s)*(2*ey - 2*pos_ob_y)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - vel_ob_x*(((2*pos_ob_x - 2*s)*(ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((2*pos_ob_x - 2*s)*(pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - (acc_ob_y*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + (acc_ob_x*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);

    temp_h_vel = 1/2*1/sqrt(dis_maxacc_sqr)*2*a_m; 

    L_f_h_vel= temp_h_vel* L_f_h_norm+L_f_h_vel;
    L_g_h_vel= temp_h_vel* L_g_h_norm+L_g_h_vel;
    L_t_h_vel= temp_h_vel* L_t_h_norm+L_t_h_vel;

    A_n_vel = -L_g_h_vel;
    b_n_vel = L_f_h_vel + L_t_h_vel +3*h_vel;

%% August, 3rd, constraint 3, velocity direction constraint for moving obstacles, test:  

    %notice there may be virtual number, nan, of inf, you should aoid these
    %conditions: 
   
    if (rel_pos(1)> 1)  && (v_vehicle(1)*vel_ob(1)<0) 
%         (rel_pos'* v_vehicle* rel_pos'*vel_ob < 0) 
        h_move_angle =  -2; 
        A_n_anglemoving = [0, 0];
        B_n_anglemoving = 1; 
    elseif (v_vehicle(1)*vel_ob(1)<0)
        rel_pos_vert =  [-rel_pos(2); rel_pos(1)];  %normal to rel_pos
        % h_move_angle = -rel_pos_vert'* v_vehicle* rel_pos_vert'*vel_ob/norm(rel_pos_vert)/norm(rel_pos_vert) ;
        h_move_angle = -rel_pos_vert'* v_vehicle* rel_pos_vert'*vel_ob  ;
        %very important, if this is zero, may make the qp infeasible due to the
        %coefficient matrix be zeros, but the right matrix is negative 
        %sometimes, it may be NaN of inf, so this constraint actually do not work
        %noticed on July, 18th, 2018
        % if(abs(pos_ob_y)<1e-4)    
        %     pos_ob_y = sign(pos_ob_y)*1e-4;
        % end
        % if (abs(epsi) <= 1e-5)
        %     epsi = sign(epsi)*1e-5;
        % end

        L_f_h_move_angle = cos(epsi)*(xp_dot - yp_dot)*(vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi))) - (yp_dot*cos(epsi) + xp_dot*sin(epsi))*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot)) - (vel_ob_y*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s) + vel_ob_x*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y))*(psi_dot - psi_dot_com) - psi_dot*yp_dot*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey*cos(epsi) - pos_ob_y*cos(epsi) + pos_ob_x*sin(epsi) - s*sin(epsi)) - (cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s)*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot))/(m*xp_dot);
        L_g_h_move_angle = [ (2*cf*cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s))/m, - vel_ob_x*(ey - pos_ob_y)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s)) - vel_ob_y*(pos_ob_x - s)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s))];
        L_t_h_move_angle = vel_ob_y*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot)) - vel_ob_x*(vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi))) - acc_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) - acc_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s);

        % L_f_h_move_angle = cos(epsi)*(xp_dot - yp_dot)*((vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((2*pos_ob_x - 2*s)*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2) - ((vel_ob_y*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s) + vel_ob_x*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y))*(psi_dot - psi_dot_com))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s))*(2*ey - 2*pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) - (psi_dot*yp_dot*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey*cos(epsi) - pos_ob_y*cos(epsi) + pos_ob_x*sin(epsi) - s*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - (cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s)*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2));
        % L_g_h_move_angle = [ (2*cf*cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)), -(vel_ob_x*(ey - pos_ob_y)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s)) + vel_ob_y*(pos_ob_x - s)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)];
        % L_t_h_move_angle = vel_ob_y*((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s))*(2*ey - 2*pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2) - vel_ob_x*((vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((2*pos_ob_x - 2*s)*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2) - (acc_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - (acc_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2);

        A_n_anglemoving = -L_g_h_move_angle;
        B_n_anglemoving = L_f_h_move_angle + L_t_h_move_angle + 3*h_move_angle;
        
        
        %test only: 
        h_move_angle = -2; 
        A_n_anglemoving = [0, 0];
        B_n_anglemoving = 1; 
    else 
        h_move_angle = -2; 
        A_n_anglemoving = [0, 0];
        B_n_anglemoving = 1; 
    end 
 
        
%% output, notice the define of the output variables: 
    out(i_ob).norm_relpos = norm(rel_pos);
    out(i_ob).h_angle_moving= h_move_angle; 
    out(i_ob).A_n_angle_moving  = A_n_anglemoving; 
    out(i_ob).B_n_angle_moving = B_n_anglemoving;    
    out(i_ob).h_angle_fix =  h_ang; 
    out(i_ob).A_n_angle_fix=  A_n_angle_fix; 
    out(i_ob).B_n_angle_fix = b_n_angle_fix;
    out(i_ob).h_dis = h_vel; 
    out(i_ob).A_n_dis = A_n_vel; 
    out(i_ob).B_n_dis = b_n_vel; 
        
    %test:
%     out(i_ob).h_angle_moving= NaN;  %cannot be 0?
%     out(i_ob).A_n_angle_moving  = A_n_anglemoving; 
%     out(i_ob).B_n_angle_moving = B_n_anglemoving;    
%     out(i_ob).h_angle_fix =  h_ang; 
%     out(i_ob).A_n_angle_fix=  A_n_angle_fix; 
%     out(i_ob).B_n_angle_fix = b_n_angle_fix;
%     out(i_ob).h_dis = h_vel; 
%     out(i_ob).A_n_dis = A_n_vel; 
%     out(i_ob).B_n_dis = b_n_vel; 
end

