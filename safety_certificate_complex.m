function output_safety = safety_certificate_complex(u)
%calculate the safet_certificate of the system

%u: the input and the state of the system and the obstacles 

%feedback states:
xp_dot = u(1);  %lateral speed
yp_dot = u(2);  %longitudinal speed
psi_dot = u(3); 
epsi = u(4);
ey= u(5);  %lateral position
s = u(6);  %logitudinal position 

%reference trajectory
tra_com = [u(7);u(8);u(9)];  %epsi, ey, s, notice the variables are in this order, velocity and acc are the same 
tra_com_dot = [u(10);u(11);u(12)];
tra_com_ddot = [u(13);u(14);u(15)]; 
time = u(16); 

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

%nominal input
% feedback linearization control to obtain the nominal input: 
k1= 9;
k2=2*1.5*sqrt(k1);    %very important, k2 and k1 must be tuned together. 

L_f_output =[  psi_dot - psi_dot_com
  yp_dot*cos(epsi) + xp_dot*sin(epsi)
  xp_dot*cos(epsi)-yp_dot*cos(epsi)];

% L_f_f_output =[(2*b*cr*psi_dot*(b + p))/(Iz*xp_dot) - (2*b*cr*yp_dot)/(Iz*xp_dot)
%  epsi*(- p*psi_dot^2 + yp_dot*psi_dot) - psi_dot*xp_dot + xp_dot*(psi_dot - psi_dot_com)
%                                                           - p*psi_dot^2 + yp_dot*psi_dot];  
%                                                       
% L_g_f_output =[[                 0,            (2*Fzf*a*mu)/Iz,                 0]
% [ (2*Fzf*epsi*mu)/m, (Fzf*mu*(2*a + 2*b))/(b*m), (2*Fzr*epsi*mu)/m]
% [      (2*Fzf*mu)/m,                          0,      (2*Fzr*mu)/m]];
L_f_f_output = [ - (yp_dot*(2*a*cf - 2*b*cr))/(Iz*xp_dot) - (psi_dot*(2*cf*a^2 + 2*cr*b^2))/(Iz*xp_dot)
 (psi_dot - psi_dot_com)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - cos(epsi)*(psi_dot*xp_dot + (psi_dot*(2*a*cf - 2*b*cr))/(m*xp_dot) + (yp_dot*(2*cf + 2*cr))/(m*xp_dot)) + psi_dot*yp_dot*sin(epsi)
         psi_dot*yp_dot*cos(epsi) - sin(epsi)*(psi_dot - psi_dot_com)*(xp_dot - yp_dot) + (cos(epsi)*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot))/(m*xp_dot)];
     
L_g_f_output = [[         (2*a*cf)/Iz,         0]
[  (2*cf*cos(epsi))/m, sin(epsi)]
[ -(2*cf*cos(epsi))/m, cos(epsi)]];

u_nom_lin=tra_com_ddot-k1*([epsi; ey; s]-tra_com)-k2*(L_f_output-tra_com_dot);
u_nom = pinv(L_g_f_output)*(u_nom_lin-L_f_f_output);   %feedback linearization

%bound for control
alpha=[1.0; 4];

%flag to determin if use the bounded input:
flag_bound = 0; 



% %test, July, 31th, 2018
% %%states of ob: 
% pos_ob_x = 200;
% pos_ob_y = 0;
% vel_ob_x =0;
% vel_ob_y =0;
% acc_ob_x = 0;
% acc_ob_y = 0;
% 
% pos_ob_x = 200 - 10*time;
% pos_ob_y = 0.1;
% vel_ob_x = -10;
% vel_ob_y = 0;
% acc_ob_x = 0;
% acc_ob_y = 0;
% 
% %safety distance: 
% Ds = 1.2;
% 
% 
% %single constraint test:
% %some states relating to obstacles: 
% v_vehicle = [xp_dot*cos(epsi)-yp_dot*cos(epsi); yp_dot*cos(epsi) + xp_dot*sin(epsi)];
% pos_ob= [pos_ob_x; pos_ob_y];
% rel_pos = [s; ey] - pos_ob;
% rel_vel = v_vehicle - [vel_ob_x; vel_ob_y]; %assume epsi is small, 
% 
% %constraint 1, angle constraint, test:  
% %notice there may be virtual number, nan, of inf, you should aoid these
% %conditions: 
% cos_rel_ang = (-rel_pos.'*rel_vel) /norm(rel_pos)/norm(rel_vel); 
% if (cos_rel_ang>=1)
%     cos_rel_ang = 1;
% elseif (cos_rel_ang<=-1)
%     cos_rel_ang =-1;
% end
% rel_ang = acos(cos_rel_ang); 
% % h_ang = rel_ang-asin(Ds/norm(rel_pos));
% 
% ratio = Ds/norm(rel_pos);
% %notice if do not deal carefully, there maybe virtual number appears. 
% if ratio>=1
%     ratio = 0.9999;
% end    
% h_ang = rel_ang - asin(ratio); 
% 
% %very important, if this is zero, may make the qp infeasible due to the
% %coefficient matrix be zeros, but the right matrix is negative 
% %sometimes, it may be NaN of inf, so this constraint actually do not work
% %noticed on July, 18th, 2018
% if(abs(pos_ob_y)<1e-4)    
%     pos_ob_y = sign(pos_ob_y)*1e-4;
% end
% if (abs(epsi) <= 1e-5)
%     epsi = sign(epsi)*1e-5;
% end
% 
% %notice, the longitudinal velocity is not be considered as state here 
% % L_f_h_ang = (yp_dot*cos(epsi) + xp_dot*sin(epsi))*((Ds*(ey - pos_ob_y))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - ((pos_ob_x - s)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) - cos(epsi)*(xp_dot - yp_dot)*((Ds*(pos_ob_x - s))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) + ((ey - pos_ob_y)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) + ((psi_dot - psi_dot_com)*(xp_dot*yp_dot - xp_dot^2 + vel_ob_x*xp_dot*cos(epsi) + vel_ob_y*xp_dot*sin(epsi) - vel_ob_x*yp_dot*sin(epsi) - vel_ob_y*yp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
% % L_g_h_ang = (2*cf*cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
% 
% L_f_h_ang_part1 = ((psi_dot - psi_dot_com)*(xp_dot*yp_dot - xp_dot^2 + vel_ob_x*xp_dot*cos(epsi) + vel_ob_y*xp_dot*sin(epsi) - vel_ob_x*yp_dot*sin(epsi) - vel_ob_y*yp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - ((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
% L_t_h_ang_part1 = -((ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi))*(pos_ob_y*vel_ob_x^3 - pos_ob_x*vel_ob_y^3 - ey*vel_ob_x^3 + s*vel_ob_y^3 - acc_ob_x*pos_ob_x^2*vel_ob_y + acc_ob_y*pos_ob_x^2*vel_ob_x - acc_ob_x*pos_ob_y^2*vel_ob_y + acc_ob_y*pos_ob_y^2*vel_ob_x - acc_ob_x*s^2*vel_ob_y + acc_ob_y*s^2*vel_ob_x - ey*vel_ob_x*vel_ob_y^2 - ey*vel_ob_x*xp_dot^2 - pos_ob_x*vel_ob_x^2*vel_ob_y + pos_ob_y*vel_ob_x*vel_ob_y^2 - pos_ob_x*vel_ob_y*xp_dot^2 + pos_ob_y*vel_ob_x*xp_dot^2 + s*vel_ob_x^2*vel_ob_y + s*vel_ob_y*xp_dot^2 - acc_ob_x*ey^2*vel_ob_y + acc_ob_y*ey^2*vel_ob_x + 2*acc_ob_x*ey*pos_ob_y*vel_ob_y - 2*acc_ob_y*ey*pos_ob_y*vel_ob_x + 2*acc_ob_x*pos_ob_x*s*vel_ob_y - 2*acc_ob_y*pos_ob_x*s*vel_ob_x - 2*ey*vel_ob_x*yp_dot^2*cos(epsi)^2 - 2*pos_ob_x*vel_ob_y*yp_dot^2*cos(epsi)^2 + 2*pos_ob_y*vel_ob_x*yp_dot^2*cos(epsi)^2 + 2*s*vel_ob_y*yp_dot^2*cos(epsi)^2 - acc_ob_y*ey^2*xp_dot*cos(epsi) + acc_ob_x*ey^2*yp_dot*cos(epsi) + acc_ob_y*ey^2*yp_dot*cos(epsi) - acc_ob_y*pos_ob_x^2*xp_dot*cos(epsi) - acc_ob_y*pos_ob_y^2*xp_dot*cos(epsi) + acc_ob_x*pos_ob_x^2*yp_dot*cos(epsi) + acc_ob_x*pos_ob_y^2*yp_dot*cos(epsi) + acc_ob_y*pos_ob_x^2*yp_dot*cos(epsi) + acc_ob_y*pos_ob_y^2*yp_dot*cos(epsi) - acc_ob_y*s^2*xp_dot*cos(epsi) + acc_ob_x*s^2*yp_dot*cos(epsi) + acc_ob_y*s^2*yp_dot*cos(epsi) + acc_ob_x*ey^2*xp_dot*sin(epsi) + 2*ey*vel_ob_x^2*xp_dot*cos(epsi) - 2*ey*vel_ob_x^2*yp_dot*cos(epsi) + acc_ob_x*pos_ob_x^2*xp_dot*sin(epsi) + acc_ob_x*pos_ob_y^2*xp_dot*sin(epsi) + acc_ob_x*s^2*xp_dot*sin(epsi) - 2*pos_ob_y*vel_ob_x^2*xp_dot*cos(epsi) + 2*pos_ob_x*vel_ob_y^2*yp_dot*cos(epsi) + 2*pos_ob_y*vel_ob_x^2*yp_dot*cos(epsi) - 2*s*vel_ob_y^2*yp_dot*cos(epsi) + 2*pos_ob_x*vel_ob_y^2*xp_dot*sin(epsi) - 2*s*vel_ob_y^2*xp_dot*sin(epsi) - 2*s*vel_ob_x*vel_ob_y*xp_dot*cos(epsi) + 2*s*vel_ob_x*vel_ob_y*yp_dot*cos(epsi) + 2*ey*vel_ob_x*vel_ob_y*xp_dot*sin(epsi) - 2*pos_ob_y*vel_ob_x*vel_ob_y*xp_dot*sin(epsi) + 2*ey*vel_ob_x*xp_dot*yp_dot*cos(epsi)^2 + 2*pos_ob_x*vel_ob_y*xp_dot*yp_dot*cos(epsi)^2 - 2*pos_ob_y*vel_ob_x*xp_dot*yp_dot*cos(epsi)^2 - 2*s*vel_ob_y*xp_dot*yp_dot*cos(epsi)^2 - ey*vel_ob_x*xp_dot*yp_dot*sin(2*epsi) - pos_ob_x*vel_ob_y*xp_dot*yp_dot*sin(2*epsi) + pos_ob_y*vel_ob_x*xp_dot*yp_dot*sin(2*epsi) + s*vel_ob_y*xp_dot*yp_dot*sin(2*epsi) + 2*acc_ob_y*ey*pos_ob_y*xp_dot*cos(epsi) - 2*acc_ob_x*ey*pos_ob_y*yp_dot*cos(epsi) - 2*acc_ob_y*ey*pos_ob_y*yp_dot*cos(epsi) + 2*acc_ob_y*pos_ob_x*s*xp_dot*cos(epsi) - 2*acc_ob_x*pos_ob_x*s*yp_dot*cos(epsi) - 2*acc_ob_y*pos_ob_x*s*yp_dot*cos(epsi) - 2*acc_ob_x*ey*pos_ob_y*xp_dot*sin(epsi) + 2*ey*vel_ob_x*vel_ob_y*yp_dot*cos(epsi) - 2*acc_ob_x*pos_ob_x*s*xp_dot*sin(epsi) + 2*pos_ob_x*vel_ob_x*vel_ob_y*xp_dot*cos(epsi) - 2*pos_ob_x*vel_ob_x*vel_ob_y*yp_dot*cos(epsi) - 2*pos_ob_y*vel_ob_x*vel_ob_y*yp_dot*cos(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2));
% L_g_h_ang_part1 = (2*cf*cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
%  
% L_f_h_ang_part2 = ((ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - ((xp_dot*cos(epsi) - yp_dot*cos(epsi))*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
% L_g_h_ang_part2 = 0;
% L_t_h_ang_part2 = -(ey*vel_ob_y - pos_ob_x*vel_ob_x - pos_ob_y*vel_ob_y + s*vel_ob_x)/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
% 
% ratio_pf = -Ds/ (norm(rel_pos))^2 *L_f_h_ang_part2; 
% ratio_pg = -Ds/ (norm(rel_pos))^2 *L_g_h_ang_part2; 
% ratio_pt = -Ds/ (norm(rel_pos))^2 *L_t_h_ang_part2; 
% asin_dot = 1/(1 - ratio^2)^(1/2); 
% L_f_h_ang=  L_f_h_ang_part1   -asin_dot*ratio_pf;
% L_g_h_ang=  L_g_h_ang_part1   -asin_dot*ratio_pg;
% L_t_h_ang=  L_t_h_ang_part1   -asin_dot*ratio_pt;
%  
% A_n = -L_g_h_ang;
% b_n = L_f_h_ang + L_t_h_ang +3*h_ang;
% H= diag([1]);
% f2 = -2* u_nom(1);  %the optimal goal is for the entire control
% optoption_1 = optimset('Display', 'off', 'TolFun', 1e-20);
% [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -1, 1, [], optoption_1);
% % [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
% 
% 
% 
% %constraint 2, velocity constraint, test:  
% %notice there may be virtual number, nan, of inf, you should aoid these
% %conditions: 
% a_m = 4;   %max acc
% dis_maxacc_sqr = 2*a_m*(norm(rel_pos)-Ds); 
% if (dis_maxacc_sqr<=0)
%     dis_maxacc_sqr = 1e-4;
% end
% h_vel  = sqrt(dis_maxacc_sqr) + rel_pos'/norm(rel_pos)*rel_vel; 
% 
% %very important, if this is zero, may make the qp infeasible due to the
% %coefficient matrix be zeros, but the right matrix is negative 
% %sometimes, it may be NaN of inf, so this constraint actually do not work
% %noticed on July, 18th, 2018
% if(abs(pos_ob_y)<1e-4)    
%     pos_ob_y = sign(pos_ob_y)*1e-4;
% end
% if (abs(epsi) <= 1e-5)
%     epsi = sign(epsi)*1e-5;
% end
% 
% % L_f_h_ang = (yp_dot*cos(epsi) + xp_dot*sin(epsi))*((Ds*(ey - pos_ob_y))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - ((pos_ob_x - s)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) - cos(epsi)*(xp_dot - yp_dot)*((Ds*(pos_ob_x - s))/((1 - Ds^2/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2))^(1/2)*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) + ((ey - pos_ob_y)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(1/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2))) + ((psi_dot - psi_dot_com)*(xp_dot*yp_dot - xp_dot^2 + vel_ob_x*xp_dot*cos(epsi) + vel_ob_y*xp_dot*sin(epsi) - vel_ob_x*yp_dot*sin(epsi) - vel_ob_y*yp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)) - (cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
% % L_g_h_ang = (2*cf*cos(epsi)*(vel_ob_x + vel_ob_y - xp_dot*cos(epsi) - xp_dot*sin(epsi))*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y - ey*xp_dot*cos(epsi) + ey*yp_dot*cos(epsi) + pos_ob_y*xp_dot*cos(epsi) - pos_ob_x*yp_dot*cos(epsi) - pos_ob_y*yp_dot*cos(epsi) + s*yp_dot*cos(epsi) - pos_ob_x*xp_dot*sin(epsi) + s*xp_dot*sin(epsi)))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)^(3/2)*(1 - ((pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)) + (ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))^2/(((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)*((vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))^2 + (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))^2)))^(1/2)); 
% 
% L_f_h_norm = ((ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - ((xp_dot*cos(epsi) - yp_dot*cos(epsi))*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
% L_g_h_norm = [ 0, 0];
% L_t_h_norm = -(ey*vel_ob_y - pos_ob_x*vel_ob_x - pos_ob_y*vel_ob_y + s*vel_ob_x)/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
% 
% L_f_h_vel = (((ey - pos_ob_y)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((pos_ob_x - s)*(xp_dot*sin(epsi) - yp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2))*(psi_dot - psi_dot_com) + (xp_dot*cos(epsi) - yp_dot*cos(epsi))*(((2*pos_ob_x - 2*s)*(ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((2*pos_ob_x - 2*s)*(pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - (yp_dot*cos(epsi) + xp_dot*sin(epsi))*(((ey - pos_ob_y)*(2*ey - 2*pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((pos_ob_x - s)*(2*ey - 2*pos_ob_y)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - psi_dot*yp_dot*((cos(epsi)*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - (sin(epsi)*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)) - (cos(epsi)*(psi_dot*xp_dot + (psi_dot*(2*a*cf - 2*b*cr))/(m*xp_dot) + (yp_dot*(2*cf + 2*cr))/(m*xp_dot))*(ey + pos_ob_x - pos_ob_y - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
% L_g_h_vel = [ (2*cf*cos(epsi)*(ey + pos_ob_x - pos_ob_y - s))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)), (sin(epsi)*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) - (cos(epsi)*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2)];
% L_t_h_vel = vel_ob_y*(((ey - pos_ob_y)*(2*ey - 2*pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((pos_ob_x - s)*(2*ey - 2*pos_ob_y)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - vel_ob_x*(((2*pos_ob_x - 2*s)*(ey - pos_ob_y)*(yp_dot*cos(epsi) - vel_ob_y + xp_dot*sin(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2)) - (vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + ((2*pos_ob_x - 2*s)*(pos_ob_x - s)*(vel_ob_x - xp_dot*cos(epsi) + yp_dot*cos(epsi)))/(2*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(3/2))) - (acc_ob_y*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2) + (acc_ob_x*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^(1/2);
% 
% temp_h_vel = 1/2*1/sqrt(dis_maxacc_sqr)*2*a_m; 
%  
% L_f_h_vel= temp_h_vel* L_f_h_norm+L_f_h_vel;
% L_g_h_vel= temp_h_vel* L_g_h_norm+L_g_h_vel;
% L_t_h_vel= temp_h_vel* L_t_h_norm+L_t_h_vel;
%  
% A_n = -L_g_h_vel;
% b_n = L_f_h_vel + L_t_h_vel +3*h_vel;
% H= diag([1,1]);
% f2 = -2* u_nom;  %the optimal goal is for the entire control
% optoption_1 = optimset('Display', 'off', 'TolFun', 1e-20);
% [x_vel, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
% % [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
% 
% 
% %constraint 3, velocity direction constraint for moving obstacles, test:  
% 
% %notice there may be virtual number, nan, of inf, you should aoid these
% %conditions: 
% rel_pos_vert =  [-rel_pos(2); rel_pos(1)];  %normal to rel_pos
% vel_ob = [vel_ob_x; vel_ob_y];
% % h_move_angle = -rel_pos_vert'* v_vehicle* rel_pos_vert'*vel_ob/norm(rel_pos_vert)/norm(rel_pos_vert) ;
% h_move_angle = -rel_pos_vert'* v_vehicle* rel_pos_vert'*vel_ob  ;
% %very important, if this is zero, may make the qp infeasible due to the
% %coefficient matrix be zeros, but the right matrix is negative 
% %sometimes, it may be NaN of inf, so this constraint actually do not work
% %noticed on July, 18th, 2018
% % if(abs(pos_ob_y)<1e-4)    
% %     pos_ob_y = sign(pos_ob_y)*1e-4;
% % end
% % if (abs(epsi) <= 1e-5)
% %     epsi = sign(epsi)*1e-5;
% % end
% 
% L_f_h_move_angle = cos(epsi)*(xp_dot - yp_dot)*(vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi))) - (yp_dot*cos(epsi) + xp_dot*sin(epsi))*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot)) - (vel_ob_y*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s) + vel_ob_x*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y))*(psi_dot - psi_dot_com) - psi_dot*yp_dot*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey*cos(epsi) - pos_ob_y*cos(epsi) + pos_ob_x*sin(epsi) - s*sin(epsi)) - (cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s)*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot))/(m*xp_dot);
% L_g_h_move_angle = [ (2*cf*cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s))/m, - vel_ob_x*(ey - pos_ob_y)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s)) - vel_ob_y*(pos_ob_x - s)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s))];
% L_t_h_move_angle = vel_ob_y*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot)) - vel_ob_x*(vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi))) - acc_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) - acc_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s);
% 
% % L_f_h_move_angle = cos(epsi)*(xp_dot - yp_dot)*((vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((2*pos_ob_x - 2*s)*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2) - ((vel_ob_y*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s) + vel_ob_x*((pos_ob_x - s)*(xp_dot*cos(epsi) - yp_dot*sin(epsi)) - sin(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y))*(psi_dot - psi_dot_com))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s))*(2*ey - 2*pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) - (psi_dot*yp_dot*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey*cos(epsi) - pos_ob_y*cos(epsi) + pos_ob_x*sin(epsi) - s*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - (cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s)*(m*psi_dot*xp_dot^2 + 2*cf*yp_dot + 2*cr*yp_dot + 2*a*cf*psi_dot - 2*b*cr*psi_dot))/(m*xp_dot*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2));
% % L_g_h_move_angle = [ (2*cf*cos(epsi)*(ey*vel_ob_x + pos_ob_x*vel_ob_y - pos_ob_y*vel_ob_x - s*vel_ob_y)*(ey - pos_ob_x - pos_ob_y + s))/(m*((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)), -(vel_ob_x*(ey - pos_ob_y)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s)) + vel_ob_y*(pos_ob_x - s)*(cos(epsi)*(ey - pos_ob_y) + sin(epsi)*(pos_ob_x - s)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)];
% % L_t_h_move_angle = vel_ob_y*((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot) + vel_ob_y*cos(epsi)*(pos_ob_x - s)*(xp_dot - yp_dot))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s))*(2*ey - 2*pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2) - vel_ob_x*((vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot)) + vel_ob_x*(ey - pos_ob_y)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + vel_ob_y*(pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - ((2*pos_ob_x - 2*s)*(vel_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y) + vel_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s)))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2)^2) - (acc_ob_x*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(ey - pos_ob_y))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2) - (acc_ob_y*((pos_ob_x - s)*(yp_dot*cos(epsi) + xp_dot*sin(epsi)) + cos(epsi)*(ey - pos_ob_y)*(xp_dot - yp_dot))*(pos_ob_x - s))/((pos_ob_x - s)^2 + (ey - pos_ob_y)^2);
% 
% A_n = -L_g_h_move_angle;
% b_n = L_f_h_move_angle + L_t_h_move_angle + 3*h_move_angle;
% H= diag([1,1]);
% f2 = -2* u_nom;  %the optimal goal is for the entire control
% optoption_1 = optimset('Display', 'off', 'TolFun', 1e-20);
% % [x_move_angle, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, [], optoption_1);
% [x_move_angle, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], [], optoption_1);
% 
% A_n_angle = -L_g_h_ang;
% b_n_angle = L_f_h_ang + L_t_h_ang +3*h_ang;
% A_n = [A_n_angle, 0; A_n];
% b_n = [b_n_angle; b_n];
% 
% [x_comp, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], [], optoption_1);

%actual control 
% u_nom(1) = x;
% u_nom  =  x_comp;

%% for dynamic obstacles:  
global results_2; 
% results_2 = constraint_obstacles_dynamics([p_x; p_y; v; psi], time); 
results_2 = constraint_obstacles_dynamics_complex([xp_dot; yp_dot; psi_dot; epsi; ey; s], time ); 

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
     f2 = -2* u_nom;  %the optimal goal is for the entire control
     optoption_1 = optimset('Display', 'off', 'TolFun', 1e-15);
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
        end   
 
%     end
    
%     if(delta_just2.max < max_delta_lb) 
%         max_delta_lb = delta_just2.max; 
%         x_min = x; 
%     end
end
 
% the output: 
if (value_min~=100000000) 
% if (max_delta_lb<0)
    %select the minimal solution 
     out= x_min; 
else
    %no solution exsits, brake  
    out = [0; -alpha(2)];
end






%if test
testflag = 0;
if (testflag ==1)
%     global results_2; 
%     results_2 = constraint_obstacles_dynamics([p_x; p_y; v; psi], 0); 
%     result_dynamic = constraint_obstacles_dynamics([p_x; p_y; v; psi], time); 
%     no_ob_dynamics = size(result_dynamic,2);
% 
%     %define empty matrix 
%     A_n = [];
%     b_n =[];    
%     for aa = 1:no_ob_dynamics
%         A_n = [A_n; result_dynamic(aa).A_n_dis];
%         b_n = [b_n; result_dynamic(aa).B_n_dis];  
%     end
% 
%     f2 = -2* u_nom;  %the optimal goal is for the entire control: norm(x-bar x), notice 
% 
%     H =eye(2,2);
% flag_bound = 0; 
% alpha= [4;4];
%     if(flag_bound ==0)
%         [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], -alpha, alpha, []);
%     else
%         [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], []);
%     end
% 
%     value_min = 100;
%     if (EXITFLAG < 0)  
%     %qp  has no solution 
%         out = [-alpha(1); 0];   %no solution, brake  
%     else 
%         out = x; 
%     end
end 

%  A_n = [  results_2(1).A_n_angle_fix; results_2(1).A_n_angle_moving];
%  b_n = [ results_2(1).B_n_angle_fix;  results_2(1).B_n_angle_moving ]; 
%  [x, FVAL, EXITFLAG] = quadprog(H, f2, A_n, b_n, [], [], [], [], [], optoption_1);

% if (time>6.8)
%     out = u_nom;
% end

global output_safety; 
output_safety = [out; ...
    results_2(1).A_n_angle_fix(1); results_2(1).A_n_angle_fix(2); results_2(1).B_n_angle_fix; ...
    results_2(1).A_n_angle_moving(1); results_2(1).A_n_angle_moving(2); results_2(1).B_n_angle_moving; ...
    results_2(1).A_n_dis(1); results_2(1).A_n_dis(2); results_2(1).B_n_dis; ...
    results_2(1).h_angle_fix; results_2(1).h_angle_moving;  results_2(1).h_dis; beta_2(1); value_min];
% output_safety = [out; h_move_angle; A_n(1,1); A_n(1,2); b_n(1); zeros(5,1)];
 

