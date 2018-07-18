function out = cbf_seperate(u)
%calculate the cbf constraints acoording to the states and the obstacles. 
%run in unicycle_c_seperate.m 

%input: 
%u: 4-by-1, is the feedback states 

%output: 
%out: 2-by-1, is  the coefficients of the constraints, [out(1), out(2)]*u <= out(3)   



px = u(1);
p_y = u(2);
v = u(3); 
psi_ = u(4); 

    cc = 1; 
    if (v>cc)
        matrix_line = [cos(psi_), -v*sin(psi_); sin(psi_),  v*cos(psi_)]; 
    else
        matrix_line = [cos(psi_), -cc*sin(psi_); sin(psi_),  cc*cos(psi_)]; 
    end

    
alpha = 4; 
gamma = 2; 
Ds = 1;

p_x_dot = v*cos(psi_);
p_y_dot = v*sin(psi_); 

pos_ob = [800; 0.5]; %obstacle 
vel_ob = [0; 0];

rel_pos = [px; p_y] - pos_ob;
rel_vel = [p_x_dot; p_y_dot] - vel_ob; 

norm_deltap = (norm(rel_pos)); 

if (norm_deltap -Ds < 0) 
    norm_deltap = Ds; 
end
norm_rel_p_dot = 0.5/norm_deltap*2*rel_pos'*rel_vel;

h = sqrt(2*alpha*(norm_deltap-Ds)) + rel_pos'/norm_deltap*rel_vel; 
coef_u =  rel_pos'/norm_deltap* matrix_line; 
% coef_u =  rel_pos'/norm_deltap; 
remaining   = 0.5*1/sqrt(2*alpha*norm_deltap-Ds) *2*alpha* norm_rel_p_dot +  (rel_vel'*norm_deltap - rel_pos'*norm_rel_p_dot)/norm_deltap^2*rel_vel...
    + gamma*h;

out = [-coef_u(1); -coef_u(2); remaining];


