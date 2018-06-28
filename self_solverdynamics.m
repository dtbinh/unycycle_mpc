function [t1,y1]=self_solverdynamics(ode, tspan, y0, options, current_hdl)


% quad_3d_ode(t, y,  ctrl_hdl)
% [t1, y1] = ode45(@quad_3d_ode, tspan, y0, options, current_hdl);

dt=0.1;

n_state=length(y0);   %dimension of the states

t1=tspan(1):dt:tspan(end);
n_time=length(t1);

y1=zeros(n_time, n_state);
y1(1,:) = y0';
y=y0;

for i=2:n_time    
    t=t1(i);    
    dy = feval(ode, t, y,  current_hdl);
    y=y+dy*dt; 
    y1(i,:)=y';
end
