function control = series_oc(u)
%construct the MPC with CBF constraints; 


%input: 
%u: 



%input: 
state_fb = [u(1); u(2); u(3); u(4)];  %note this is the predicted states

t0 = u(5); 

%should be tunned: 
horizon = 1; 
t_sampl = 0.1; 

t_series = t0:t_sampl:t0; 

N = length(t_series); 

for i=1:N
    coef_ =  cbf_seperate(state_fb);   %cefficients given by cbf
    t0_i = t_series(i);
    out = unicycle_input_RUN(t0_i, t0_i+t_sampl, y(1), y(2), y(3), y(4), coef_(1), coef_(2), coef_(3) ); 
end

u = out.CONTROLS(1,2:end)'; 