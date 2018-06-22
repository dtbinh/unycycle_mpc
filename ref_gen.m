function out = ref_gen()

%generate the reference trajectory for the system 

%output: 

%the first column is the time series 
% the other columns are the state
%each row corresponds to each time instant 

%n-by-(1+m): n is the time instants, m is the dimension of the states 


t0=0;
tf=20; 
samp_T = 0.1;

t_seri = t0:samp_T:tf;

len = length(t_seri); 

out = zeros(len, 5);

for i=1:len
    out(i,1) = t_seri(i);
    
    px_i = 10*t_seri(i);
    py_i = 0; 
    v_i = 10; 
    psi_i = 0;
    
    out(i,2:end) = [px_i py_i v_i psi_i];

end
