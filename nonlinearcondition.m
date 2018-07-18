function [f,ceq] = nonlinearcondition(x)

% ocp.subjectTo( a^2+ 20^2*psi_dot^2 - (0.7*9.8)^2  <= 0); %slip constraints
    f =  ( x(1)^2+ 20^2*x(2)^2 - (0.7*9.8)^2 ); 
    f = [];
    ceq = [];             %non equation constraints 

end