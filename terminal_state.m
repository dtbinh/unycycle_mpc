function out = terminal_state(t)
%compute the terminal state for the optimal control problem 

%     ref[0] = 15*t;
%     ref[1] = 0;
%     ref[2] = 15; 
%     ref[3] = 0;
    v =5; 
    out = [v*t; 0; v; 0];