


main_tune;


for testii=1:50
    global pos_ob_array_pre;
    global name; 
    name = ['mpconly', num2str(testii),]; 

    for i=1:5
        pos_ob_array_pre(:,i) = [50+20*rand(1,1);  -2.7 + 5.4*rand(1,1) ];
    end
    
    unicycle_c_seperate;
    run bicycle_sim.m; 
    
end