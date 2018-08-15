

clear all;
main_tune;

global pos_ob_array_pre radius_pre;
global no_ob;

load pos_ob_array_pre_store3.mat; 


for testii=22:22
   
    global name; 
    global flag_mode; 
    no_ob = 5;

    pos_ob_array_pre =  pos_ob_array_pre_store(:,:,testii);
    radius_pre = radius_pre_store(:,testii);
    
    %mpc and cbf
    flag_mode =1;     
    name = ['mpc_cbf_20180812', num2str(testii),];         
    unicycle_c_seperate;
    bicycle_sim; 
    
    %mpc only: 
%     flag_mode =2; 
%     name = ['mpc_20180810', num2str(testii),]; 
%     unicycle_c_seperate;
%     run bicycle_sim.m; 
    
end