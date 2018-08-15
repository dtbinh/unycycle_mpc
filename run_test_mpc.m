
%% generate random obstacles: 
% clear all;
% no_ob = 5;
% pos_ob_array_pre_store = zeros(2,no_ob,50);
% radius_pre_store = zeros(no_ob, 50);
% for i_page=1:50
%     
%     flag_ok = 0; 
%     radius = zeros(no_ob,1);
%     %any two of the obstcles should not overlap with another 
%     while(flag_ok ==0)
%         for i=1:no_ob
%             radius(i) = 1+ 2.5*rand(1);
%             pos_ob(:,i) = [40+20*rand(1,1);  -2.7+ 3.4*rand(1,1) ];
%         end        
%         pos_ob(1,:) = sort(pos_ob(1,:));
%         
%         for i=1:(no_ob-1)
%             flagin = 0;
%             for j = (i+1):no_ob  
%                 norm_test = norm(pos_ob(:,i) -  pos_ob(:,j));
%                 if(norm_test <= radius(i)+radius(j))
%                     flagin= 1;
%                     break;
%                 end
%             end  
%             if(flagin==1)
%                    break;
%             end
%             if(i==no_ob-1) && (j==no_ob)
%                 flag_ok=1;
%             end
%         end    
%     end
%     
%     
%     pos_ob_array_pre_store(:,:,i_page) = pos_ob;
%     radius_pre_store(:, i_page)= radius;
% end
% 
% save pos_ob_array_pre_store3.mat pos_ob_array_pre_store radius_pre_store;

%% run simulation: 

clear all;
main_tune;

global pos_ob_array_pre radius_pre;  
global no_ob;
 
load pos_ob_array_pre_store4.mat; 

for testii=40:40
   
    global name; 
    global flag_mode; 
    no_ob = 5;    
    pos_ob_array_pre =  pos_ob_array_pre_store(:,:,testii);
    radius_pre = radius_pre_store(:,testii);
%     radius_pre = ones(5,1);  %tunning
    %mpc and cbf
%     flag_mode =1;     
%     name = ['mpc_cbf_20180810', num2str(testii),];         
%     unicycle_c_seperate;
%     run bicycle_sim.m; 
    
    %mpc only: 
    flag_mode =2; 
    name = ['mpc_20180815_testseries6_', num2str(testii),]; 
    unicycle_c_seperate;
    run bicycle_sim.m; 
    
end

load(name); plot_;