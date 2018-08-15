

fails = [];

for testii=1:50

    name = ['mpc_20180811_testseries5_', num2str(testii),];  
%     name = ['mpc_cbf_20180812', num2str(testii),];
    load (name);  
    
    fail_flag = 0;

    t = t1;
    P_sens = y1(:, [6,5]); 
    
    for i = 1:length(t)
        
        if(fail_flag ==1)
                break;
        end
        for i_ob =1:5
            if(fail_flag ==1)
                break;
            end
            delta = pos_ob_array_pre(:,i_ob) - P_sens(i,:)';
%             if(norm(delta)<1.0)
            if(norm(delta)< radius_pre(i_ob)) || (abs(P_sens(i,2))>3.7)
                fails = [fails; testii];
                fail_flag = 1;
                break;
            end
            
        end
    end
end

