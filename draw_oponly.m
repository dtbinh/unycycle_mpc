
figure;

% DifferentialState xp_dot yp_dot  psi_dot epsi  ey s L;  %sometimes, the name of the variables induces errors, I don't know why. 
%     Control delta_f a_x;  
    
subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,7), 'r')
title('px');

subplot(2,2,2)
plot(out.STATES(:,1), out.STATES(:,6), 'r')
title('py');

subplot(2,2,3)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('v');

subplot(2,2,4)
plot(out.STATES(:,1), out.STATES(:,5), 'r')
title('psi');


figure;  
plot(out.STATES(:,1), out.STATES(:,8), 'r')
title('L');

figure;  
plot(out.STATES(:,1), out.CONTROLS(:,2), 'r')
title('u');


figure;
plot(out.STATES(:,1), 0.5.*out.STATES(:,2).*out.STATES(:,2), 'r')
title('Kinetic Engery');