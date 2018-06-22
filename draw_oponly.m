
figure;

% DifferentialState px py v psi L;
%     Control a psi_dot;
    
subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('px');

subplot(2,2,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('py');

subplot(2,2,3)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('v');

subplot(2,2,4)
plot(out.STATES(:,1), out.STATES(:,5), 'r')
title('psi');


figure;  
plot(out.STATES(:,1), out.STATES(:,6), 'r')
title('L');

figure;  
plot(out.STATES(:,1), out.CONTROLS(:,2), 'r')
title('u');


figure;
plot(out.STATES(:,1), 0.5.*out.STATES(:,4).*out.STATES(:,4), 'r')
title('Kinetic Engery');