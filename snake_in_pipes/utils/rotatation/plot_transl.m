function plot_transl(t, time, title_name)
% Given a sequence of translation vectors 3xn and time 1xn
% plots their components over time


figure
ax11 = subplot(3,1,1);
plot(time, t(1,:)); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('x [m]'); 
xlabel('time [sec]') ;
title(['Translation ', title_name]);
ax21 = subplot(3,1,2);
plot(time, t(2,:)); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('y [m]'); 
xlabel('time [sec]') ;
ax31 = subplot(3,1,3);
plot(time, t(3,:)); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('z [m]'); 
xlabel('time [sec]');
linkaxes([ax11,ax21,ax31],'x');

end