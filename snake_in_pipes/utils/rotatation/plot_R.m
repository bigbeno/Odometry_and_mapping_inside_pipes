function plot_R(R, time, title_name)
%Given a sequence of rotation matrices 3x3xn and time 1xn
% plot the euler angles correponding to the rotations over time


[pitch, roll, yaw] = ...
        dcm2angleElena( R(1:3,1:3,:), 'YXZ' );

figure
ax11 = subplot(3,1,1);
plot(time, roll); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('roll [rad]') ; 
xlabel('time [sec]') ;
title(['Rotation ', title_name]);
ax21 = subplot(3,1,2);
plot(time, pitch); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('pitch [rad]') ; 
xlabel('time [sec]') ;
ax31 = subplot(3,1,3);
plot(time, yaw); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('yaw [rad]'); 
xlabel('time [sec]');
linkaxes([ax11,ax21,ax31],'x');

end