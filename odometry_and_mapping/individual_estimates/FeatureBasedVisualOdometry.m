function [camprev_T_cam, time_VO] = FeatureBasedVisualOdometry(filename)

import_visual_odometry_data;

time = time - time(1);
time_AO = time/1000; %[sec]
time = time_AO';

dR = [R11'; R12'; R13'; R21'; R22'; R23'; R31'; R32'; R33'];
dR_AO = reshape(dR,3,3,[]);

dcamera_AO_position = [x'; y'; z'];


T_AO = zeros(4,4,length(time_AO));
for i=1:length(time_AO)
    % CHANGING THE SIGN OF THE TRANSLATION
    T_AO(:,:,i) = [dR_AO(:,:,i), -dcamera_AO_position(:,i); 0 0 0 1];
end

camprev_T_cam = T_AO;
time_VO = time;

%%
[dpitch_AO, droll_AO, dyaw_AO] = ...
        dcm2angleElena( T_AO(1:3,1:3,:), 'YXZ' );
dAO_x = reshape(T_AO(1,4,:),1,[]);
dAO_y = reshape(T_AO(2,4,:),1,[]);
dAO_z = reshape(T_AO(3,4,:),1,[]);




figure
ax1 = subplot(3,2,1);
plot(time, dAO_x); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('dx [m]') 
xlabel('Time [sec]') 
title('VISUAL ODOMETRY: differential translation');
ax2 = subplot(3,2,3);
plot(time, dAO_y); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('dy [m]') 
xlabel('Time [sec]') 
ax3 = subplot(3,2,5);
plot(time, dAO_z); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('dz [m]') 
xlabel('Time [sec]') 
legend('CF',  'AO');
linkaxes([ax1,ax2,ax3],'x')
ax11 = subplot(3,2,2);
plot(time, droll_AO); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('d roll [rad]') ; 
xlabel('Time [sec]') ;
title(['differential rotation (dt:',num2str(mean(diff(time))), ...
    ' sec)']);
ax21 = subplot(3,2,4);
plot(time, dpitch_AO); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('d pitch [rad]') ; 
xlabel('Time [sec]') ;
ax31 = subplot(3,2,6);
plot(time, dyaw_AO); hold on
plot([min(time), max(time)], [0 0], '--k');
ylabel('d yaw [rad]'); 
xlabel('Time [sec]');
linkaxes([ax11,ax21,ax31],'x')


end