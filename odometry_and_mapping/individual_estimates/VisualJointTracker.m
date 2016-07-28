function [cam_t_joint, time_VO, visible_joint] = VisualJointTracker(filename)

import_visual_odometry_data;

time = time - time(1);
time_AO = time/1000; %[sec]
time = time_AO';

visible_joint = ones(1,length(time));
bad_odometry = find(roll==-1 & pitch==-1 & yaw==-1);
visible_joint(bad_odometry) = 0;

% FILTERING THE JOINT ABSOLUTE MOTION
CO_x = roll;
CO_y = pitch;
CO_z = yaw;
CO_x(bad_odometry) = NaN;
CO_y(bad_odometry) = NaN;
CO_z(bad_odometry) = NaN;


span_time = 1; %0.5;
windowSize = find(time_AO>span_time,1);
if mod(windowSize,2)
    windowSize = windowSize +1;
end
a = 1;
b = (1/windowSize)*ones(1,windowSize);

CO_x_f = filter(b,a,CO_x);
CO_y_f = filter(b,a,CO_y);
CO_z_f = filter(b,a,CO_z);
CO_x_f = [CO_x_f(windowSize/2: end); NaN(windowSize/2-1,1)];
CO_y_f = [CO_y_f(windowSize/2: end); NaN(windowSize/2-1,1)];
CO_z_f = [CO_z_f(windowSize/2: end); NaN(windowSize/2-1,1)];

visible_joint(isnan(CO_z_f)) = 0;

time_VO = time;
cam_t_joint = [CO_x_f, CO_y_f, CO_z_f];
cam_t_joint = cam_t_joint';

%%
figure;
x_h = plot(time, CO_x); hold on
plot(time, CO_x_f); hold on
y_h = plot(time, CO_y);
plot(time, CO_y_f);
z_h = plot(time, CO_z);
plot(time, CO_z_f);
legend([x_h, y_h , z_h], {'x', 'y', 'z'})
ylabel('Next joint center position [m]')
xlabel('Time [sec]')
title('VISUAL JOINT TRACKER')

end
