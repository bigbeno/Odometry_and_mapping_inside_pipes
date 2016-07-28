function [angles, time] = Kinematics(run_log)

time = [run_log.t] -run_log(1).t;
num_joints = length(run_log(1).position);
angles = reshape([run_log.position], num_joints,[]);
VC = reshape([run_log.VC_frame], 4, 4, []);

PLOT = true;
%%
VC_x_axis = reshape(VC(1:3,1,:),3,[]);
windowSize = 4000;

filtered_VC_x_in_gravity(1,:) = filter_no_delay_with_beginning_and_end...
    (1:length(VC_x_axis), VC_x_axis(1,:), windowSize);
filtered_VC_x_in_gravity(2,:) = filter_no_delay_with_beginning_and_end...
    (1:length(VC_x_axis), VC_x_axis(2,:), windowSize);  
filtered_VC_x_in_gravity(3,:) = zeros(1,length(filtered_VC_x_in_gravity)); 
pipe_direction = filtered_VC_x_in_gravity(:,1);

if PLOT
    figure
    subplot(3,1,1)
    plot(time, VC_x_axis(1,:)); hold on
    plot(time, filtered_VC_x_in_gravity(1,:));
    legend('original', 'filtered');    
    xlabel('Time [sec]')    
    ylabel('x component [-]')
    title('VC main axis')
    subplot(3,1,2)
    plot(time, VC_x_axis(2,:)); hold on
    plot(time, filtered_VC_x_in_gravity(2,:));
    legend('original', 'filtered');
    xlabel('Time [sec]')    
    ylabel('y component [-]')
    subplot(3,1,3)
    plot(time, VC_x_axis(3,:)); hold on
    plot(time, filtered_VC_x_in_gravity(3,:));
    legend('original', 'filtered');    
    xlabel('Time [sec]')
    ylabel('z component [-]')
end
    
end