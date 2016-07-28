function [pipe_T_CF, world_R_pipe, in_a_bend] = PipeEstimator(CF_T_cam, miscOut, angles, time)

head_hebi = miscOut.head_hebi;
module_positions =  reshape(miscOut.modules(1:3,4,:,:),3,size(miscOut.modules,3),[]);

% ESTIMATE BEND ANGLE
[d_theta, head_theta1, tail_theta1] = theta_head_tail(module_positions, time, head_hebi, angles);
bend_info = bend_angle_and_times(d_theta, time);

% ESTIMATE PIPE ORIENTATION + POSITION
[head_theta2, tail_theta2] = theta_head_tail_impose_bend_angle(bend_info, head_theta1, tail_theta1);

bend_info_updated1 = bend_position(module_positions, time, bend_info, head_theta2, tail_theta2, head_hebi, angles);

[head_theta3, tail_theta3] = theta_head_tail_VC(module_positions, bend_info_updated1);

bend_info_updated2 = bend_position(module_positions, time, bend_info_updated1, head_theta3, tail_theta3, head_hebi, angles);

[head_theta4, tail_theta4] = theta_head_tail_VC(module_positions, bend_info_updated2);

% pipe_T_CF
[pipe_T_CF, in_a_bend] = retrieve_pipe_T_CF(head_theta4, bend_info_updated2);
% world_R_pipe
world_R_pipe = retrieve_world_R_pipe(bend_info_updated2, time);





%%
pipe_T_cam = nan(4,4,length(pipe_T_CF));
for i=1:length(pipe_T_CF)
    pipe_T_cam(:,:,i) = pipe_T_CF(:,:,i)*CF_T_cam(:,:,i);
end

figure
subplot(1,2,1)
pipe_t_CF = reshape(pipe_T_CF(1:3,4,:),3,[]);
plot(time, pipe_t_CF);
legend('x','y','z')
xlabel('Time [sec]')
ylabel('Position [m]')
title('Position of Complementary Filter in pipe frame (pipe t CF)');

subplot(1,2,2)
pipe_t_cam = reshape(pipe_T_cam(1:3,4,:),3,[]);
plot(time, pipe_t_cam); hold on
plot([time(1) time(end)], [0.05 0.05])
plot([time(1) time(end)], -[0.05 0.05])

legend('x','y','z')
xlabel('Time [sec]')
ylabel('Position [m]')
title('Position of camera in pipe frame (pipe t cam)');


PLOT = false;
if PLOT
    plt = HebiPlotter('frame','head');
    for i=1:round(length(pipe_T_CF)/100):length(pipe_T_CF)
        plt.plot_with_head(angles(:,i), head_hebi(:,:,i), []); hold on
        drawframe(R2T(inv(pipe_T_CF(1:3,1:3,i))));
        pause(0.01)
        axis equal
    end
    title('CF T pipe')

    figure
    for i=1:round(length(pipe_T_CF)/100):length(pipe_T_CF)
        drawframe(pipe_T_cam(:,:,i)); hold on
        pause(0.01)
        axis equal
    end
    title('pipe T cam')
    figure
    for i=1:round(length(pipe_T_CF)/100):length(pipe_T_CF)
        drawframe(R2T(world_R_pipe(:,:,i))); hold on
        pause(0.01)
    end
    title('world R pipe')
end



%%
figure
plot(head_theta1); hold on
plot(head_theta2); hold on
plot(head_theta3); hold on
plot(head_theta4); hold on
plot(tail_theta1); hold on
plot(tail_theta2); hold on
plot(tail_theta3); hold on
plot(tail_theta4); hold on
legend('head estimate #1', ...
    'head estimate #2', ...
    'head estimate #3', ...
    'head estimate #4', ...
    'tail estimate #1', ...
    'tail estimate #2', ...
    'tail estimate #3', ...
    'tail estimate #4');
xlabel('Time [sec]')
ylabel('Position [m]')
title('Evolution of head/tail angle estimates');

figure
for b=1:length(bend_info_updated1)
    plot(bend_info_updated1(b).start_idx:bend_info_updated1(b).end_idx, ...
        bend_info_updated1(b).bend_percentage*100); hold on
end
for b=1:length(bend_info_updated1)
    plot(bend_info_updated1(b).start_idx:bend_info_updated1(b).end_idx, ...
        bend_info_updated2(b).bend_percentage*100); hold on
end
xlabel('Time [sec]')
ylabel('Distance from head [% of body length]')
legend('estimate #1', 'estimate #2');
title('Evolution of bend position along robot bosy');
%

end