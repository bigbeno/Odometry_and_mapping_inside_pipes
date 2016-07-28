
load(truth_name);

%% MAP

figure

truth_bend_coord = zeros(3,length(truth.pipes));
absolute_angle = 0;
for i=1:length(truth.pipes)
    current_pipe= truth.pipes(i);
    absolute_angle = truth.pipes(i).angle +absolute_angle;
    truth_bend_coord(1,i+1) = [...
        truth_bend_coord(1,i)+current_pipe.length*cos(deg2rad(-absolute_angle))/100];
    truth_bend_coord(2,i+1) = [...
        truth_bend_coord(2,i)+current_pipe.length*sin(deg2rad(-absolute_angle))/100];

end
truth_map = plot3(truth_bend_coord(1,:),...
        truth_bend_coord(2,:), ...
        truth_bend_coord(3,:),'r','LineWidth',3); hold on


w_t_pipe = x(19:21,:);

end_point = x(5:7,end);
last_bend = x(19:21,end);
end_map = last_bend + quat2dcmElena(x(15:18,end)')*norm(end_point-last_bend)*[1 0 0 ]';
w_t_pipe_extended = [w_t_pipe, end_map];

EKF_map = plot3(w_t_pipe_extended(1,:), w_t_pipe_extended(2,:), w_t_pipe_extended(3,:),'*-k', 'LineWidth', 3);


legend([truth_map, EKF_map], {'Truth','EKF'});
xlabel('x [m]')
ylabel('y [m]')
title('Map')
axis equal
disp('end of map')
pause()

%% HEAD TRACE

head_trace = plot3(x(5,:), x(6,:), x(7,:), '.b');
legend([truth_map, EKF_map, head_trace], {'Truth','EKF', 'EKF robot head trajectory'});
disp('end of head trace')
pause()

%% PIPES

%find bends
norm_w_t_pipe = sqrt(sum(w_t_pipe_extended.^2));
difference_of_norm_w_t_pipe = diff(norm_w_t_pipe);
bend_idx = find(difference_of_norm_w_t_pipe>0.1);
bends = [[0 0 0]', w_t_pipe_extended(:,bend_idx+1)];

for i=2:length(bends)
    pipe_dir = bends(:,i)-bends(:,i-1);
    pipe_dir = pipe_dir/norm(pipe_dir);
    [X, Y, Z] = pipe_from_pipe_direction_and_bend_pos...
                (pipe_dir, bends(1:2,i-1), bends(1:2,i));   
    surf(X,Y,Z , 'FaceAlpha', 0, 'LineWidth', 0.1, 'EdgeAlpha',0.3); hold on  
            
end

disp('showed pipes')
pause();


%% PLOT SNAKE
plt = HebiPlotter('frame','head', 'figHandle', gcf);
CF_t_hebi = [0 -1 0 0; 1 0 0 0; 0 0 1 -.03; 0 0 0 1]; 
for i=1:round(length(time)/100):length(time)
    
   [w_T_cam_T, w_T_camp_T, w_T_pipe_T, pipe_T_CF_T] = transform_xqt_in_xT(x(:,i));  
   plt.plot_with_head(angles(:,i), w_T_cam_T*CF_t_hebi, []); hold on
     
    hold off
    pause(0.001)
    
end

%% PLOT ODOMETRY

if isfield(truth.pipes, 'end_time') && ...
        length([truth.pipes.end_time])==length(truth_bend_coord)-1

    diag3=@(a) a(sub2ind(size(a),repmat((1:size(a,1))',[1, size(a,3)]),...
                               repmat((1:size(a,2))',[1, size(a,3)]),...
                               repmat(1:size(a,3)',[size(a,1), 1])));
    std_diag = sqrt(diag3(P));
    w_t_cam = x(5:7,:);
    k = 3;
    figure
    ax1 =subplot(1,3,1);
    plot([0,[truth.pipes.end_time]], truth_bend_coord(1,:), 'k', 'LineWidth',2); hold on
    plot(time, w_t_cam(1,:),'.-'); hold on 
    plot(time, w_t_cam(1,:)+ k*sqrt(std_diag(5,:)),'--b', 'LineWidth', 0.5); hold on
    plot(time, w_t_cam(1,:)- k*sqrt(std_diag(5,:)),'--b', 'LineWidth', 0.5); hold on
    legend('Truth', 'EKF')
    xlabel('Time [sec]');    ylabel('x [m]')
    ax2 = subplot(1,3,2);
    plot([0,[truth.pipes.end_time]], truth_bend_coord(2,:), 'k', 'LineWidth',2); hold on
    plot(time, w_t_cam(2,:),'.-'); hold on 
    plot(time, w_t_cam(2,:)+ k*sqrt(std_diag(6,:)),'--b', 'LineWidth', 0.5); hold on
    plot(time, w_t_cam(2,:)- k*sqrt(std_diag(6,:)),'--b', 'LineWidth', 0.5); hold on
    legend('Truth', 'EKF')
    xlabel('Time [sec]');    ylabel('y [m]');    title('Odometry')
    ax3 = subplot(1,3,3);
    plot([0,[truth.pipes.end_time]], truth_bend_coord(3,:), 'k', 'LineWidth',2); hold on
    plot(time, w_t_cam(3,:),'.-'); hold on 
    plot(time, w_t_cam(3,:)+ k*sqrt(std_diag(7,:)),'--b', 'LineWidth', 0.5); hold on
    plot(time, w_t_cam(3,:)- k*sqrt(std_diag(7,:)),'--b', 'LineWidth', 0.5); hold on
    legend('Truth', 'EKF')
    ylabel('z [m]');    xlabel('Time [sec]')

    linkaxes([ax1 ax2 ax3],'xy')
else
    if     isfield(truth, 'x')
        diag3=@(a) a(sub2ind(size(a),repmat((1:size(a,1))',[1, size(a,3)]),...
                               repmat((1:size(a,2))',[1, size(a,3)]),...
                               repmat(1:size(a,3)',[size(a,1), 1])));
        std_diag = sqrt(diag3(P));
        w_t_cam = x(5:7,:);
        k = 3;
        figure
        ax1 =subplot(1,3,1);
        plot(truth.time, truth.x, 'k', 'LineWidth',2); hold on
        plot(time, w_t_cam(1,:),'.-'); hold on 
        plot(time, w_t_cam(1,:)+ k*sqrt(std_diag(5,:)),'--b', 'LineWidth', 0.5); hold on
        plot(time, w_t_cam(1,:)- k*sqrt(std_diag(5,:)),'--b', 'LineWidth', 0.5); hold on
        legend('Truth', 'EKF')
        xlabel('Time [sec]');    ylabel('x [m]')        
        ax2 = subplot(1,3,2);
        plot(truth.time, truth.y, 'k', 'LineWidth',2); hold on
        plot(time, w_t_cam(2,:),'.-'); hold on 
        plot(time, w_t_cam(2,:)+ k*sqrt(std_diag(6,:)),'--b', 'LineWidth', 0.5); hold on
        plot(time, w_t_cam(2,:)- k*sqrt(std_diag(6,:)),'--b', 'LineWidth', 0.5); hold on
        legend('Truth', 'EKF')
        xlabel('Time [sec]');    ylabel('y [m]');    title('Odometry')
        ax3 = subplot(1,3,3);
        plot(truth.time, zeros(length(truth.x)), 'k', 'LineWidth',2); hold on
        plot(time, w_t_cam(3,:),'.-'); hold on 
        plot(time, w_t_cam(3,:)+ k*sqrt(std_diag(7,:)),'--b', 'LineWidth', 0.5); hold on
        plot(time, w_t_cam(3,:)- k*sqrt(std_diag(7,:)),'--b', 'LineWidth', 0.5); hold on
        legend('Truth', 'EKF')
        ylabel('z [m]');    xlabel('Time [sec]')
        
    else
        
    disp('no true odometry to plot against')
    end
end




