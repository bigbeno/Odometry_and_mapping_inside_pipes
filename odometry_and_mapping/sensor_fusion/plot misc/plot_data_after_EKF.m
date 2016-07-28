%plot_data_after_EKF

%% PLOT TRAJECTORY OF STATES

diag3=@(a) a(sub2ind(size(a),repmat((1:size(a,1))',[1, size(a,3)]),...
                           repmat((1:size(a,2))',[1, size(a,3)]),...
                           repmat(1:size(a,3)',[size(a,1), 1])));
std_diag = sqrt(diag3(P));
quat = sym('quat', [1 4]);
sym(quat, 'real');
[quat2angle(1), quat2angle(2), quat2angle(3)] = quat2angleElena(quat, 'YXZ');
Jacobian_quat2angle_sym = jacobian(quat2angle', quat);
Jacobian_quat2angle_sthg = symfun(Jacobian_quat2angle_sym, quat);
Jacobian_quat2angle_f = matlabFunction(Jacobian_quat2angle_sthg);
for i=1:length(x)
    quat = num2cell(x(1:4,i)');
    Jacobian_quat2angle_current =  Jacobian_quat2angle_f(quat{:});
    w_angle_cam_sigma(:,:,i) = Jacobian_quat2angle_current*P(1:4,1:4,i)*Jacobian_quat2angle_current';
    w_angle_pipe_sigma(:,:,i) = Jacobian_quat2angle_current*P(15:18,15:18,i)*Jacobian_quat2angle_current';
    pipe_angle_CF_sigma(:,:,i) = Jacobian_quat2angle_current*P(22:25,22:25,i)*Jacobian_quat2angle_current';    
end
std_cam_angles_diag = sqrt(diag3(w_angle_cam_sigma));
std_pipe_angles_diag = sqrt(diag3(w_angle_pipe_sigma));
std_CF_angles_diag = sqrt(diag3(pipe_angle_CF_sigma));

in_bend_all = reshape(sum(isnan(z_PE_t_CF))==0,1,[]);
b_start = find(diff(in_bend_all)==1);
b_end = find(diff(in_bend_all)==-1);
if length(b_end)< length(b_start)
    b_end = [b_end, length(time)];
end
for i=1:length(b_start)
x_data_bi(i,:) = [time(b_start(i)), time(b_end(i)), time(b_end(i)), time(b_start(i))];
y_data_bi(i,:) = [0 0 1 1];
end

k = 0.1;
%%% W T cam
w_t_cam = x(5:7,:);
[pitch, roll, yaw] = quat2angleElena( x(1:4,:)', 'YXZ' );
figure
subplot(4,2,[1 3])
x_h = plot(time, w_t_cam,'.-'); hold on 
ax = gca; ax.ColorOrderIndex = 1;
plot(time, w_t_cam+ k*sqrt(std_diag(5:7,:)),'--', 'LineWidth', 0.5); hold on
ax.ColorOrderIndex = 1;
plot(time, w_t_cam- k*sqrt(std_diag(5:7,:)),'--', 'LineWidth', 0.5); hold on
y_limit = ylim;
for i=1:length(b_start)
    patch_h =patch('XData',x_data_bi(i,:), ...
        'YData',y_data_bi(i,:)*(y_limit(2)-y_limit(1))+y_limit(1), ...
        'FaceAlpha',0.1);
end
legend([x_h; patch_h], {'x','y','z', 'in bend'})

ylabel('w t cam'); xlabel('Time [sec]')
subplot(4,2,[5 7])
x_h = plot(time,[ pitch, roll, yaw] ,'.-'); hold on
ax = gca; ax.ColorOrderIndex = 1;
plot(time,[ pitch, roll, yaw] + k*std_cam_angles_diag','--', 'LineWidth', 0.5); hold on
ax.ColorOrderIndex = 1;
plot(time,[ pitch, roll, yaw] - k*std_cam_angles_diag','--', 'LineWidth', 0.5); hold on
y_limit = ylim;
for i=1:length(b_start)
    patch_h = patch('XData',x_data_bi(i,:), ...
        'YData',y_data_bi(i,:)*(y_limit(2)-y_limit(1))+y_limit(1), ...
        'FaceAlpha',0.1);
end
legend([x_h; patch_h], {'pitch','roll','yaw', 'in bend'})
ylabel('w R cam');xlabel('Time [sec]')

w_t_pipe = x(19:21,:);
[pitch_pipe, roll_pipe, yaw_pipe] = quat2angleElena( x(15:18,:)', 'YXZ' );  
subplot(4,2,2)
x_h = plot(time,w_t_pipe,'.-'); hold on
ax = gca; ax.ColorOrderIndex = 1;
plot(time, w_t_pipe+ k*sqrt(std_diag(19:21,:)),'--', 'LineWidth', 0.5); hold on
ax.ColorOrderIndex = 1;
plot(time, w_t_pipe- k*sqrt(std_diag(19:21,:)),'--', 'LineWidth', 0.5); hold on
ylabel('w t pipe');xlabel('Time [sec]')
y_limit = ylim;
for i=1:length(b_start)
    patch_h = patch('XData',x_data_bi(i,:), ...
        'YData',y_data_bi(i,:)*(y_limit(2)-y_limit(1))+y_limit(1), ...
        'FaceAlpha',0.1);
end
legend([x_h; patch_h], {'x','y','z', 'in bend'})

subplot(4,2,4)
x_h = plot(time, [pitch_pipe, roll_pipe, yaw_pipe] ,'.-'); hold on
ax = gca; ax.ColorOrderIndex = 1;
plot(time,[ pitch_pipe, roll_pipe, yaw_pipe] + k*std_pipe_angles_diag','--', 'LineWidth', 0.5); hold on
ax.ColorOrderIndex = 1;
plot(time,[ pitch_pipe, roll_pipe, yaw_pipe] - k*std_pipe_angles_diag','--', 'LineWidth', 0.5); hold on
ax = gca; ax.ColorOrderIndex = 1;
y_limit = ylim;
for i=1:length(b_start)
    patch_h = patch('XData',x_data_bi(i,:), ...
        'YData',y_data_bi(i,:)*(y_limit(2)-y_limit(1))+y_limit(1), ...
        'FaceAlpha',0.1);
end
ylabel('w R pipe');xlabel('Time [sec]')
legend([x_h; patch_h], {'pitch','roll','yaw', 'in bend'})


pipe_t_CF = x(26:28,:);
[pitch_CF, roll_CF, yaw_CF] = quat2angleElena( x(22:25,:)', 'YXZ' );    
subplot(4,2,6)
x_h = plot(time,pipe_t_CF,'.-'); hold on
ax = gca; ax.ColorOrderIndex = 1;
plot(time, pipe_t_CF+ k*sqrt(std_diag(26:28,:)),'--', 'LineWidth', 0.5); hold on
ax.ColorOrderIndex = 1;
plot(time, pipe_t_CF- k*sqrt(std_diag(26:28,:)),'--', 'LineWidth', 0.5); hold on
ax = gca; ax.ColorOrderIndex = 1;
y_limit = ylim;
for i=1:length(b_start)
    patch_h = patch('XData',x_data_bi(i,:), ...
        'YData',y_data_bi(i,:)*(y_limit(2)-y_limit(1))+y_limit(1), ...
        'FaceAlpha',0.1);
end
ylabel('pipe t CF');xlabel('Time [sec]')
legend([x_h; patch_h], {'x','y','z', 'in bend'})

subplot(4,2,8)
x_h = plot(time, [pitch_CF, roll_CF, yaw_CF] ,'.-'); hold on
ax = gca; ax.ColorOrderIndex = 1;
plot(time,[ pitch_CF, roll_CF, yaw_CF] + k*std_CF_angles_diag','--', 'LineWidth', 0.5); hold on
ax.ColorOrderIndex = 1;
plot(time,[ pitch_CF, roll_CF, yaw_CF] - k*std_CF_angles_diag','--', 'LineWidth', 0.5); hold on
ax = gca; ax.ColorOrderIndex = 1;
y_limit = ylim;
for i=1:length(b_start)
    patch_h = patch('XData',x_data_bi(i,:), ...
        'YData',y_data_bi(i,:)*(y_limit(2)-y_limit(1))+y_limit(1), ...
        'FaceAlpha',0.1);
end
ylabel('pipe R CF');xlabel('Time [sec]') 
legend([x_h; patch_h], {'pitch','roll','yaw', 'in bend'})


% return

%% PLOT COVARIANCES

P_norm = nan(1,length(P));
for i=1:length(P)
    P_norm(i) = norm(P(:,:,i));
    P_norm_w_q_cam(i) = norm(P(1:4,1:4,i));
    P_norm_w_t_cam(i) = norm(P(5:7,5:7,i));
    P_norm_w_q_camp(i) = norm(P(8:11,8:11,i));
    P_norm_w_t_camp(i) = norm(P(12:14,12:14,i));
    P_norm_w_q_pipe(i) = norm(P(15:18,15:18,i));
    P_norm_w_t_pipe(i) = norm(P(19:21,19:21,i));
    P_norm_pipe_q_CF(i) = norm(P(22:25,22:25,i));
    P_norm_pipe_t_CF(i) = norm(P(26:28,26:28,i));    
end
figure
subplot(3,3,1)
plot(time, P_norm_w_q_cam);xlabel('Time [sec]') 
ylabel('P norm w q cam')
subplot(3,3,2)
plot(time, P_norm_w_t_cam);xlabel('Time [sec]') 
ylabel('P norm w t cam')
title('Covariance matrix norm')
subplot(3,3,3)
plot(time, P_norm_w_q_camp);xlabel('Time [sec]') 
ylabel('P norm w q camp')
subplot(3,3,4)
plot(time, P_norm_w_t_camp);xlabel('Time [sec]') 
ylabel('P norm w t camp')
subplot(3,3,5)
plot(time, P_norm_w_q_pipe);xlabel('Time [sec]') 
ylabel('P norm w q pipe')
subplot(3,3,6)
plot(time, P_norm_w_t_pipe);xlabel('Time [sec]') 
ylabel('P norm w t pipe')
subplot(3,3,7)
plot(time, P_norm_pipe_q_CF);xlabel('Time [sec]') 
ylabel('P norm pipe q CF')
subplot(3,3,8)
plot(time, P_norm_pipe_t_CF);xlabel('Time [sec]') 
ylabel('P norm pipe t CF')
subplot(3,3,9)
plot(time, P_norm);xlabel('Time [sec]') 
ylabel('P norm')



%% PLOT SNAKE
plt = HebiPlotter('frame','head')
CF_t_hebi = [0 -1 0 0; 1 0 0 0; 0 0 1 -.03; 0 0 0 1]; 
for i=1:round(length(time)/20):length(time)
    
   [w_T_cam_T, w_T_camp_T, w_T_pipe_T, pipe_T_CF_T] = transform_xqt_in_xT(x(:,i));  
   plt.plot_with_head(angles(:,i), w_T_cam_T*CF_t_hebi, []); hold on
   
    drawframe_multiple_colors(w_T_cam_T, 0.1, 1); hold on         
    drawframe_multiple_colors(w_T_pipe_T, 0.1, 2); hold on        
    drawframe_multiple_colors(w_T_pipe_T*pipe_T_CF_T, 0.1); hold on    
    hold off
    
    ylabel('world T cam')
end

return
%% CHECK
% If Filter output agrees with Complementary Filter + Pipe Estimator
CF_t_hebi = [0 -1 0 0; 1 0 0 0; 0 0 1 -.03; 0 0 0 1];  

plt = HebiPlotter('frame','head');
world_T_pipe = eye(4);
pipe_t_CF = zeros(3,1);
pipe_T_CF = [quat2dcmElena(z_PE_R_CF(1:4,1)'), pipe_t_CF; 0 0 0 1];
in_a_bend_prev = false;
    
for i=1:round(length(time)/20):length(time)

    CF_T_cam = [quat2dcmElena(z_CF(1:4,i)'), z_CF(5:7,i); 0 0 0 1];
    world_T_cam_pp = world_T_pipe*pipe_T_CF*CF_T_cam;


    if sum(isnan(z_PE_t_CF(:,i)))==0
        in_a_bend=true;
        pipe_t_CF = z_PE_t_CF(:,i);
    else
        in_a_bend=false;            
    end
    pipe_T_CF = [quat2dcmElena(z_PE_R_CF(1:4,i)'), pipe_t_CF; 0 0 0 1];


    new_bend = in_a_bend & ~in_a_bend_prev;
    [world_t_pipe, world_R_pipe] = PipeEstimator_world_t_current...
        (CF_T_cam, world_T_cam_pp, pipe_T_CF, new_bend);

    if i==400
        disp('hey')
    end
    

    if sum(isnan(world_t_pipe))==0
        world_T_pipe(1:3,4) = world_t_pipe
    end    
    world_T_pipe(1:3,1:3) = quat2dcmElena(z_PE_pipe(:,i)');    

    world_T_CF = world_T_pipe*pipe_T_CF ;      
    world_T_cam = world_T_CF*CF_T_cam;        


    plt.plot_with_head(angles(:,i), world_T_cam*CF_t_hebi, []); hold on        
    drawframe_multiple_colors(world_T_cam, 0.1,1); hold on
    drawframe_multiple_colors(world_T_pipe, 0.1, 2); hold on  
    drawframe_multiple_colors(world_T_CF, 0.1); hold on     
    hold off

    in_a_bend_prev = in_a_bend;
    axis equal
    ylabel('world T cam NO FILTER')

end 