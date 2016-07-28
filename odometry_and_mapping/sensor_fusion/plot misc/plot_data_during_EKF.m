%% Plot a buch of information during EKF

% How often do you wanna plot things?
skip = 50;

% Which information do you wanna plot
DISP_VALUES = false;
PLOT_GRAPHS = true;
PLOT_SNAKE = false;
PLOT_COV =false;


%% Preliminary things

if DISP_VALUES
   meas_names = {'|', '|', '| CF_q_cam', '|_', ...
       '|','| CF_t_cam', '|_', ...
    '|','| - delta(pipe_t_cam)', '|_', ...
    '|', '|', '| pipe_R_CF', '|_',...
    '|', '|', '| w_R_pipe', '|_',...      
    '|','| pipe_t_CF', '|', ...
    '|','| world_t_pipe', '|'}';
   state_names = {'|', '|', '| w_q_cam', '|_', '|','| w_t_cam', '|_', ...
   '|', '|', '| w_q_camp', '|_', '|','| w_t_camp', '|_', ...
      '|', '|', '| w_q_pipe', '|_', '|','| w_t_pipe', '|_', ...
   '|', '|', '| pipe_q_CF', '|_', '|','| pipe_t_CF', '|'}';
end
if PLOT_GRAPHS && ~exist('f1_h', 'var')
   f1_h = figure;
   f2_h = figure;
end
if PLOT_COV && ~exist('f3_h', 'var')
   f3_h = figure;
end
if PLOT_GRAPHS || PLOT_SNAKE
    [w_T_cam_T, w_T_camp_T, w_T_pipe_T, pipe_T_CF_T] = transform_xqt_in_xT(x(:,i));    
end
if PLOT_SNAKE && ~exist('plt', 'var')
    plt = HebiPlotter('frame', 'head');
end


%% DIPLAY STATE AND MEASUREMENT
% On Command Window

if DISP_VALUES
   % Display Measurements
%    measurements = [z, z_pred, v];
%    meas_names_now = meas_names;   
%    if  ~joint_visible
%         meas_names_now(8:10) = [];       
%    end
%    if ~in_bend
%        meas_names_now(end-5) = [];
%    end
% 
%     display(sprintf('real \t pred\t error\t MEASUREMENT\t')); 
%     for j=1:length(z)
%         display(sprintf('%.3f\t%.3f\t%.3f \t%s', ...
%             measurements(j,1), measurements(j,2), measurements(j,3), meas_names_now{j})); 
%     end    
%      disp('---------------------------------------')  

    % Display STATE
    states = [x(:,i-1), x_p(:,i), x(:,i)];
    display(sprintf('prev \t pred\t current\t STATE\t')); 
    for j=1:28
        display(sprintf('%.3f\t%.3f\t%.3f\t%s', ...
            states(j,1), states(j,2), states(j,3), state_names{j})); 
    end   
    disp(' ')
end

%% PLOT COVARIANCE MATRICES

if PLOT_COV
    if mod(i,skip)==1
        set(0,'CurrentFigure',f3_h)

        subplot(2,3,1)
        imagesc(P(:,:,i))
        axis equal
        title(['P (',num2str(time(i)),')']);
        colorbar        
        subplot(2,3,2)
        imagesc(F(:,:,i-1));
        axis equal
        title('F');
        colorbar        
        subplot(2,3,3)
        imagesc(Q);
        axis equal
        title('Q');
        colorbar        
        subplot(2,3,4)
        imagesc(R);
        title('R'); 
        axis equal
        colorbar
        subplot(2,3,5)
        imagesc(H);
        title('H'); 
        axis equal
        colorbar
        subplot(2,3,6)
        imagesc(S);
        title('S'); 
        axis equal
        colorbar
    end
end


%% PLOT TRAJECTORY GRAPHS

if PLOT_GRAPHS
    if mod(i,skip)
    [w_T_cam_Tpred, w_T_camp_Tpred, w_T_pipe_Tpred, pipe_T_CF_Tpred] = transform_xqt_in_xT(x_p(:,i));    
    [CF_T_cam, pipe_R_CF, w_R_pipe, pipe_t_CF]  = transform_zqt_in_zT(z, in_bend);
    if(sum(isnan(pipe_t_CF))==0)
        pipe_T_CF_T_meas = [pipe_R_CF, pipe_t_CF; 0 0 0 1];
    else
        pipe_T_CF_T_meas = [pipe_R_CF, pipe_T_CF_Tpred(1:3,4); 0 0 0 1];
    end
    w_T_CF_T_meas = w_T_pipe_Tpred*pipe_T_CF_T_meas;
    w_T_pipe_T_meas = [w_R_pipe, w_T_pipe_Tpred(1:3,4); 0 0 0 1];
    w_T_cam_T_meas = w_T_pipe_Tpred*pipe_T_CF_Tpred*CF_T_cam;    

    [pitch, roll, yaw] = dcm2angleElena( w_T_cam_T(1:3,1:3,:), 'YXZ' );
    [pitchp, rollp, yawp] = dcm2angleElena( w_T_camp_T(1:3,1:3,:), 'YXZ' );
    w_t_cam = reshape(w_T_cam_T(1:3,4,:),3,[]);
    w_t_camp = reshape(w_T_camp_T(1:3,4,:),3,[]);
    [pitch_meas, roll_meas, yaw_meas] = dcm2angleElena( w_T_cam_T_meas(1:3,1:3,:), 'YXZ' );
    w_t_cam_meas = reshape(w_T_cam_T_meas(1:3,4,:),3,[]);  
    [pitch_pred, roll_pred, yaw_pred] = dcm2angleElena( w_T_cam_Tpred(1:3,1:3,:), 'YXZ' );
    w_t_cam_pred = reshape(w_T_cam_Tpred(1:3,4,:),3,[]);     

    set(0,'CurrentFigure',f1_h)
    subplot(4,2,[1 3])
    plot(time(i-1:i),[w_t_camp, w_t_cam],'.-'); hold on
    plot(time(i), w_t_cam_meas,'.-k'); hold on    
    plot(time(i), w_t_cam_pred,'o-k'); hold on    
    title('w t cam')
    subplot(4,2,[5 7])
    plot(time(i-1:i),[ pitchp, rollp, yawp; pitch, roll, yaw] ,'.-'); hold on
    plot(time(i), [pitch_meas , roll_meas , yaw_meas ],'.-k'); hold on 
    plot(time(i), [pitch_pred , roll_pred , yaw_pred ],'o-k'); hold on   
    title('w R cam')

    w_t_pipe = reshape(w_T_pipe_T(1:3,4,:),3,[]); 
    [pitch_pipe, roll_pipe, yaw_pipe] = dcm2angleElena( w_T_pipe_T(1:3,1:3,:), 'YXZ' );    
    subplot(4,2,2)
    plot(time(i),w_t_pipe,'*-'); hold on
    title('w t pipe')
    subplot(4,2,4)
    plot(time(i), [pitch_pipe, roll_pipe, yaw_pipe] ,'*-'); hold on
    title('w R pipe')

    pipe_t_CF = reshape(pipe_T_CF_T(1:3,4,:),3,[]); 
    [pitch_CF, roll_CF, yaw_CF] = dcm2angleElena( pipe_T_CF_T(1:3,1:3,:), 'YXZ' );    
    subplot(4,2,6)
    plot(time(i),pipe_t_CF,'*-'); hold on
    title('pipe t CF')
    subplot(4,2,8)
    plot(time(i), [pitch_CF, roll, yaw_CF] ,'*-'); hold on
    title('pipe R CF')   


    set(0,'CurrentFigure',f2_h)  
    subplot(2,2,1)
    drawframe_multiple_colors(w_T_cam_Tpred, 0.2, 2); hold on;
    drawframe_multiple_colors(w_T_cam_T_meas, 0.15, 1); hold on     
    drawframe_multiple_colors(w_T_cam_T, 0.1); hold on 
    axis equal
    title('w T cam (CMK: predicted, RGB: measured, ')

    subplot(2,2,2)    
    drawframe_multiple_colors(w_T_cam_Tpred*pipe_T_CF_Tpred,0.2, 2);    hold on
    drawframe_multiple_colors(w_T_CF_T_meas, 0.15, 1); 
    drawframe_multiple_colors(w_T_pipe_T*pipe_T_CF_T,0.1);
    axis equal
    title('w T CF')    

    subplot(2,2,3)    
    drawframe_multiple_colors(w_T_pipe_Tpred,0.2, 2);hold on
    drawframe_multiple_colors(w_T_pipe_T_meas, 0.15, 1);          
    drawframe_multiple_colors(w_T_pipe_T,0.1);    
    title('w T pipe')    
    axis equal
    end

end

%% PLOT SNAKE POSE

if PLOT_SNAKE     
    plt.plot_with_head(angles(:,i), w_T_cam_T*CF_t_hebi, []);
end