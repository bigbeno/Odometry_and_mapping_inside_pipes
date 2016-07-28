function [z_CF, z_VO, z_VJT, z_PE_R_CF, z_PE_pipe, z_PE_t_CF,...
    time, angles] = estimates_to_measurements(log_name, fbvo_log_name, vjt_log_name)
%% Estimates to measurements
% This transforms the previously computed estimates from the Complementary
% Filter, the Feature-Based Visual Odometry and the Visual Joint Tracker
% into measurements suitable for the Extended Kalman Filter

%% Load run log

load(log_name);    
run_log = run_log(1:end-1);

%% Kineamtics 
[angles, ~] = Kinematics(run_log);

%% Complementary Filter Block
[CF_T_cam, time_CF, misc_CF] = ComplementaryFilterBlk(run_log);
time_CF = time_CF-2;

%% Feature Based Visual Odometry
[camprev_T_cam, time_VO] = FeatureBasedVisualOdometry(fbvo_log_name);

%% Visual Joint Tracker
[cam_t_joint, ~, visible_joint] = VisualJointTracker(vjt_log_name);

%% Pipe Estimator
[pipe_T_CF, world_R_pipe, ~] = PipeEstimator(CF_T_cam, misc_CF, angles, time_CF);



%% MEASURMENTS
% Interpolate at time_VO
CF_T_cam = interpolate_T(CF_T_cam, time_CF, time_VO);
pipe_T_CF = interpolate_T(pipe_T_CF, time_CF, time_VO);
world_R_pipe = interpolate_R(world_R_pipe, time_CF, time_VO);
in_a_bend = reshape(~isnan(pipe_T_CF(1,4,:)),1,[]);
angles = interpolate_scalars(angles, time_CF, time_VO);

%%% Assign measurements as functions of estimates

% z_CF = CF_T_cam
z_CF = CF_T_cam;

% z_VO = cam'_R_t*lamba_cam
lambda = lambda_f(z_CF, camprev_T_cam, time_VO);
z_VO = camprev_T_cam;
for i=1:length(z_VO)
    z_VO(1:3,4,i) = z_VO(1:3,4,i)*lambda(i);
end
%Removing spikes (hampel would be better)
z_VO = medfilt1(z_VO,3,[],3);


% z_VJT = pipe_t_joint(i) - pipe_t_joint(i-1) (when visible-joint in i and i-1)
z_VJT = nan(3, length(cam_t_joint));
z_VJT_avaialble = zeros(1,length(cam_t_joint));
for i=1:length(z_VJT)
    if visible_joint(i) && visible_joint(i-1)
        z_VJT(:,i) = pipe_T_CF(1:3,1:3,i)*CF_T_cam(1:3,1:3,i)*cam_t_joint(:,i) - ...
                    pipe_T_CF(1:3,1:3,i-1)*CF_T_cam(1:3,1:3,i-1)*cam_t_joint(:,i-1);
        z_VJT_avaialble(i) = 1;   
    end
end

% z_PE = pipe_R_CF, world_R_pipe, pipe_t_CF (when in_a_bend)
z_PE_R_CF = pipe_T_CF(1:3,1:3,:);
z_PE_pipe = world_R_pipe;
z_PE_t_CF = pipe_T_CF(1:3,4,:);


time = time_VO;

end



























