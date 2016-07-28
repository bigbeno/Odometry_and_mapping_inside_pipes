%% odometry_and_mapping_in_pipes_main
% This transforms the previously computed estimates from the Complementary
% Filter, the Feature-Based Visual Odometry and the Visual Joint Tracker
% into measurements suitable for the Extended Kalman Filter.
% Then it computes the Pipe Estimator estimate.
% Then it runs the Extended Kalman Filter
% 
% 
% Notation:
% 'a_T_b' is the transformation that describes frame 'b' in frame 'a'
% It can be either expressed with:
% * an homogenous transformation matrix (4x4)
% * a quaternion (4x1) and a translation (3x1) (7x1)

close all
clear all
clc
HebiLookup
pause(1)

%% OPTIONS

% Specify files with the estimates
log_name = 'run_log_sample4.mat'; % Kinematics and Complementary Filter estimate
fbvo_log_name = 'appearance_odometry_sample4.csv'; % Feature-based Visual Odometry estimate
co_log_name = 'circle_odometry_sample4.csv'; % Visual Joint Tracker estimate

%If available, ground truth of the experiment
truth_name = 'truth_sample4.mat';

% Plot options
PLOT_DURING_EKF = false;

%% ESTIMATES TO MEASURMENTS
%
% Measurements available from the different estimates:
% z_CF_T: pose of the camera in the CF frame (7x1) 
%               [Coming from Complementarty Filter]
% z_VO_T: pose of the camera in the previous camera pose frame (7x1) 
%               [Coming from Feature-Based Visual Odometry]
% z_VJT_T: position of the pipe joint in the camera frame (3x1)
%               [Coming from Visual Joint Tracker]
%               [Available only if a joint is visible in current and previous frmae]
% z_PE_R_CF_T: orientation of the CF frame in the current pipe frame (4x1)
%               [Coming from the Pipe Estimator]
% z_PE_t_CF_T: position of the CF frame in the current pipe frame (3x1)
%               [Coming from the Pipe Estimator] 
%               [Avaialble only if snake is going though a bend]
%               [Otherwise it is assumed it lies on the pipe main axis (2x1)] 
% z_PE_pipe_T: orientation of the current pipe frame in the world frame (4x1)
%               [Coming from the Pipe Estimator] 
% z_PE_t_pipe: position of the current pipe frame in the world frame (3x1)
%               [Coming from the Pipe Estimator ON LINE] 
%               [Available only when the snake just moved to a new pipe]

% Transform estimates into EKF measurements (homogeneous
[z_CF_T, z_VO_T, z_VJT_T, z_PE_R_CF_T, z_PE_pipe_T, z_PE_t_CF_T,...
    time, angles] = estimates_to_measurements(log_name, fbvo_log_name, co_log_name);

% Transform homogeneous matrices [R, t; 0 0 0 1] (4x4)
% into (quaternion, translation) format (7x1)
z_CF = dcm2quatElena(z_CF_T(1:3,1:3,:));
z_CF = [z_CF'; reshape(z_CF_T(1:3,4,:),3,[])];
z_VO = dcm2quatElena(z_VO_T(1:3,1:3,:));
z_VO = [z_VO'; reshape(z_VO_T(1:3,4,:),3,[])];
z_VJT = z_VJT_T;
z_PE_R_CF = dcm2quatElena(z_PE_R_CF_T(1:3,1:3,:))';
z_PE_pipe = dcm2quatElena(z_PE_pipe_T(1:3,1:3,:))';
z_PE_t_CF = reshape(z_PE_t_CF_T,3,[]);

disp('End of computation of EKF measurements')
disp('Press any key to continue')
pause()

%% MODEL
%
% STATE 'x' (28x1) is  made of 4 pose estimates
% state(1:7) = camera pose wrt world;
% state(8:14) = camera pose wrt world in previous iteration;
% state(15:21) = current pipe pose wrt world;
% state(22:28) = CF pose wrt current pipe;
%
% MOTION MODEL 'g' is
% state(1:7, k) = state(1:7, k-1) '+' z_VO_T = we use Feature-Based Visual 
%                               Odometry measurement to predict camera motion
% state(8:14, k) = state(1:7, k-1) = current previous pose is previous
%                                   current pose
% state(15:21, k) = state(15:21, k-1) = zero-velocity model
% state(22:28, k) = state(22:28, k-1) = zero-velocity model
%
% MESUREMENT 'z' is made of the measurements form estimates
% At its longest it is (24x1): 
% z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF; z_PE_t_pipe];
% Depending on the current state of the robot, some measurements may not be
% available. 
% * If no pipe joint is visible in current an previous frame,  z_VJT (3x1)
%       is not  avaiable 
% * If the snake is not going trough a bend, z_PE_t_CF (3x1) is not avaialble,
%       but we approximate z_PE_[y z]_CF as [0 0] (2x1)
% * If the snake didn't just turn a bend, z_PE_t_pipe is not available (3x1)
%
% ERROR COVARIANCE MATRICES
% P: state covariance matrix (28x28)
% Q: motion model covariance matrix (28x28)
% R_full: measurement covariance matrix for the full measurement (24x24)
% cov_pipe_t_CF_assumption: measurement covariance matrix for z_PE_[y z]_CF (2x2)

% Computes inital state and covariance from measurements
CF_T_cam = z_CF(:,1);
pipe_R_CF = z_PE_R_CF(:,1);
[x0, P0] = define_inital_data(CF_T_cam, pipe_R_CF);

% Definition of covariances
[Q, R_full, cov_pipe_t_CF_assumption] = define_covariances();
% Linearization of motion model and measurement model
%define_model_jacobians(); % Uncomment this to re-write the jacobian files


%% EXTENDED KALMAN FILTER
close all
clear x x_p P F z

% Initalize state and covarianvce
x(:,1) = x0;
P(:,:,1) = P0;


for i=2:length(time)
    
    disp([num2str(mod(i/length(time),10)*100),'%']);
    
    %%%%%%%%%%%%%%% Prediction step %%%%%%%%%%%%%%%%%%%%%%%%
    
    % Predict state
    x_p(:,i) = g_fnc(x(:,i-1), z_VO(:,i));   
    
    %Make sure quaternions stay unitary
    x_p(:,i)  = normalize_state_q(x_p(:,i) );   
    
    % Predict covariance
    input = num2cell([x(:,i-1); z_VO(:,i)] );
    F(:,:,i-1) = dgdx_fnc(input{:});
    P_p(:,:,i) = F(:,:,i-1) * P(:,:,i-1) * F(:,:,i-1)' + Q;
    
    
    
    %%%%%%%%%%%%%%% Measurement update step %%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%% Build measurement vector
    R = R_full;
    
    % Is Visual-Joint tracker measurement avaialble?
    % [ie. Was a joint tracked both in current a previous frame?]
    if sum(isnan(z_VJT(:,i)))==0
        joint_visible = true;
        z = [z_CF(:,i); z_VJT(:,i); z_PE_R_CF(:,i); z_PE_pipe(:,i)];
    else
        joint_visible = false;
        z = [z_CF(:,i); z_PE_R_CF(:,i); z_PE_pipe(:,i)];
        R(8:10,:) = []; R(:,8:10) = []; % Prune measurement matrix
    end
    
    % Is Pipe Estimator measurement of the position of CF avaialble?
    % [i.e. Is snake goign though a bend?]
    if sum(isnan(z_PE_t_CF(:,i)))==0
        in_bend = true;
        z = [z; z_PE_t_CF(:,i)];
    else
        % If not, use assumption that CF origin lies on pipe main axis
        z = [z; [0 0]'];
        R(end-5,:) = []; R(:,end-5) = [];
        R(end-4:end-3,end-4:end-3) = cov_pipe_t_CF_assumption;
        in_bend = false;
    end  
    
    % Is Pipe Estimator measurement of the position of pipe avaialble?
    % [i.e. Did snake just locomte to a new pipe?]
%     if in_bend && sum(isnan(z_PE_t_CF(:,i-2)))==3 && sum(isnan(z_PE_t_CF(:,i-1)))==0
    if in_bend && sum(isnan(z_PE_t_CF(:,i-1)))==3
        new_pipe = true;
        disp('new pipe!')
%         pause()
%         z_PE_t_pipe = PipeEstimator_world_t_pipe_online...
%             (z_CF(:,i), x(1:7,i-4), x(22:28, i-1));
        z_PE_t_pipe = PipeEstimator_world_t_pipe_online...
            (z_CF(:,i), x(1:7,i-1), [z_PE_R_CF(:,i); z_PE_t_CF(:,i)]);

        z = [z; z_PE_t_pipe];
    else
        new_pipe = false;
        R(end-2:end,:) = []; R(:,end-2:end) = [];  % Prune measurement matrix
    end   
    
    %%%% Predicted measurment
    z_pred = h_fnc(x_p(:,i), in_bend, joint_visible, new_pipe);
    
    %%%% Residual
    % NB: I am subtracting quaternions, this is not the most hortodox
    % choice but TUM does it (http://campar.in.tum.de/Chair/KalmanFilter)
    % and it so much simpler than indirect EKF
    v = z - z_pred;

    %%% Measurement matrix
    H = dhdx_fnc(x_p(:,i), in_bend, joint_visible, new_pipe);
    %%% Innovation matrix
    S = H * P_p(:,:,i) * H' + R;
    %%% Gain matrix
    K= P_p(:,:,i)*H'*inv(S);

    %%% Update the state
    x(:,i) =  x_p(:,i) + K*v;
    
    %%% Make sure quaternions stay unitary
    x(:,i)  = normalize_state_q(x(:,i) );
    
    %%% Update the covariance
    P(:,:,i) = (eye(size(P0))- K*H)*P_p(:,:,i)*(eye(size(P0))- K*H)' +K*R*K';
   
    
    %%%%%%%%%%%% PLOT THINGS %%%%%%%%%%%%%%%%%%%%
    if PLOT_DURING_EKF
    plot_data_during_EKF;
    pause(0.001)    
    end
    
end



disp('END')

% Plotting

plot_data_after_EKF

plot_EKF_vs_truth

return




