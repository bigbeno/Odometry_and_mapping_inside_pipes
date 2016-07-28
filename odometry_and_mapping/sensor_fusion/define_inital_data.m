function [x0, P0] = define_inital_data(CF_T_cam, pipe_R_CF)

%%% INITIAL STATE
% x = w_T_cam, w_T_cam_p, w_T_pipe, pipe_T_CF
% Initial pipe is supposed to be aligned with world
pipe_T_CF = [pipe_R_CF', 0 0 0] ;
w_T_cam = multT(pipe_T_CF',CF_T_cam);
w_T_cam(5) = 0; % removing head offset since CoM is in (0,0,0)
w_T_cam_p = w_T_cam;
w_T_pipe = [1 0 0 0 0 0 0];
x0 = [w_T_cam', w_T_cam_p', w_T_pipe, pipe_T_CF]';

%%% INITIAL STATE COVARIANCE
P0 = [eye(28)]* eps;
P0 = P0.^2;

end