function x_kp1 = g_fnc(x_k, z_VO)
% Given the current state, returns the next a-priori state

% The state is 28x1
% x = [w_T_cam, w_T_cam_p, w_T_pipe, pipe_T_CF]'
% with each foo1_T_foo2 being (q,t) corresponding to the homogeneous
% transformation that described the pose of foo2 in the reference frame of
% foo1
w_T_cam_k =  x_k(1:7);
w_T_cam_km1 =  x_k(8:14);
w_T_pipe_k =  x_k(15:21);
pipe_T_CF =  x_k(22:28);

% The next state is:
% Driven by VO
% z_VO = cam_km1_T_cam_k
cam_k_T_cam_kp1 = z_VO;
w_T_cam_kp1 =  multT(w_T_cam_k,cam_k_T_cam_kp1);
% Constant velocity model
w_T_cam_k =  w_T_cam_k;
w_T_pipe_kp1 =  w_T_pipe_k;
pipe_T_CF_kp1 =  pipe_T_CF;

x_kp1 = [w_T_cam_kp1; w_T_cam_k; w_T_pipe_kp1; pipe_T_CF_kp1];

end