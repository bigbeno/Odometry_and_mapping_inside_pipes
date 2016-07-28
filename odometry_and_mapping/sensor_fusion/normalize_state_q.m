function x_norm = normalize_state_q(x)
%Given a state, normalize all the quaternions inside it

% The state is 28x1
% x = [w_T_cam, w_T_cam_p, w_T_pipe, pipe_T_CF]'
% with foo1_T_foo2 being foo2 in frame foo1 expressend as (q,t) 7x1
% with q quaternion  and t translation vector
w_T_cam =  x(1:7); w_R_cam = w_T_cam(1:4);
w_T_camp =  x(8:14); w_R_camp = w_T_camp(1:4);
w_T_pipe =  x(15:21); w_R_pipe = w_T_pipe(1:4);
pipe_T_CF =  x(22:28);  pipe_R_CF = pipe_T_CF(1:4);

%Normalize all foo1_q_foo2
w_R_cam = w_R_cam/norm(w_R_cam);
w_R_camp = w_R_camp/norm(w_R_camp);
w_R_pipe = w_R_pipe/norm(w_R_pipe);
pipe_R_CF = pipe_R_CF/norm(pipe_R_CF);

%Return state with normalized quaternions
w_T_cam(1:4) = w_R_cam;
w_T_camp(1:4) = w_R_camp;
w_T_pipe(1:4) = w_R_pipe;
pipe_T_CF(1:4) = pipe_R_CF;
x_norm = [w_T_cam; w_T_camp; w_T_pipe; pipe_T_CF];


end