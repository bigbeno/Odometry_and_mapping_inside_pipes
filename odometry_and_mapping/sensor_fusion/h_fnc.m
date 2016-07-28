function z = h_fnc(x, in_bend, joint_visible, new_bend)
%Given the state, returns the predicted measurement

% The state is 28x1
% x = [w_T_cam, w_T_cam_p, w_T_pipe, pipe_T_CF]'
w_T_cam =  x(1:7); w_t_cam = w_T_cam(5:7);
w_T_camp =  x(8:14); w_t_camp = w_T_camp(5:7);
w_T_pipe =  x(15:21); w_R_pipe = w_T_pipe(1:4); w_t_pipe = w_T_pipe(5:7);
pipe_T_CF =  x(22:28); pipe_R_CF = pipe_T_CF(1:4); pipe_t_CF = pipe_T_CF(5:7); 

% A measurement z is either
% *   z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]'  21x1
%     if a joint is visible and the snake is through a bend
% *   z = [z_CF; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]; 18 x1
%     if no joint is visible and the snake is going through a bend
% *   z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF(2:3)];  18x1
%     if a joint is visible and the snake is not going through a bend
% *   z = [z_CF; z_PE_R_CF; z_PE_pipe; z_PE_t_CF(2:3)]; 15x1
%     if no joint is visible and the snake is not going through a bend
% *   z = [z; z_PE_t_pipe] n+3x1
%     if the snake just got into a bend

%z_CF = inv(pipe_T_CF)*inv(w_T_pipe)*w_T_cam;
z_CF =  multT(multT(invT(pipe_T_CF'), invT(w_T_pipe')),w_T_cam');

% z_VJT = - (inv(w_R_pipe)*w_t_cam - inv(w_R_pipe)*w_t_camp);
z_VJT = - (Rtimest(quatinvElena(w_R_pipe'), w_t_cam') - ...
                Rtimest(quatinvElena(w_R_pipe'), w_t_camp'));

% z_PE_R_CF = pipe_R_CF;
z_PE_R_CF = pipe_R_CF;

%z_PE_pipe = w_R_pipe;
z_PE_pipe = w_R_pipe;

%z_PE_t_CF = pipe_t_CF;
z_PE_t_CF = pipe_t_CF;

%z_PE_t_pipe = world_t_pipe
z_PE_t_pipe = w_t_pipe;

if in_bend && joint_visible
    z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF];
elseif in_bend && ~joint_visible
    z = [z_CF; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]; %NO VJT
elseif joint_visible && ~in_bend
    z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF(2:3)]; %NO PE_x_CF
else
    z = [z_CF; z_PE_R_CF; z_PE_pipe;  z_PE_t_CF(2:3)]; %NO VJT and NO PE_x_CF
end
if new_bend
    z = [z; z_PE_t_pipe];
end


end