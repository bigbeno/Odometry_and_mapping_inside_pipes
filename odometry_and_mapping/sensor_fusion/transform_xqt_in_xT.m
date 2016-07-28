function [w_T_cam_T, w_T_camp_T, w_T_pipe_T, pipe_T_CF_T] = transform_xqt_in_xT(x)
%Transform the state from the notation (q,t) to the notation T
% with q quaternion, t translation vector and T homogeneous transformation
% T = [R, t; 0 0 0 1];

w_T_cam =  x(1:7,:);
w_T_camp =  x(8:14,:); 
w_T_pipe =  x(15:21,:);
pipe_T_CF =  x(22:28,:);

for i=1:size(x,2)
    w_T_cam_T(:,:,i) = [quat2dcmElena(w_T_cam(1:4,i)'), w_T_cam(5:7,i); 0 0 0 1];
    w_T_camp_T(:,:,i) = [quat2dcmElena(w_T_camp(1:4,i)'), w_T_camp(5:7,i); 0 0 0 1];
    w_T_pipe_T(:,:,i) = [quat2dcmElena(w_T_pipe(1:4,i)'), w_T_pipe(5:7,i); 0 0 0 1];
    pipe_T_CF_T(:,:,i) = [quat2dcmElena(pipe_T_CF(1:4,i)'), pipe_T_CF(5:7,i); 0 0 0 1];
end


end
