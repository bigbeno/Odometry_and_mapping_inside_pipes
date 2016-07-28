function [CF_T_cam, pipe_R_CF, w_R_pipe, pipe_t_CF] = transform_zqt_in_zT(z, in_bend)
%Transform the state from the notation (q,t) to the notation T
% with q quaternion, t translation vector and T homogeneous transformation
% T = [R, t; 0 0 0 1];

% A measurement z is either
% *   z = [z_CF; z_VO; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]' 
%     if a joint is visible and the snake is through a bend
% *   z = [z_CF; z_VO; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]; 
%     if no joint is visible and the snake is going through a bend
% *   z = [z_CF; z_VO; z_VJT; z_PE_R_CF; z_PE_pipe]; 
%     if a joint is visible and the snake is not going through a bend
% *   z = [z_CF; z_VO; z_PE_R_CF; z_PE_pipe]; 
%     if no joint is visible and the snake is not going through a bend

z_CF = z(1:7); 
z_PE_R_CF = z(8:11);
z_PE_pipe = z(12:15);

pipe_t_CF = nan(3,1);
for i=1:size(z,2)
    CF_T_cam(:,:,i) = [quat2dcmElena(z_CF(1:4,i)'), z_CF(5:7,i); 0 0 0 1];
    pipe_R_CF(:,:,i) = quat2dcmElena(z_PE_R_CF(1:4,i)');
    w_R_pipe(:,:,i) = quat2dcmElena(z_PE_pipe(1:4,i)');
end

if in_bend
    z_PE_pipe = z(12:15);
    pipe_t_CF = z_PE_pipe;
end


end
