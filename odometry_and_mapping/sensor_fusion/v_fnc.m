function v = v_fnc(z, zpred, in_bend, joint_visible)
% Given two measurements computed the error between them

%%%% ISOLATE MEASUREMNTS
% A measurement z is either
% *   z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]'  21x1
%     if a joint is visible and the snake is through a bend
% *   z = [z_CF; z_PE_R_CF; z_PE_pipe; z_PE_t_CF]; 18 x1
%     if no joint is visible and the snake is going through a bend
% *   z = [z_CF; z_VJT; z_PE_R_CF; z_PE_pipe];  18x1
%     if a joint is visible and the snake is not going through a bend
% *   z = [z_CF; z_PE_R_CF; z_PE_pipe]; 15x1
%     if no joint is visible and the snake is not going through a bend

z_CF = z(1:7);
z_CF_pred = zpred(1:7);
if joint_visible
    z_VJT =z(8:10);
    z_VJT_pred =zpred(8:10); 
    z_PE_R_CF =  z(11:14);
    z_PE_pipe =  z(15:18);    
    z_PE_R_CF_pred =  zpred(11:14);
    z_PE_pipe_pred =  zpred(15:18);      
else
    z_PE_R_CF =  z(8:11);
    z_PE_pipe =  z(12:15);    
    z_PE_R_CF_pred =  zpred(8:11);
    z_PE_pipe_pred =  zpred(12:15);    
end
if in_bend
    z_PE_t_CF =  z(end-2:end);
    z_PE_t_CF_pred =  zpred(end-2:end);
end


%%%% COMPUTE MEASURMENT ERRROR

% q_error = q_measured x inv(q_predicted)
% v_CF_q = quat_multElena(z_CF(1:4)', quatinvElena(z_CF_pred(1:4)'));
% % v_CF_q = [1 0 0 0]' - v_CF_q;
% v_CF_q = [0; - v_CF_q(2:4)];
v_CF_q = z_CF(1:4) - z_CF_pred(1:4);
% dt_error= dt_measured - dt_predicted
v_CF_t = z_CF(5:7)-z_CF_pred(5:7);

% v_PE_R_CF= quat_multElena(z_PE_R_CF', quatinvElena(z_PE_R_CF_pred'));
v_PE_R_CF= z_PE_R_CF- z_PE_R_CF_pred;
% v_PE_R_CF = [1 0 0 0]' - v_PE_R_CF;
% v_PE_R_CF = [0; - v_PE_R_CF(2:4)];

% v_PE_pipe= quat_multElena(z_PE_pipe', quatinvElena(z_PE_pipe_pred'));  
v_PE_pipe= z_PE_pipe- z_PE_pipe_pred;
% v_PE_pipe = [1 0 0 0]' - v_PE_pipe;
% v_PE_pipe = [0; - v_PE_pipe(2:4)];

if joint_visible
    v_VJT = z_VJT-z_VJT_pred;
end

if in_bend
    v_PE_t_CF = z_PE_t_CF-z_PE_t_CF_pred;
end

%%%% ASSEMBLE ERROR
% The error v is either
% *   v = [v_CF; v_VJT; v_PE_R_CF; v_PE_pipe; v_PE_t_CF]' 
%     if a joint is visible and the snake is through a bend
% *   v = [v_CF; v_PE_R_CF; v_PE_pipe; v_PE_t_CF]; 
%     if no joint is visible and the snake is going through a bend
% *   v = [v_CF; v_VJT; v_PE_R_CF; v_PE_pipe]; 
%     if a joint is visible and the snake is not going through a bend
% *   v = [v_CF; v_PE_R_CF; v_PE_pipe]; 
%     if no joint is visible and the snake is not going through a bend

v = [v_CF_q; v_CF_t];

if joint_visible
    v = [v; v_VJT; v_PE_R_CF; v_PE_pipe];
else
    v = [v; v_PE_R_CF; v_PE_pipe];
end

if in_bend
    v = [v; v_PE_t_CF];
end


end