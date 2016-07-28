function [Q, R, cov_pipe_t_CF_assumption] = define_covariances()


%%% SYSTEM DYNAMICS COVARIANCE
% z_VO covariance
cov_w_q_cam = ones(4,1)*1e-1;
cov_w_t_cam = ones(3,1)*1e-3;
% Others
cov_w_T_camp =  ones(7,1)* eps; % We are very sure that previous current state is current previous state!
cov_w_q_pipe =  ones(4,1)* 1e-8;
cov_w_t_pipe =  ones(3,1)* 1e-8;
cov_pipe_q_CF = ones(4,1)* 1;
cov_pipe_t_CF = ones(3,1)* 1;
Q = diag([cov_w_q_cam; cov_w_t_cam; cov_w_T_camp; cov_w_q_pipe; cov_w_t_pipe; cov_pipe_q_CF; cov_pipe_t_CF]);


%%% MEASURMENT COVARIANCE
%z_CF
cov_CF_q_cam = ones(4,1)*1e-3;
cov_CF_t_cam = ones(3,1)*1e-3;
%z_VJT
cov_d_pipe_t_cam = [1e-3;   1e-1;   1e-1];
%z_PE_R_CF
cov_pipe_q_CF = ones(4,1)*1e-10;
%z_PE_PIPE
cov_w_q_pipe = ones(4,1)*1e-10;
%z_PE_t_CF
cov_pipe_t_CF= ones(3,1)*1e-10;
%z_PE_[y z]_CF_assumption
cov_pipe_t_CF_assumption= diag([1e-3, 1e-3]); 
%z_PE_t_CF
cov_w_t_pipe= ones(3,1)*1e-20;
R = diag([cov_CF_q_cam; cov_CF_t_cam; cov_d_pipe_t_cam; cov_pipe_q_CF; cov_w_q_pipe; cov_pipe_t_CF; cov_w_t_pipe]);


R = R.^2;
Q = Q.^2;
cov_pipe_t_CF_assumption =  cov_pipe_t_CF_assumption.^2;

end