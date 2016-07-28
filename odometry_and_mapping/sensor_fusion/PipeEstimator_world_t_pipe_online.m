function world_t_pipe= PipeEstimator_world_t_pipe_online(CF_T_cam, world_T_cam, pipe_T_CF)

% world_t_pipe = world_T_cam * inv(CF_T_cam) * CF_t_pipe;

% world_T_pipe = world_T_cam * inv(CF_T_cam) * inv(pipe_T_CF);
% world_t_pipe = world_T_pipe(1:3,4);
world_T_pipe = multT(multT(world_T_cam, invT(CF_T_cam')),invT(pipe_T_CF')');
world_t_pipe = world_T_pipe(5:7);

end