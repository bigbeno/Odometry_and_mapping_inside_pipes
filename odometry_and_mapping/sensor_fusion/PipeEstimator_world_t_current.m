function [world_t_pipe, world_R_pipe] = PipeEstimator_world_t_current(CF_T_cam, world_T_cam, pipe_T_CF, in_a_bend)

%world_T_pipe
world_t_pipe = nan(4,1);
world_R_pipe = nan(3,3,1);
if in_a_bend
    CF_T_pipe = inv(pipe_T_CF);
    world_T_pipe = world_T_cam * inv(CF_T_cam) * CF_T_pipe;
    world_t_pipe = world_T_pipe(1:3,4);
    world_R_pipe = world_T_pipe(1:3,1:3);
    
end
world_t_pipe = world_t_pipe(1:3);

end