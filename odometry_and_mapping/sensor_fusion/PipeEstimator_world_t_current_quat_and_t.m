function world_t_pipe = PipeEstimator_world_t_current_quat_and_t...
    (CF_T_cam, world_T_cam, pipe_T_CF, in_a_new_bend)

%world_T_pipe
world_t_pipe = nan(4,1);
if in_a_new_bend
    CF_T_pipe = inv(pipe_T_CF);
    world_T_pipe = world_T_cam * inv(CF_T_cam) * CF_T_pipe;
    world_t_pipe = world_T_pipe(1:3,4);  
end

end