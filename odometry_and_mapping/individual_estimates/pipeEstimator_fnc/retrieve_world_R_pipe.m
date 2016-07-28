function world_R_pipe = retrieve_world_R_pipe(bend_info, time)

world_R_pipe = nan(3,3, length(time));
world_R_pipe(:,:,1) = eye(3);
for i=2:length(time)
    world_R_pipe(:,:,i) =  world_R_pipe(:,:,i-1);
    for b=1:length(bend_info)
        if i==bend_info(b).start_idx
            theta = deg2rad(bend_info(b).angle);
            world_R_pipe(:,:,i) = world_R_pipe(:,:,i-1)*...
                                    [cos(theta), -sin(theta), 0; ...
                                    sin(theta), cos(theta), 0; ...
                                    0, 0, 1];
        end
    end
end

end