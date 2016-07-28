
function [pipe_T_CF, in_a_bend] = retrieve_pipe_T_CF(head_theta, bend_info)

theta = deg2rad(head_theta);
theta = theta +pi; % WARNING, for some reason head orientation is pointing iside

CF_R_pipe = nan(3,3,length(theta));
for i=1:length(theta)
    CF_R_pipe(:,:,i) = [cos(theta(i)), -sin(theta(i)), 0; ...
                sin(theta(i)), cos(theta(i)), 0; ...
                0, 0, 1];
end
        
CF_t_pipe = nan(3,length(head_theta));
in_a_bend = zeros(1,length(head_theta));
for b=1:length(bend_info)
    start_idx = bend_info(b).start_idx;
    end_idx = bend_info(b).end_idx;       
    CF_t_pipe(:,start_idx:end_idx) = [bend_info(b).bend_pos; ...
                        zeros(1,length(start_idx:end_idx))];
    in_a_bend(start_idx:end_idx) = 1;
end

CF_T_pipe = nan(4,4,length(head_theta));
CF_T_pipe(1:3,1:3,:) = CF_R_pipe;
CF_T_pipe(1:3,4,:) = CF_t_pipe;
CF_T_pipe(4,1:4,:) = repmat([0 0 0 1],1,1,length(head_theta));

pipe_T_CF = nan(size(CF_T_pipe));
for i=1:length(CF_T_pipe)
    if in_a_bend(i)
        pipe_T_CF(:,:,i) = inv(CF_T_pipe(:,:,i));
    else
        pipe_T_CF(1:3,1:3,i) = inv(CF_T_pipe(1:3,1:3,i));
        pipe_T_CF(1:3,4,i) = nan(3,1);
        pipe_T_CF(4,1:4,i) = [0 0 0 1];
    end
end

end