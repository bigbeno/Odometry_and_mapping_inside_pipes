function [head_theta_new, tail_theta_new] = theta_head_tail_impose_bend_angle(bend_info, head_theta, tail_theta)

PLOT = false;
if PLOT
    figure
end
tail_theta_new = tail_theta;
head_theta_new = head_theta;
for i=1:length(bend_info)
    
    start_idx = bend_info(i).start_idx;
    end_idx = bend_info(i).end_idx;
    angle = bend_info(i).angle;
    
    head_theta_bend = head_theta(start_idx:end_idx);
    tail_theta_bend = tail_theta(start_idx:end_idx);
    tail_theta_bend_constrained = head_theta_bend - angle;
    
    [~, min_idx] = min(abs(tail_theta_bend-tail_theta_bend_constrained));
    min_idx = min_idx+start_idx;
    if PLOT
        scatter(min_idx, tail_theta(min_idx)); hold on
    end
    
    tail_theta_new(start_idx:min_idx) = tail_theta(start_idx:min_idx);
    tail_theta_new(min_idx:end_idx) = head_theta(min_idx:end_idx) - angle;
    head_theta_new(start_idx:min_idx) = tail_theta(start_idx:min_idx) + angle;
    head_theta_new(min_idx:end_idx) = head_theta(min_idx:end_idx);
    
end

if PLOT
    plot(head_theta); hold on
    plot(tail_theta); hold on
    plot(head_theta_new); hold on
    plot(tail_theta_new); hold on
    legend('','head','tail','head new', 'tail new')
end

end