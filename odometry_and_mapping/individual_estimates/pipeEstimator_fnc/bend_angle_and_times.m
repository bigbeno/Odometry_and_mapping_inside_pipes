function bend_info = bend_angle_and_times(d_theta, time)

dtheta_more_than_thresh = abs(d_theta)>5;

diff_tf = diff(dtheta_more_than_thresh);

start_bend_idx = find(diff_tf==1);
end_bend_idx = find(diff_tf==-1);
if start_bend_idx(1)>end_bend_idx(1)
    end_bend_idx(1) = [];
end

if length(end_bend_idx)<length(start_bend_idx)
    end_bend_idx = [end_bend_idx length(d_theta)];
end
assert(length(end_bend_idx)==length(start_bend_idx))


angle_bend= zeros(1, length(d_theta));

for i=1:length(start_bend_idx)
    bend_info(i).start_idx = start_bend_idx(i);
    bend_info(i).end_idx = end_bend_idx(i);
    
    
    [~, max_diff] = max(abs(d_theta(start_bend_idx(i):end_bend_idx(i))));
    theta_angle = d_theta(max_diff + start_bend_idx(i));
    bend_info(i).angle = theta_angle;
    
    
    angle_bend(start_bend_idx(i):end_bend_idx(i)) = theta_angle;
    
end

figure
plot(time, d_theta); hold on
plot(time, angle_bend);
legend('Relative orientation between head and tail', 'Postprocessed bend angle');
ylabel('Angle [deg]');
xlabel('Time [sec]');
title('Estimated pipe angle');


end