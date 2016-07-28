function bend_info = bend_position(module_positions, time, bend_info_in, head_theta, tail_theta, head_hebi, angles)
    
PLOT_trajectory = false;
PLOT = false;

bend_info = bend_info_in;

head_position =reshape(module_positions(1:2,1,:),2,[]);
tail_position =reshape(module_positions(1:2,end,:),2,[]);    
time_range = 30;
head_pos_f(1,:) = filter_no_delay_extended(time, head_position(1,:),time_range);
head_pos_f(2,:) = filter_no_delay_extended(time, head_position(2,:),time_range);
tail_pos_f(1,:) = filter_no_delay_extended(time, tail_position(1,:),time_range);
tail_pos_f(2,:) = filter_no_delay_extended(time, tail_position(2,:),time_range);   

if PLOT
    figure
    subplot(2,1,1)
    plot(time, head_position(1,:)); hold on
    plot(time, tail_position(1,:)); hold on
    plot(time, head_pos_f(1,:)); hold on
    plot(time, tail_pos_f(1,:)); hold on
    title('Position X')
    legend('head', 'tail', 'head filt', 'tail filt');    
    subplot(2,1,2)    
    plot(time, head_position(2,:)); hold on
    plot(time, tail_position(2,:)); hold on
    plot(time, head_pos_f(2,:)); hold on
    plot(time, tail_pos_f(2,:)); hold on
    title('Position Y')
    legend('head', 'tail', 'head filt', 'tail filt');
end

if PLOT_trajectory
    plt = HebiPlotter('frame','head');
    plt.plot_with_head(angles(:,1), head_hebi(:,:,1));
    hold on
    pause(0.5)
end

for b=1:length(bend_info)

    start_idx = bend_info(b).start_idx;
    end_idx = bend_info(b).end_idx;        

    head_theta_bend = head_theta(start_idx:end_idx);
    tail_theta_bend = tail_theta(start_idx:end_idx);
    head_position_bend = head_pos_f(:,start_idx:end_idx);
    tail_position_bend = tail_pos_f(:,start_idx:end_idx);

    bend_perc = nan(1, length(head_theta_bend));
    bend_pos = nan(2,length(head_theta_bend));
    for i=1:length(head_theta_bend)

        mh = tan(deg2rad(head_theta_bend(i)));
        qh = head_position_bend(2,i) - mh*head_position_bend(1,i);

        mt = tan(deg2rad(tail_theta_bend(i)));
        qt = tail_position_bend(2,i) - mt*tail_position_bend(1,i);

        bend_pos(1,i) = (qt - qh)/(mh - mt);
        bend_pos(2,i) = (mh*qt - mt*qh)/(mh - mt);

        segment_tail_head = head_position_bend(1:2,i)-tail_position_bend(1:2,i);
        segment_tail_bend = bend_pos(1:2,i)-tail_position_bend(1:2,i); 

        length_of_projection  = dot(segment_tail_bend, segment_tail_head)/norm(segment_tail_head);
        bend_perc(i) = length_of_projection/norm(segment_tail_head);

        if PLOT_trajectory

            if mod(i,round(length(head_theta_bend)/50))==0

                bend_perc(i)
                h_bend_pos = scatter(bend_pos(1,i), bend_pos(2,i), 500, 'filled');     hold on

                h_head_tail = plot([head_position_bend(1,i), tail_position_bend(1,i)], [head_position_bend(2,i), tail_position_bend(2,i)]);

                h_head_slope = plot([head_position_bend(1,i) bend_pos(1,i)], ...
                    [head_position_bend(2,i) bend_pos(2,i)],'g');
                h_tail_slope = plot([tail_position_bend(1,i) bend_pos(1,i)], ...
                    [tail_position_bend(2,i) bend_pos(2,i)],'m');

                projected_tail_head = segment_tail_head/norm(segment_tail_head)*length_of_projection;
                h_PERC = plot([tail_position_bend(1,i), tail_position_bend(1,i)+ projected_tail_head(1)], ...
                    [tail_position_bend(2,i), tail_position_bend(2,i)+ projected_tail_head(2)],'r', 'LineWidth', 3);

                plt.plot_with_head(angles(:,i+start_idx), head_hebi(:,:,i+start_idx));

                pause(0.01)

                delete(h_bend_pos);
                delete(h_head_tail);
                delete(h_head_slope);
                delete(h_tail_slope);  
                delete(h_PERC);
            end
        end
    end
    
    bend_info(b).bend_percentage = bend_perc;
    bend_info(b).bend_pos = bend_pos;
    
    if PLOT
        figure
        plot(time(start_idx:end_idx), bend_perc);
        title('bend percentage');   
    end
end    

 
end