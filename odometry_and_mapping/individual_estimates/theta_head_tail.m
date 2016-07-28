function [d_theta_f, head_theta, tail_theta] = theta_head_tail(module_positions, time, head_hebi, angles)

PLOT = false;

if PLOT
    plt = HebiPlotter('frame','head');
    f3 = figure;
    h1= []; h2= []; h3= []; h4= [];
end


theta1_from_modules_rad = zeros(1, length(module_positions));
theta2_from_modules_rad = zeros(1, length(module_positions));


% load('theta1_thata2'); %TODO REMOVE! it's just to save time
for i=1:length(module_positions)
    xy_pts =module_positions(1:2,:,i)';
    modules_filt = 3;
    xy_pts_filt = xy_pts;
    xy_pts_filt(:,1) = filter_no_delay_extended(1:19,xy_pts(:,1),modules_filt);
    xy_pts_filt(:,2) = filter_no_delay_extended(1:19,xy_pts(:,2),modules_filt);
    xy_pts_filt(:,1) = filter_no_delay_extended(1:19,xy_pts_filt(:,1),modules_filt);
    xy_pts_filt(:,2) = filter_no_delay_extended(1:19,xy_pts_filt(:,2),modules_filt);    

    deltas = diff(xy_pts_filt);
    dx_start = deltas(1,1); dx_end = deltas(end,1);
    dy_start = deltas(1,2); dy_end = deltas(end,2);
    m1_filt = dy_start/dx_start;
    m2_filt = dy_end/dx_end;
    q1 = xy_pts_filt(1,2)- xy_pts_filt(1,1)*m1_filt;
    q2 = xy_pts_filt(end,2) - xy_pts_filt(end,1)*m2_filt;
    theta1 = atan2(dy_start, dx_start);%atan(m1_filt);    
    theta2 = atan2(dy_end, dx_end);   

    theta1_from_modules_rad(i) =  theta1;        
    theta2_from_modules_rad(i) =  theta2;        

    if PLOT && mod(i,round(length(module_positions)/200))==1
        delete(h1);
        delete(h2);    
        delete(h3);    
        delete(h4);   
        

        %From filtered module positions 
        set(0, 'currentfigure', f3);       
        xi = linspace(min(xy_pts(:,1)),max(xy_pts(:,1)),1025);
        h1 = plot(xy_pts(:,1), xy_pts(:,2), '*k'); hold on     
        h2 = plot(xy_pts_filt(:,1), xy_pts_filt(:,2), 'k-','LineWidth',3); hold on

        yfit1 = polyval([m1_filt, q1],xi);
        yfit2 = polyval([m2_filt, q2],xi);    
        [~, meeting_idx] = min(abs(yfit1-yfit2));
        h3 = plot([xy_pts(1,1),xi(meeting_idx)], ...
        polyval([m1_filt, q1],[xy_pts(1,1),xi(meeting_idx)]), 'r', 'LineWidth',3);
        h4 = plot([xy_pts(end,1),xi(meeting_idx)], ...
        polyval([m2_filt, q2],[xy_pts(end,1),xi(meeting_idx)]), 'b', 'LineWidth',3);
        axis equal
        pause(0.01)
        plt.plot_with_head(angles(:,i), head_hebi(:,:,i), []); 
        if i==1
            pause()
        end
    end        

end

theta1_from_modules_rad_uw= unwrap(theta1_from_modules_rad);
theta2_from_modules_rad_uw= unwrap(theta2_from_modules_rad);

theta1_from_modules = rad2deg(theta1_from_modules_rad_uw);
theta2_from_modules = rad2deg(theta2_from_modules_rad_uw);

filt_time = 30;%60;
theta1_from_modules_f = filter_no_delay_extended(time, theta1_from_modules, filt_time);
theta2_from_modules_f = filter_no_delay_extended(time, theta2_from_modules, filt_time);
theta1_from_modules_f = filter_no_delay_with_beginning_and_end(time, theta1_from_modules_f, filt_time/2);
theta2_from_modules_f = filter_no_delay_with_beginning_and_end(time, theta2_from_modules_f, filt_time/2);

head_theta = theta1_from_modules_f;
tail_theta = theta2_from_modules_f;

d_theta = theta1_from_modules - theta2_from_modules;
d_theta_f = theta1_from_modules_f - theta2_from_modules_f;


figure
subplot(1,2,1)
plot(time, theta1_from_modules,'.-'); hold on
plot(time, theta2_from_modules,'.-'); hold on
plot(time, theta1_from_modules_f,'.-r'); hold on    
plot(time, theta2_from_modules_f,'.-r'); hold on
grid on
xlabel('Time [sec]')
ylabel('theta [deg]');
title('Estimated head/tail angles')
legend('head', 'tail')

subplot(1,2,2)
plot(time, d_theta,'.-');  hold on
plot(time, d_theta_f,'.-r'); 
grid on
xlabel('Time [sec]')
ylabel('dtheta [deg]');
title('Estimated relative angle')


end