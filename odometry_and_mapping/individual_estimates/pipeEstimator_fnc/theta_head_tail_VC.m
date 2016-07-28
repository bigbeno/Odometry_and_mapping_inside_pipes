function [head_theta, tail_theta] = theta_head_tail_VC(module_positions, bend_info)

    PLOT = false;

    n_modules = size(module_positions, 2);
    
    theta_pipe1_pp = nan(1,size(module_positions, 3));
    theta_pipe2_pp = nan(1,size(module_positions, 3));
    theta_VC_log = nan(1,size(module_positions, 3));
    what_to_consider = nan(1,size(module_positions, 3));

    h_all = [];h_nice = [];
    
   
    for i=1:size(module_positions, 3)
             
        xy_pts =module_positions(1:2,:,i)';
        
        in_a_bend = false;
        for b=1:length(bend_info)
            if (i>=bend_info(b).start_idx && i<=bend_info(b).end_idx)
                in_a_bend = true;
                bend_perc_pp = bend_info(b).bend_percentage(i-bend_info(b).start_idx+1);
                bend_angle_pp(i) = -bend_info(b).angle;
                break;
            end
        end

        if ~in_a_bend
            
            %on a straight pipe, consider whole body
            min_module_idx = 1;
            max_module_idx = n_modules;
            
            % "whole"
            what_to_consider(i) = 0;
            
        else

            if bend_perc_pp>0.5
                %Should consider tail part
                min_module_idx = max(ceil((1-bend_perc_pp)*n_modules)+2,1);
                max_module_idx = n_modules;  
                 % "tail"
                what_to_consider(i) = 2;
                
            else
                %Should consider head part                
                min_module_idx = 1;
                max_module_idx = min(floor((1-bend_perc_pp)*n_modules)-2, n_modules);
                 % "head"
                what_to_consider(i) = 3;  
                
            end
        end

        nice_modules_idx = min_module_idx:max_module_idx;
        xy_nice_pts = xy_pts(nice_modules_idx,:);
        
        CoM = mean(xy_nice_pts);
        xy_nice_pts = xy_nice_pts - repmat(CoM,size(xy_nice_pts,1),1);   

        [S, U, V] = svd( xy_nice_pts );

        if dot( V(:,1), xy_nice_pts(1,:) ) > 0 %was <
            V(:,1) = -V(:,1);
        end
        main_axis = V(:,1);
        
        theta_VC(i) = rad2deg(atan2(main_axis(2), main_axis(1)));
        
        if PLOT
            if mod(i,round(size(module_positions, 3)/50))==1

                delete(h_all);
                delete(h_nice);
                h_all = plot(xy_pts(:,1), xy_pts(:,2), 'k*'); hold on
                h_nice = plot(xy_nice_pts(:,1), xy_nice_pts(:,2), 'r*');
                plot([0 main_axis(1)], [0 main_axis(2)]);

                pause(0.01);
    %             pause
        end
        end
        
        
    end
    
    theta_VC_uw = unwrap(deg2rad(theta_VC));
    theta_VC_uw = rad2deg(theta_VC_uw);
    
    % Appianate VC for filtering
    idx_switching_of_main_pipe = find(diff(what_to_consider)==1);
    idx_switching_of_main_pipe= idx_switching_of_main_pipe+ones(1, length(idx_switching_of_main_pipe));
    
    theta_VC_straight= theta_VC_uw;
    for i = idx_switching_of_main_pipe
        theta_VC_straight(i:end) = theta_VC_straight(i:end) + ...
            repmat(bend_angle_pp(i),1,length(theta_VC_straight(i:end)));
    end
    
    theta_VC_straight_f = filter_no_delay_with_beginning_and_end...
        (1:length(theta_VC_straight), theta_VC_straight, 4000);
    theta_VC_straight_f = filter_no_delay_with_beginning_and_end...
        (1:length(theta_VC_straight), theta_VC_straight_f, 4000);    
    
    theta_VC_f= theta_VC_straight_f;    
    for i = idx_switching_of_main_pipe
        theta_VC_f(i:end) = theta_VC_f(i:end) - ...
            repmat(bend_angle_pp(i),1,length(theta_VC_f(i:end)));
    end
    
    whole_idx = find(what_to_consider == 0);    
    head_idx = find(what_to_consider == 2);
    tail_idx = find(what_to_consider == 3);
    
    theta_pipe1_pp([whole_idx head_idx]) = theta_VC_f([whole_idx head_idx]);
    theta_pipe2_pp([ whole_idx tail_idx]) = theta_VC_f([ whole_idx tail_idx]);
    theta_pipe1_pp(tail_idx) = theta_VC_f(tail_idx)+...
        bend_angle_pp(tail_idx);
    theta_pipe2_pp(head_idx) = theta_VC_f(head_idx)-...
        bend_angle_pp(head_idx);   
    
    if PLOT
        figure
        plot(theta_VC_uw); hold on
        plot(theta_VC_straight)
        plot(theta_VC_straight_f); hold on    
        plot(theta_VC_f, '*'); hold on        
        plot(theta_pipe2_pp); hold on
        plot(theta_pipe1_pp)
        legend('VC original', 'VC appianated', 'VC appianated filtered',...
            'VC original filtered', 'pipe head', 'pipe tail');
    end

    %% WARNING FOR SOME REASON THEY ARE OPPOSITE
    head_theta = theta_pipe2_pp;
    tail_theta = theta_pipe1_pp;
end