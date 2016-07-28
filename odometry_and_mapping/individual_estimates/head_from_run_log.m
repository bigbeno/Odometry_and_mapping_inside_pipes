function [head, time, miscOut] = head_from_run_log(run_log, misc)

NEW_LOG = misc.NEW_LOG;
ORIENTATION = misc.ORIENTATION;
CORRECT_CYLINDER = misc.CORRECT_CYLINDER;
dz = misc.dz;


%Last entry could be corrupted from the control c
run_log = run_log(1:end-1);
time = [run_log.t] -run_log(1).t;

%Fixing time mismatch
% starting_idx = find(time>0.4,1);
% run_log = run_log(starting_idx:end);

time = [run_log.t] -run_log(1).t;
num_samples = length(time);
num_joints = length(run_log(1).position);
angles = reshape([run_log.position], ...
    num_joints,[]);
VC = reshape([run_log.VC_frame], 4, 4, []);
head_CF = reshape([run_log.head_frame], 4, 4, []);
head_hebi = zeros(size(head_CF));
gravity = [ 0 0 -1]';

count = 1;
for i=1:length(run_log)
    if isnan(run_log(i).all_frames)==0
        module_frames(:,:,:,count) = run_log(i).all_frames;
        count = count+1;
    end 
end

if isfield(run_log, 'VC_magnitude')
    VC_magnitude = [run_log.VC_magnitude];
else
    VC_magnitude = zeros(1,length(time));
end

time_straight = min(misc.time_straight, time(end));

%% Rotation of the system and forward motion
VC_BEGINNING = true;

    % VC from whole run
    
    end_straight_idx = find(time>=time_straight,1);
%     
    VC_x_axis = reshape(VC(1:3,1,:),3,[]);
    straight_idx = 1:end_straight_idx;
    
%     mean_VC_x_in_gravity = mean(VC_x_axis(:,1:end_straight_idx),2);
%     mean_VC_x_in_gravity(3) = 0;
%     mean_VC_x_in_gravity = mean_VC_x_in_gravity/norm(mean_VC_x_in_gravity);


    windowSize = min(4000, length(straight_idx)-2);

    filtered_VC_x_in_gravity(1,:) = filter_no_delay_with_beginning_and_end...
        (1:length(VC_x_axis), VC_x_axis(1,straight_idx), windowSize);
    filtered_VC_x_in_gravity(2,:) = filter_no_delay_with_beginning_and_end...
        (1:length(VC_x_axis), VC_x_axis(2,straight_idx), windowSize);  
   
    
    filtered_VC_x_in_gravity(1,:) = filter_no_delay_with_beginning_and_end...
        (1:length(VC_x_axis), filtered_VC_x_in_gravity(1,:), windowSize);
    filtered_VC_x_in_gravity(2,:) = filter_no_delay_with_beginning_and_end...
        (1:length(VC_x_axis), filtered_VC_x_in_gravity(2,:), windowSize);  

    filtered_VC_x_in_gravity(3,:) = zeros(1,length(filtered_VC_x_in_gravity));
    
%    
    time_straight = time(straight_idx);
    
    figure
    subplot(3,1,1)
    plot(time, VC_x_axis(1,:)); hold on
    plot(time_straight, filtered_VC_x_in_gravity(1,:));
    ylabel('x component')
    title('VC main axis')
    subplot(3,1,2)
    plot(time, VC_x_axis(2,:)); hold on
    plot(time_straight, filtered_VC_x_in_gravity(2,:));
    legend('original', 'filtered');
    ylabel('y component')
    subplot(3,1,3)
    plot(time, VC_x_axis(3,:)); hold on
    plot(time_straight, filtered_VC_x_in_gravity(3,:));
    xlabel('time')
    ylabel('z component')

%     figure
%     subplot(4,1,1)
%     plot(time, VC_x_axis(1,:)); hold on
%     plot([time_straight(1) time_straight(end)], [mean_VC_x_in_gravity(1) mean_VC_x_in_gravity(1)]);
%     ylabel('x component')
%     title('VC main axis')
%     subplot(4,1,2)
%     plot(time, VC_x_axis(2,:)); hold on
%     plot([time_straight(1) time_straight(end)], [mean_VC_x_in_gravity(2) mean_VC_x_in_gravity(2)]);
%     legend('original', 'filtered');
%     ylabel('y component')
%     subplot(4,1,3)
%     plot(time, VC_x_axis(3,:)); hold on
%     plot([time_straight(1) time_straight(end)], [mean_VC_x_in_gravity(3) mean_VC_x_in_gravity(3)]);
%     xlabel('time')
%     ylabel('z component')
%     subplot(4,1,4)
%     plot(time, VC_magnitude(1,:));
%     ylabel('magnitude');


    % With VC from whole run
% head_perfect_v(:,3) = mean_VC_x_in_gravity;
% head_perfect_v(:,2) = [0 0 -1]; 
% head_perfect_v(:,1) = cross(head_perfect_v(:,3), head_perfect_v(:,2));
% head_perfect_v = R2T(head_perfect_v);
% T_gravity_in_vertical = inv(head_perfect_v);
% head_perfect_h(:,1) = mean_VC_x_in_gravity;
% head_perfect_h(:,3) = [0 0 1];
% head_perfect_h(:,2) = cross(head_perfect_h(:,3), head_perfect_h(:,1));
% head_perfect_h = R2T(head_perfect_h);
% T_gravity_in_horizontal = inv(head_perfect_h);
    
    bended_idx = setdiff(1:num_samples,straight_idx);

    VC_x_axis_extended = nan(3,length(num_samples));
    VC_x_axis_extended(:,straight_idx) = filtered_VC_x_in_gravity;
    VC_x_axis_extended(:,bended_idx) = repmat(filtered_VC_x_in_gravity(:,end), ...
        1, length(bended_idx));

    

    if (strcmp(ORIENTATION,'vertical'))
        % With VC filtered
        head_perfect_v = zeros(4,4,num_samples);
        head_perfect_v(4,4,:) = 1;
        head_perfect_v(1:3,3, :) = VC_x_axis_extended;
%         head_perfect_v(1:3,3, bended_idx) = ;        
        head_perfect_v(1:3,2,:) = repmat([0 0 -1]',1,1,num_samples); 
        head_perfect_v(1:3,1,:) = cross(head_perfect_v(1:3,3,:), head_perfect_v(1:3,2,:));
        T_gravity_in_vertical = zeros(size(head_perfect_v));
        for i=1:num_samples
            if isnan(norm(head_perfect_v(:,:,i)))==0
                T_gravity_in_vertical(:,:,i) = inv(head_perfect_v(:,:,i));
            else
                T_gravity_in_vertical(:,:,i) = nan(4);
            end
        end
        T_gravity = T_gravity_in_vertical;
    elseif (strcmp(ORIENTATION,'horizontal'))
        head_perfect_h = zeros(4,4,num_samples);
        head_perfect_h(4,4,:) = 1;
%         head_perfect_h(1:3,1, :) = filtered_VC_x_in_gravity;
%         head_perfect_h(1:3,1, straight_idx) = filtered_VC_x_in_gravity;
%         head_perfect_h(1:3,1, bended_idx) = repmat(filtered_VC_x_in_gravity(:,end), 1, bended_idx);       
        head_perfect_h(1:3,1, :) = VC_x_axis_extended;
        head_perfect_h(1:3,3,:) = repmat([0 0 1]',1,1,num_samples); 
        head_perfect_h(1:3,2,:) = cross(head_perfect_h(1:3,3,:), head_perfect_h(1:3,1,:));
        T_gravity_in_horizontal = zeros(size(head_perfect_h));
        for i=1:num_samples
            if isnan(norm(head_perfect_h(:,:,i)))==0
                T_gravity_in_horizontal(:,:,i) = inv(head_perfect_h(:,:,i));
            else
                T_gravity_in_horizontal(:,:,i) = nan(4);
            end
        end
        T_gravity = T_gravity_in_horizontal;
    elseif (strcmp(ORIENTATION,'original'))
        T_gravity = eye(4);
    end
    
    % With VC from whole run
    if (strcmp(ORIENTATION,'vertical'))
        T_gravity = T_gravity_in_vertical;
    elseif (strcmp(ORIENTATION,'horizontal'))
        T_gravity = T_gravity_in_horizontal;
    elseif (strcmp(ORIENTATION,'original'))
        T_gravity = eye(4);
    end

%     pipe_direction = T_gravity(1:3,1:3,1)* mean_VC_x_in_gravity(:,1);
%     gravity_direction = T_gravity(1:3,1:3)* gravity;

    pipe_direction = T_gravity(1:3,1:3,1)* filtered_VC_x_in_gravity(:,1);
    gravity_direction = T_gravity(1:3,1:3,1)* gravity;

head_CF_rotated = zeros(size(head_CF));
head_hebi_rotataed = zeros(size(head_hebi));
head_CF_cylinder = zeros(size(head_CF));
head_hebi_cylinder = zeros(size(head_CF));
VC_rotated = zeros(size(VC));

    pltPre = HebiPlotter('frame','head');
%       pltPre2 = HebiPlotter('frame','head');
  
    
    
     r = 0.05; %[m]
    [X,Y,Z] = cylinder(r);
%     length_of_snake = 18*.0639;
    Z = Z*1.2-0.6;
%     Z = (Z * (0-(0-length_of_snake)+0.1))+(0-length_of_snake);%Z*0.2+0.5;

    top = [X(1,:); Y(1,:); Z(1,:)];
    bottom = [X(2,:); Y(2,:); Z(2,:)];  

    if (pipe_direction(3)==0)
        v = [0 0 1]';
    else
        x = randn; y = randn; z = -(pipe_direction(1)*x + pipe_direction(2)*y)/pipe_direction(3);
        v = [x y z]'; v = v/norm(v);
    end
    R = [v, cross(pipe_direction, v), pipe_direction];
    if ~((det(R)-1)<=10*eps && (norm(R)-1)<=10*eps)
        disp('ERROR IN R COMPUTATION');
        pause;
    end

top = R*top;
bottom = R*bottom;
X = [top(1,:); bottom(1,:)];
Y = [top(2,:); bottom(2,:)];
Z = [top(3,:); bottom(3,:)];
pipe_center = mean([mean(top,2), mean(bottom,2)],2);

other_direction = cross(pipe_direction, gravity_direction);
if (norm(other_direction-[0 0 0 ]')<=eps)
    other_direction = v;
end
translation_for_gravity = other_direction * 3*r;
gravity_origin = translation_for_gravity+pipe_center;
    
 
tic
module_frames_rotated= nan(4,4,size(module_frames,3), num_samples);
for i=1:num_samples

%     disp([num2str(i/num_samples*100),'%']);

        % Head hebi is rotated of 180 degrees in local frame
        rotz_pi = eye(4); rotz_pi(1,1) = -1; rotz_pi(2,2) = -1;
        head_hebi(:,:,i) = head_CF(:,:,i) * rotz_pi;

        %Displace along the z axis of the vertical frame (motion)
        dz_now = dz * i/num_samples;
        fw_motion = eye(4); fw_motion(1:3,4) = pipe_direction.*dz_now;   

        % Verticalize/horizontalize
        if size(T_gravity,3)==1    
            T_gravity_current = T_gravity;
        else
            T_gravity_current = T_gravity(:,:,i);
        end

        head_CF_rotated(:,:,i) = T_gravity_current*head_CF(:,:,i);
        head_hebi_rotataed(:,:,i) = T_gravity_current*head_hebi(:,:,i);
    %     
        if NEW_LOG
%             if ~isnan(run_log(i).all_frames)
                for j=1:size(module_frames,3)
                    module_frames_rotated(:,:,j,i) = fw_motion*T_gravity_current*...
                                        module_frames(:,:,j,i);
                end
%             end
        end


        % Add forward motion
        head_CF_rotated(:,:,i) = fw_motion*head_CF_rotated(:,:,i);
        head_hebi_rotataed(:,:,i) = fw_motion*head_hebi_rotataed(:,:,i);


        VC_rotated(:,:,i) = T_gravity_current*VC(:,:,i);
 
end
toc



if CORRECT_CYLINDER

    disp('End of finding best cylnder!');
end

if CORRECT_CYLINDER
    head = head_hebi_cylinder;
else
%     if CF_INSTEAD_OF_HEBI
        head = head_CF_rotated;
%     else
%         head = head_hebi_rotataed;
%     end
end

VirtualChassis = VC_rotated;

miscOut.pipe_R = [pipe_direction, cross(v, pipe_direction), v];
miscOut.pipe_direction = pipe_direction;
miscOut.angles = angles;
miscOut.VirtualChassis = VirtualChassis;
miscOut.X = X;
miscOut.Y = Y;
miscOut.Z = Z;
miscOut.gravity_origin = gravity_origin;
miscOut.gravity_direction = gravity_direction;
miscOut.r = r;
miscOut.module_frames_rotated = module_frames_rotated;
miscOut.head_hebi = head_hebi_rotataed;
miscOut.VC_magnitude = VC_magnitude;

end