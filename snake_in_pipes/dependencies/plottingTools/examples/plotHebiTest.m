% function plotHebiTest()
%Plots and animnates a large manipulator moving all joints from their
%negative to positive limits

clear all
    close all;

    num_links = 10;
    num_samples = 20;
    angles = [];
    plt = HebiPlotter('resolution', 'low');
%     plt = HebiPlotter('frame','VC')
    for i=1:num_links
%         angles = [angles, linspace(-pi/2, pi/2, num_samples)'];
        angles = [angles, zeros(num_samples,1)];
    end
    
    load ('accelerations_rolling_slowly_comp');
    
    T_head_horiz = eye(4);
T_head_horiz(1:3, 1:3) = [    0.0000         0    1.0000; ...
                                 0    1.0000         0; ...
                           -1.0000         0    0.0000];
                       
%      range = 1:3;
    
    for i=2:50:length(accelerations)
        fbk.position = zeros(1,18);
        fbk.accelX = accelerations(1,1:18,i);
        fbk.accelY = accelerations(2,1:18,i);
        fbk.accelZ = accelerations(3,1:18,i);
%         fbk.accelX = ones(1,18);
%         fbk.accelY = zeros(1,18);
%         fbk.accelZ = zeros(1,18);
%         fbk.position = zeros(1,length(range));
%         fbk.accelX = accelerations(1,range,i);
%         fbk.accelY = accelerations(2,range,i);
%         fbk.accelZ = accelerations(3,range,i);
        [fk, fk_out ] = plt.plot(fbk); hold on
        
%         if ~isempty(fk)
%             for j=1:18
%                 drawframe(fk(:,:,j), 0.05);
%             end
%         end
%         if ~isempty(fk_out)
%             for j=1:18
%                 drawframe(fk_out(:,:,j), 0.05);
%             end
%         end
        if i==1
            pause
        end
        pause(.5);
        
    end
    

% end
