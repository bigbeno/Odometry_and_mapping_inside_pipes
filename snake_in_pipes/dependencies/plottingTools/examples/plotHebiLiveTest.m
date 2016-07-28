% function plotHebiLiveTest
%Plots the Snake in real time
%You need an active hebi linkage, and you might have to change the module
%number
close all;
clear all;
clc;

    g = HebiLookup.newConnectedGroupFromName('Spare','SA008');
    accelOffsets = computeAccelOffsets(g.getInfo().name);
%     setSEASnakeGains_Strategy4;
% 
%     plt = HebiPlotter('frame','gravity');

    plt = HebiPlotter('frame','gravity', 'accelOffsets', accelOffsets);

    gyro_x=[]; gyro_y=[]; gyro_z=[];
%     accelerations=[]; accelerations_originals = [];
    while(true)
        fbk = g.getNextFeedback;
        
        if ~isempty(fbk)
%             accelerations_originals(:,:,end+1) = [fbk.accelX; fbk.accelY;fbk.accelZ];
%             accelerations(:,:,end+1) = accelerations_originals(:,:,end) - accelOffsets;

    %         fbkFixed.gyroX = fbk.gyroX;
    %         fbkFixed.gyroY = fbk.gyroY;
    %         fbkFixed.gyroZ = fbk.gyroZ;
    % 
    %         fbkFixed.accelX = fbk.accelX;
    %         fbkFixed.accelY = fbk.accelY;          
    %         fbkFixed.accelZ = fbk.accelZ; 
    % 
    %         fbkFixed.position = zeros(1,18);
    %         fbkFixed.time = fbk.time;
           [fk, fk_out] =  plt.plot(fbk); hold on
           grid on
        end
%        axis equal
       
%        for i=1:length(fk_out)
%            drawframe(fk_out(:,:,i), 0.025);
%        end
       
       if (~length(fk)==0) 
          g.setFeedbackFrequency(50);
%             pause();
       end
    end
