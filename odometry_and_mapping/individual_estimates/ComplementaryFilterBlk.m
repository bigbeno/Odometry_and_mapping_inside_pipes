function [CF_T_cam, time, miscOut] = ComplementaryFilterBlk(run_log)

    time = [run_log.t] -run_log(1).t;
    num_samples = length(time);    

    head_CF = reshape([run_log.head_frame], 4, 4, []);
    head_hebi = zeros(size(head_CF));
    count=1;
    for i=1:length(run_log)
        if isnan(run_log(i).all_frames)==0
            module_frames(:,:,:,count) = run_log(i).all_frames;
            count = count+1;
        end 
    end
    %gravity = [ 0 0 -1]';
      

    mismatch_position_head_camera = [0 0 0.03 ]';
    mismatch_CFframe_openCVframe = rotz(pi/2);
    rotz_pi = eye(4); rotz_pi(1,1) = -1; rotz_pi(2,2) = -1;    
    for i=1:num_samples
        head_hebi(:,:,i) = head_CF(:,:,i) * rotz_pi; 
        head_CF(:,:,i) = head_CF(:,:,i)...
                *t2T(mismatch_position_head_camera)...
                *R2T(mismatch_CFframe_openCVframe);         
    end    
    
    
    CF_T_cam = head_CF;    
    miscOut.head_hebi = head_hebi;
    miscOut.modules = module_frames;
    
    t = reshape(head_CF(1:3,4,:),3,[]);
    [pitch, roll, yaw] = ...
        dcm2angleElena( head_CF(1:3,1:3,:), 'YXZ' );


    figure
    ax1 = subplot(3,2,1);
    plot(time, t(1,:)); hold on
    ylabel('dx [m]') 
    xlabel('Time [sec]') 
    title('COMPLEMENTARY FILTER translation');
    ax2 = subplot(3,2,3);
    plot(time, t(2,:)); hold on
    ylabel('dy [m]') 
    xlabel('Time [sec]') 
    ax3 = subplot(3,2,5);
    plot(time, t(3,:)); hold on
    ylabel('dz [m]') 
    xlabel('Time [sec]') 
    linkaxes([ax1,ax2,ax3],'x')
    ax11 = subplot(3,2,2);
    plot(time, pitch); hold on
    plot([min(time), max(time)], [0 0], '--k');
    ylabel('d roll [rad]') ; 
    xlabel('Time [sec]') ;
    title('COMPLEMENTARY FILTER rotation');
    ax21 = subplot(3,2,4);
    plot(time, roll); hold on
    plot([min(time), max(time)], [0 0], '--k');
    ylabel('d pitch [rad]') ; 
    xlabel('Time [sec]') ;
    ax31 = subplot(3,2,6);
    plot(time, yaw); hold on
    plot([min(time), max(time)], [0 0], '--k');
    ylabel('d yaw [rad]'); 
    xlabel('Time [sec]');
    linkaxes([ax11,ax21,ax31],'x')


end