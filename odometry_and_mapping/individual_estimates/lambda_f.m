function lambda = lambda_f(C_CF, T_VO, time_VO)
% Computes scaling factor of Visual Odometry translation

% Compute CF differential values
T_CF = zeros(4,4,length(C_CF));
T_CF(4,4,:) = 1;
for i=2:length(C_CF)
    if isnan(norm(C_CF(:,:,i-1)))
        T_CF(:,:,i) = nan(4);
    else
        T_CF(:,:,i) = inv(C_CF(:,:,i-1))*C_CF(:,:,i);
    end
end


%%
dCF_x = reshape(T_CF(1,4,:),1,[]);
dCF_y = reshape(T_CF(2,4,:),1,[]);
dCF_z = reshape(T_CF(3,4,:),1,[]);

dAO_x = reshape(T_VO(1,4,:),1,[]);
dAO_y = reshape(T_VO(2,4,:),1,[]);
dAO_z = reshape(T_VO(3,4,:),1,[]);

dCF_planar_norm = sqrt(dCF_x.^2 + dCF_y.^2);
dAO_planar_norm = sqrt(dAO_x.^2 + dAO_y.^2 );

lambda_uf = medfilt1(dCF_planar_norm./dAO_planar_norm,3);
lambda = filter_no_delay_with_beginning_and_end(time_VO,lambda_uf, 10);


%%
figure
plot(dCF_planar_norm./dAO_planar_norm); hold on
plot(lambda,'r');
legend('original', 'filtered')
xlabel('TIme [sec]')
ylabel('Scale [-]')
title('Scale of Visual Odometry translation')

%% Compare VO and CF
dAO_x = dAO_x.*lambda;
dAO_y = dAO_y.*lambda;
dAO_z = dAO_z.*lambda;
[dpitch_AO, droll_AO, dyaw_AO] = ...
        dcm2angleElena( T_VO(1:3,1:3,:), 'YXZ' );
[dpitch_CF, droll_CF, dyaw_CF] = ...
    dcm2angleElena( T_CF(1:3,1:3,:), 'YXZ' );

span_time = 0.5;%1;
dAO_x =  filter_custom(dAO_x,time_VO, span_time);
dAO_y =  filter_custom(dAO_y,time_VO, span_time);
dAO_z =  filter_custom(dAO_z,time_VO, span_time);
dCF_x =  filter_custom(dCF_x,time_VO, span_time);
dCF_y =  filter_custom(dCF_y,time_VO, span_time);
dCF_z =  filter_custom(dCF_z,time_VO, span_time);

span_time = 2.5;%5;
dpitch_CF =  filter_custom(dpitch_CF,time_VO, span_time);
droll_CF =  filter_custom(droll_CF,time_VO, span_time);
dyaw_CF =  filter_custom(dyaw_CF,time_VO, span_time);
droll_AO =  filter_custom(droll_AO,time_VO, span_time);
dpitch_AO =  filter_custom(dpitch_AO,time_VO, span_time);
dyaw_AO =  filter_custom(dyaw_AO,time_VO, span_time);

figure
ax1 = subplot(3,2,1);
plot(time_VO, dAO_x); hold on
plot(time_VO, dCF_x,'r'); hold on
plot([min(time_VO), max(time_VO)], [0 0], '--k');
legend('VO','CF')
ylabel('dx [m]') 
xlabel('Time [sec]') 
title(['Differential local values (dt:',num2str(mean(diff(time_VO))), ...
    ' sec)']);
ax2 = subplot(3,2,3);
plot(time_VO, dAO_y); hold on
plot(time_VO, dCF_y,'r'); hold on
plot([min(time_VO), max(time_VO)], [0 0], '--k');
ylabel('dy [m]') 
xlabel('Time [sec]') 
ax3 = subplot(3,2,5);
plot(time_VO, dAO_z); hold on
plot(time_VO, dCF_z,'r'); hold on
plot([min(time_VO), max(time_VO)], [0 0], '--k');
ylabel('dz [m]') 
xlabel('Time [sec]') 
linkaxes([ax1,ax2,ax3],'x')
ax11 = subplot(3,2,2);
plot(time_VO, droll_AO); hold on
plot(time_VO, droll_CF,'r'); hold on
plot([min(time_VO), max(time_VO)], [0 0], '--k');
ylabel('d roll [rad]') ; 
xlabel('Time [sec]') ;
ax21 = subplot(3,2,4);
plot(time_VO, dpitch_AO); hold on
plot(time_VO, dpitch_CF,'r'); hold on
plot([min(time_VO), max(time_VO)], [0 0], '--k');
ylabel('d pitch [rad]') ; 
xlabel('Time [sec]') ;
ax31 = subplot(3,2,6);
plot(time_VO, dyaw_AO); hold on
plot(time_VO, dyaw_CF,'r'); hold on
plot([min(time_VO), max(time_VO)], [0 0], '--k');
ylabel('d yaw [rad]'); 
xlabel('Time [sec]');
linkaxes([ax11,ax21,ax31],'x')
end

function x_f =  filter_custom(x,time, span_time)

% idx_positive = find(time>0,1);
% 
% windowSize = find(time>span_time,1);
windowSize = round(span_time/mean(diff(time)));

if mod(windowSize,2)
    windowSize = windowSize +1;
end
windowSize
a = 1;
b = (1/windowSize)*ones(1,windowSize);

x_f = filter(b,a,x);
if size(x,1) > size(x,2)
    x_f = [x_f(windowSize/2: end); NaN(windowSize/2-1, 1)];
else
    x_f = [x_f(windowSize/2: end), NaN(1, windowSize/2-1)];
end

end