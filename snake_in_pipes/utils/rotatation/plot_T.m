function plot_T(T,time, title_name)
% Given a sequence of homogeneous transformation matrices 4x4xn
% plots the reference frames which represent them (one very 50)

figure
for i=1:round(length(time)/50):length(time)
    drawframe(T(:,:,i), 0.05); hold on
end
axis equal
title(title_name);


end