function C_CF_interp = interpolate_R(C_CF,time_CF, time_VO)


% Get C_CF values at  time_VO values
C_CF_interp = zeros(3,3,length(time_VO));
for i=1:length(time_VO)
    idx = find(time_CF>=time_VO(i),1);
    if isempty(idx)
        idx = length(time_CF);
    end
    C_CF_interp(:,:,i) = C_CF(:,:,idx);             
end


end

function x_f =  filter_shit(x,time, span_time)

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