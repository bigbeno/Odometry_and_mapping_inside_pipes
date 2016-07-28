function x_filt = filter_no_delay_with_beginning_and_end(time, x, span_time)


% I want to filter over previous span_time sec)
dt = median(diff(time));
windowSize = round(span_time/dt);
if mod(windowSize,2)
    windowSize = windowSize +1;
end
windowSize;

a = 1;
b = (1/windowSize)*ones(1,windowSize);

x = filter(b,a,x);

% if size(x,1)>1
%     x = [x(windowSize/2: end); NaN(windowSize/2-1,1)];
% else
%     x = [x(windowSize/2: end), NaN(1, windowSize/2-1)];
% end


if size(x,1)>1
    x = [repmat(x(windowSize),windowSize/2,1); x(windowSize: end); repmat(x(end),windowSize/2-1,1)];
else
    x = [repmat(x(windowSize),1, windowSize/2), x(windowSize: end), repmat(x(end),1, windowSize/2-1)];
end



x_filt = x;

