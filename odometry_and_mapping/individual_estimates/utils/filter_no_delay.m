function x_filt = filter_no_delay(time, x, span_time)


% I want to double filter over previous 0.5 sec)
windowSize = find(time>span_time,1);
if mod(windowSize,2)
    windowSize = windowSize +1;
end
windowSize

a = 1;
b = (1/windowSize)*ones(1,windowSize);

x = filter(b,a,x);

if size(x,1)>1
    x = [x(windowSize/2: end); NaN(windowSize/2-1,1)];
else
    x = [x(windowSize/2: end), NaN(1, windowSize/2-1)];
end

x_filt = x;

