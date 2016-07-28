function x_filt = filter_no_delay_extended(time, x_orig, span_time)


% I want to filter over previous span_time sec)
dt = median(diff(time));
windowSize = round(span_time/dt);
if mod(windowSize,2)
    windowSize = windowSize +1;
end
windowSize;

a = 1;
b = (1/windowSize)*ones(1,windowSize);
x = filter(b,a,x_orig);



if size(x,1)>1
    x = [NaN(windowSize/2,1); x(windowSize: end); NaN(windowSize/2-1,1)];
else
    x = [NaN(1, windowSize/2), x(windowSize: end), NaN(1, windowSize/2-1)];
end

for i = 1:(windowSize/2)
    x(i) = mean(x_orig(1:2*i));
end

for i = (length(x)-(windowSize/2-1)):length(x)
    j = length(x) - i;
    x(i) = mean(x_orig((i-j):end));
end

x_filt = x;

