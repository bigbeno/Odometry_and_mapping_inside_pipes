function x_interp = interpolate_scalars(x,time_CF, time_VO)
% Given x at time time_CF, return x at time instants time_VO
% It is assumed t_CO has much higher frequency than time_VO
% so you can just get the value at the time instant which is closer to
% time_VO instant

%Put it with time on columns
if size(x,1)>size(x,2)
    x = x';
end

% Get C_CF values at  time_VO values
x_interp = zeros(size(x,1),length(time_VO));
for i=1:length(time_VO)
    idx = find(time_CF>=time_VO(i),1);
    if isempty(idx)
        idx = length(time_CF);
    end
    x_interp(:,i) = x(:,idx);             
end


end
