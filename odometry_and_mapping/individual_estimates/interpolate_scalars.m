function x_interp = interpolate_scalars(x,time_CF, time_VO)

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
