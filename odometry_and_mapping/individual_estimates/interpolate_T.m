function C_CF_interp = interpolate_T(C_CF,time_CF, time_VO)


%Smooth C_CF
transl_CF_uf = reshape(C_CF(1:3,4,:),3,[]);
span_time = 0.66;
transl_CF(1,:) = filter_custom(transl_CF_uf(1,:), time_CF,span_time);
transl_CF(2,:) = filter_custom(transl_CF_uf(2,:), time_CF,span_time);
transl_CF(3,:) = filter_custom(transl_CF_uf(3,:), time_CF,span_time);
transl_CF_interp(1,:) = interp1(time_CF,transl_CF(1,:),time_VO);
transl_CF_interp(2,:) = interp1(time_CF,transl_CF(2,:),time_VO);
transl_CF_interp(3,:) = interp1(time_CF,transl_CF(3,:),time_VO);
% HACK! because of filtering inital planar norm of CF is unreliable
% transl_CF_interp = [transl_CF_interp(:,3), transl_CF_interp(:,3), transl_CF_interp(:,3:end)];

% Get C_CF values at  time_VO values
C_CF_interp = zeros(4,4,length(time_VO));
for i=1:length(time_VO)
    idx = find(time_CF>=time_VO(i),1);
    if isempty(idx)
        idx = length(time_CF);
    end
    C_CF_interp(:,:,i) = C_CF(:,:,idx);             
end
C_CF_interp(1:3,4,:) = reshape(transl_CF_interp, 3, 1, []);

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

%Solving problem when there are Nans at the beginning
messed_up_idx = find(isnan(x_f) & isnan(x)==0);
x_f(messed_up_idx) = x(messed_up_idx);

end