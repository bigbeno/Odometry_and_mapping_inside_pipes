function T = R2T(R)
%converts a rotation matrix to an homogeonous matrix

T = eye(4);
T(1:3,1:3) = R;

end
