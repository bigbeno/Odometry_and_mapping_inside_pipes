function T = t2T(t)
%converts a rotation matrix to an homogeonous matrix

T = eye(4);
T(1:3,4) = t;

end
