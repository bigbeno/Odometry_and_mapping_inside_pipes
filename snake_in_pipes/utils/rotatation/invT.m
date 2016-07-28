function invT = invT(T)
% Given a transformation represented with (q,t), returns the inverse
% transformation
% with q quaternion and t translation vector, and T = [q t]

q = T(1:4); t = T(5:7);

% invT = [ R', -R'*t ]
qinv = quatinvElena(q);
tinv = - quat_multElena(quat_multElena(q, [0 t]), quatinvElena(q));
tinv = tinv(2:4);

invT = [qinv'; tinv];

end