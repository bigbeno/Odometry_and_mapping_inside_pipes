function T1T2 = multT(T1,T2)
% % Given two transformations represented with (q,t), returns the
% transformation which is the multiplication of the two
% with q quaternion and t translation vector, and T = [q t]

q1 = T1(1:4); t1 = T1(5:7);
q2 = T2(1:4); t2 = T2(5:7);

% T1T2 = [ R1*R2, R1*t2 + t1] 
qMult = quat_multElena(q2',q1');
tMult = Rtimest(q1',t2');
tMult = tMult +t1;
T1T2 = [qMult; tMult];

end