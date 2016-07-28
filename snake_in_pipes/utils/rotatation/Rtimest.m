function tMult = Rtimest(q, t)
% Computes the equivalent of R*t given q instead of R
% with R rotation matrix, q quatertion vector, and t translation vector


if size(q,1)>1
    q = q';
end

if size(t,1)>1
    t = t';
end

tMult = quat_multElena(quat_multElena(quatinvElena(q), [0 t]), q);
tMult = tMult(2:4);

end