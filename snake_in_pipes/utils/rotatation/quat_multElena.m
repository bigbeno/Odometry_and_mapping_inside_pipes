function [ q_prod ] = quat_mult( q1, q2 )
%  ---I had to append Elena to the name because there was a conflict with
%  the MATLAB Aerospace toolbox ---
% QUAT_MULT Product of 2 quaternions (scalar on top)
% This function calculates q1 * q2

    a = q1(1);
    b = q1(2);
    c = q1(3);
    d = q1(4);

    e = q2(1);
    f = q2(2);
    g = q2(3);
    h = q2(4);
    
    % From Wikipedia:
    % http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    
    if isa(q1, 'sym')
        q_prod = sym('q_prod', [4 1]);
    else
        q_prod = zeros(4,1);
    end
    
    q_prod(1) = a*e - b*f - c*g - d*h;
    q_prod(2) = a*f + b*e + c*h - d*g;
    q_prod(3) = a*g + c*e + d*f - b*h;
    q_prod(4) = a*h + d*e + b*g - c*f;
    
    

end

