function qinv = quatinvElena( q )
%  ---I had to append Elena to the name because there was a conflict with
%  the MATLAB Aerospace toolbox ---
%  QUATINV Calculate the inverse of a quaternion.
%   N = QUATINV( Q ) calculates the inverse, N, for a given quaternion, Q.  
%   Input Q is an M-by-4 matrix containing M quaternions.  N returns an 
%   M-by-4 matrix of inverses.  Each element of Q must be a real number.  
%   Additionally, Q has its scalar number as the first column.
%
%   Examples:
%
%   Determine the inverse of q = [1 0 1 0]:
%      qinv = quatinv([1 0 1 0])
%
%   See also QUATCONJ, QUATDIVIDE, QUATMOD, QUATMULTIPLY, QUATNORM, 
%   QUATNORMALIZE, QUATROTATE.

% qinv  = quatconjElena( q );

if (size(q,2) ~= 4)
    q = q';
end
qinv  = quatconjElena( q )./sum(q.^2,2); %( q )*ones(1,4));

