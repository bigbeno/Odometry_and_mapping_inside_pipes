function qout = quatconj( qin ) 
%  ---I had to append Elena to the name because there was a conflict with
%  the MATLAB Aerospace toolbox ---
%  QUATCONJ Calculate the conjugate of a quaternion.
%   N = QUATCONJ( Q ) calculates the conjugate, N, for a given quaternion, Q.  
%   Input Q is an M-by-4 matrix containing M quaternions.  N returns an 
%   M-by-4 matrix of conjugates.  Each element of Q must be a real number.  
%   Additionally, Q has its scalar number as the first column.
%
%   Examples:
%
%   Determine the conjugate of q = [1 0 1 0]:
%      conj = quatconj([1 0 1 0])
%
%   See also QUATDIVIDE, QUATINV, QUATMOD, QUATMULTIPLY, QUATNORM, 
%   QUATNORMALIZE, QUATROTATE.

%   Copyright 2000-2005 The MathWorks, Inc.
%   $Revision: 1.1.6.1 $  $Date: 2005/11/01 23:39:29 $

% if any(~isreal(qin(:)))
%     error('aero:quatconj:isnotreal','Input elements are not real.');
% end

if (size(qin,2) ~= 4)
    error('aero:quatconj:wrongdim','Input dimension is not M-by-4.');
end

qout = [ qin(:,1)  -qin(:,2:4) ];
