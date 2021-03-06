%% gSat.m
% *Summary:* Compute moments of the saturating function
% $e*(9*\sin(x(i))+\sin(3*x(i)))/8$,
% where $x \sim\mathcal N(m,v)$ and $i$ is a (possibly empty) set of $I$
% indices. The optional  scaling factor $e$ is a vector of length $I$.
% Optionally, compute derivatives of the moments.
%
%    function [M, S, C, dMdm, dSdm, dCdm, dMdv, dSdv, dCdv] = gSat(m, v, i, e)
%
% *Input arguments:*
%
%   m     mean vector of Gaussian                                    [ d       ]
%   v     covariance matrix                                          [ d  x  d ]
%   i     vector of indices of elements to augment                   [ I  x  1 ]
%   e     (optional) scale vector; default: 1                        [ I  x  1 ]
%
% *Output arguments:*
%
%  M     output means                                                [ I       ]
%  V     output covariance matrix                                    [ I  x  I ]
%  C     inv(v) times input-output covariance                        [ d  x  I ]
%  dMdm  derivatives of M w.r.t m                                    [ I  x  d ]
%  dVdm  derivatives of V w.r.t m                                    [I*^2 x  d ]
%  dCdm  derivatives of C w.r.t m                                    [d*I x  d ]
%  dMdv  derivatives of M w.r.t v                                    [ I  x d^2]
%  dVdv  derivatives of V w.r.t v                                    [I^2 x d^2]
%  dCdv  derivatives of C w.r.t v                                    [d*I x d^2]
%
%
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Last modified: 2013-03-25

function [M, S, C, dMdm, dSdm, dCdm, dMdv, dSdv, dCdv] = gSat(m, v, i, e)
%% Code
d = length(m); I = length(i); i = i(:)';
if nargin < 4; e = ones(1, I); end; e = e(:)';

P = [eye(d); 3*eye(d)];                                        % augment inputs
ma = P*m;    madm = P;
va = P*v*P'; vadv = kron(P,P); va = (va+va')/2;

% do the actual augmentation with the right parameters
[M2, S2, C2, Mdma, Sdma, Cdma, Mdva, Sdva, Cdva] ...
  = gSin(ma, va, [i d+i], [9*e e]/8);

P = [eye(I) eye(I)]; Q = [eye(d) 3*eye(d)];
M = P*M2;                                                                % mean
S = P*S2*P'; S = (S+S')/2;                                           % variance
C = Q*C2*P';                                    % inv(v) times input-output cov

if nargout > 3                                        % derivatives if required
  dMdm = P*Mdma*madm;         dMdv = P*Mdva*vadv;
  dSdm = kron(P,P)*Sdma*madm; dSdv = kron(P,P)*Sdva*vadv;
  dCdm = kron(P,Q)*Cdma*madm; dCdv = kron(P,Q)*Cdva*vadv;
end
