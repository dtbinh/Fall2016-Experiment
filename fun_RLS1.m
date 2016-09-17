function r = fun_RLS1 (r,z,H)

r.R=1;
%algorithm
r.K = r.P*H'*inv(H*r.P*H'+r.R);
r.x = r.x + r.K*(z-H*r.x);
r.P = (1 - r.K*H)*r.P;
