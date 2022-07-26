function y = trapzFilter(u, eps, wn, Ts)

% Second-order digital filter

persistent yk_1 yk_2 numDT denDT

if isempty(yk_1)
   yk_1 = u;
   yk_2 = u;
   rP = exp(-Ts*eps*wn);
   thetaP = Ts*wn*sqrt(1-eps^2);
   polesDT = [rP*exp(thetaP*1i) rP*exp(-thetaP*1i)];
   denDT = poly(polesDT);
   numDT = sum(denDT);
end

yk = numDT*u - denDT(2)*yk_1 - denDT(3)*yk_2;
yk_2 = yk_1;
yk_1 = yk;

y = yk;

end