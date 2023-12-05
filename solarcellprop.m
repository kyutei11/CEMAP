function resid = solarcellprop(i,com)
  v     = com.solarcell.v;
  rs    = com.solarcell.rs;
  rp    = com.solarcell.rp;
  alpha = com.solarcell.alpha;

  i0 = (1-1/rp)*(1/(exp(alpha)-exp(alpha*rs)));
  iL = 1 + i0*(exp(alpha*rs)-1);

#  residv = rp*(iL - i - i0*(exp(alpha*(v + rs*i)) - 1))  - v;
  resid = iL - v/rp - i0*(exp(alpha*(v + rs*i)) - 1)   - i;
endfunction


function [i v p] = resid_mpp(x,com)
  i     = x(1);
  v     = x(2);
  rs    = com.solarcell.rs;
  rp    = com.solarcell.rp;
  alpha = com.solarcell.alpha;

  i0 = (1-1/rp)*(1/(exp(alpha)-exp(alpha*rs)));
  iL = 1 + i0*(exp(alpha*rs)-1);

#  power = i*v, resid = d/di(power)
  resid(1) = iL - v/rp - i0*(exp(alpha*(v + rs*i)) - 1)   - i;
  resid(2) = v*(-i0*rs*exp(alpha*(v + rs*i)) - 1);
endfunction
