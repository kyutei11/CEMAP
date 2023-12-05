# result of identification :
# rs = 0.040992
# rp = 4.7906
# alpha = 17.809

com.solarcell.Voc = 7.05;
com.solarcell.Isc = 42*0.001;

com.solarcell.rs = 0.040992;
com.solarcell.rp = 4.7906;
com.solarcell.alpha = 17.809;

com.solarcell.cascade = 2;
com.solarcell.parallel = 11;

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


function I = I_solarcell(Volt,com)
  com.solarcell.v = (Volt+0.6)/com.solarcell.cascade/com.solarcell.Voc; # 2-cascaded cell + 1-diode
  i = fzero('solarcellprop',1.0,com);

  I = i * com.solarcell.Isc * com.solarcell.parallel;
  if I<0, I=0; endif
endfunction





function resid = resid_mpp(x,com)
  i     = x(1);
  v     = x(2);
  rs    = com.solarcell.rs;
  rp    = com.solarcell.rp;
  alpha = com.solarcell.alpha;

  i0 = (1-1/rp)*(1/(exp(alpha)-exp(alpha*rs)));
  iL = 1 + i0*(exp(alpha*rs)-1);

  L = x(3);
#  uses lagrange multiplier tecnique
#  power = i*v, phi = i*v + L*resid (of solarcellprop)
#  solve [dphi/di dphi/dv dphi/dL] = 0
  resid(1) = v +    L*(-i0*rs*alpha*exp(alpha*(v + rs*i))        - 1);
  resid(2) = i +    L*( -1/rp - i0*alpha*exp(alpha*(v + rs*i))       );
  resid(3) = iL - v/rp - i0*(  exp(alpha*(v + rs*i)) - 1)   - i;
endfunction



function [I V P] = mpp(com) # get maximum power point
  param = fzeros('resid_mpp',ones(3,1),com,1e-9);
  
  I = param(1) * com.solarcell.Isc * com.solarcell.parallel;
  V = param(2) * com.solarcell.Voc * com.solarcell.cascade - 0.6;
  P = I*V;
endfunction
