#Motor parameters:RS-385PH

# Resistance of a power unit is quite important.

# basic param.
com.m.motor = 0.07;
com.R.motor = 6.0;
com.Kv = 505;
com.Imin = 0.1;

# parameters related to maximum voltage
function Imax = Imax(V,com)
	Imax = V/com.R.motor;
endfunction

function Nmax = Nmax(V,com)
	Nmax = com.Kv*V;
endfunction

function Tmax = Tmax(V,com)
	Tmax = V*(Imax(V,com) - com.Imin)/(Nmax(V,com)/60);
endfunction

# parameters related to voltage and angular velocity

function Power = P_Motor(V,n,com)
	Power = Tmax(V,com)*(1-n/Nmax(V,com))*(n/60);
endfunction

function Current = I_Motor(V,n,com)
	Current = (Imax(V,com) - com.Imin) * (1 - n/Nmax(V,com)) + com.Imin;
endfunction

function eta = eta_Motor(V,n,com)
	eta = P_Motor(V,n,com)/(V * I_Motor(V,n,com));
endfunction
