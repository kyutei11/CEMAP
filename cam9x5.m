# Prop param.
com.m.prop = 0.03;

com.Dprop = 9*0.0254;
com.Pprop = 5*0.0254;

com.etamax_Prop = 0.7;

com.J0 = 1.35*com.Pprop/com.Dprop; # Advance ratio where CT equals to zero

function J = J_Prop(Velo,n,com)
	J = Velo/(com.Dprop*n/60);
endfunction

function eta = eta_Prop(Velo,n,com)
	J = J_Prop(Velo,n,com);
	eta = com.etamax_Prop*(J/com.J0)*(1-(J/com.J0)**2)*(3*3**0.5)/2;
endfunction

function CT = CT_Prop(Velo,n,com)
	J = J_Prop(Velo,n,com);
	if J<com.J0/3,
		CT = 0.16*com.Pprop/com.Dprop;
	else
		CT = 0.16*(com.Pprop/com.Dprop)*(1-(J-com.J0/3)/(com.J0*2/3));
	end
endfunction

function T = T_Prop(Velo,n,com)
	T = CT_Prop(Velo,n,com) * com.rho * (n/60)**2 * com.Dprop**4;
endfunction

function P = P_Prop(Velo,n,com) # required power input to the prop.
    P = Velo * T_Prop(Velo,n,com) / eta_Prop(Velo,n,com);
endfunction
