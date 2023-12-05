# Parameters of the Airplane

#Mass
                  # wing  fuse/tail  solarcell  radio
com.m.structure =   0.1  +  0.05    +  0.07    + 0.09;

#Geometory
com.b = 2.4;
com.S = 0.4;
com.AR = com.b**2/com.S;
com.A_fuse = 0.05*0.05;

#Aerodynamic

com.Cd0_wing = 0.02;
com.Cdp_fuse = 0.1;
com.CL_min = 0.3;
com.CL_max = 1.2;

function CD = CD(CL,com)
         #  CL_2Dwing                       # 3D property
	CD = com.Cd0_wing + 0.015*(CL-0.25)**2 + CL**2/(0.9*pi*com.AR) + com.Cdp_fuse*com.A_fuse/com.S;
endfunction

function [L D] = LD(Vc,CL,com)
    q = 0.5*com.rho*Vc**2;
    L = q*CL*com.S;
    D = q*CD(CL,com)*com.S;
endfunction
