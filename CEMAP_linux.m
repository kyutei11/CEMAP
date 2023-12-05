#CEMAP: Caluculation script of an Electoric Model AirPlane

# solves nonlinear equation of the system that is connections of the each module:

clear

com.rho=1.2;
com.g=9.8;  # if you fly your plane in the atmosphere on the earth :-)

# Load airplane functions and properties: Use your own
solarplane

# Load battery functions and properties
NiMH_AAA

# Load solarcell functions and properties
solarcell

# Load motor  functions and properties
rs385ph

# Load controller functions and properties
controller

# Load reduction gear functions and properties
direct

# Load propeller functions and properties
cam9x5

com.m.gross = com.m.structure + com.m.cell*com.CellNo + com.m.gear + com.m.prop + com.m.motor;

function [resid, p] = solveParam(x,com)
# input parameter: CL,Powercontrol

    resid = zeros(length(x),1);

    p.Ibatt    = x(1);
    p.Nprop    = x(2);
    if com.input.static,
        p.Velo  = 0.01;
        PowCtrl = 1.0;
    else
        p.Velo     = x(3);
        p.gamma    = x(4);
        CL = com.input.CL;
        PowCtrl = com.input.Powercontrol;
    endif

    p.Vbatt  = Vbattery(p.Ibatt,com);
    p.Isolar = I_solarcell(p.Vbatt,com); # solarcell is connetced parallel to the battery

    p.Vmotor = (p.Vbatt + dVwire(p.Ibatt + p.Isolar,com) ) * PowCtrl;
    p.Nmotor = p.Nprop * com.t_spar / com.t_pinion;
    p.Imotor = I_Motor(p.Vmotor,p.Nmotor,com);
    p.Pmotor = P_Motor(p.Vmotor,p.Nmotor,com);
    p.Emotor = eta_Motor(p.Vmotor,p.Nmotor,com);

    p.Tprop  = T_Prop(p.Velo, p.Nprop, com);
    p.Eprop  = eta_Prop(p.Velo, p.Nprop, com);
    if p.Eprop<0,p.Eprop =0; endif

    resid(1) = p.Ibatt + p.Isolar - I_controlin(PowCtrl,p.Imotor,com);
    resid(2) = p.Pmotor*com.eta_gear - P_Prop(p.Velo,p.Nprop,com);

    if !com.input.static,
        [L D] = LD(p.Velo,CL,com);
        
        resid(3) = L*cos(p.gamma) - D*sin(p.gamma) - com.m.gross*com.g;
        resid(4) = p.Tprop - (D + com.m.gross*com.g*sin(p.gamma) );
    endif

endfunction

# get static parameters
com.input.static = 1;
x0 = [0.2 5000]';
[x_static itl] = fzeros('solveParam',x0,com);
[dum param_static] = solveParam(x_static,com);

# get MPP of solarcell
[I_mpp V_mpp P_mpp] = mpp(com);


# get parameter matrix
com.input.static = 0;

CL = com.CL_min:0.1:com.CL_max; nCL = length(CL);
PC = 0:0.1:1.0;          nPC = length(PC);
PC(1) = 1e-6;

Velocity    = zeros(nPC,nCL);# (m/sec)
Climbrate   = zeros(nPC,nCL);# (m/sec)
RPM         = zeros(nPC,nCL);
Omega       = zeros(nPC,nCL);# Normalized Angular Velocity of the motor
Thrust      = zeros(nPC,nCL);# (N)
TWratio     = zeros(nPC,nCL);
Power       = zeros(nPC,nCL);# (W)
BAR         = zeros(nPC,nCL);# Battery Absorption Rate(1/h)
SolarcellP  = zeros(nPC,nCL);# solar cell power normalised by the value at MPP
SolarcellV  = zeros(nPC,nCL);# solar cell voltage normalised by the value at MPP
eta_Motor   = zeros(nPC,nCL);
eta_Prop    = zeros(nPC,nCL);

x = [0.2 5000 5 0]';

for iPC = nPC:-1:1,
    for iCL = nCL:-1:1,
        com.input.CL = CL(iCL);
        com.input.Powercontrol = PC(iPC);
        
        [x itl] = fzeros('solveParam',x,com);

        if x==Inf,
            printf("Nonlinear solver did not converge ");
            printf("at CL=%f, PC=%f\n" ,PC(iPC),CL(iCL));
            return;
        endif
        [dum p] = solveParam(x,com);

          Velocity(iPC,iCL) = p.Velo;
         Climbrate(iPC,iCL) = p.Velo * sin(p.gamma);
               RPM(iPC,iCL) = p.Nmotor;
             Omega(iPC,iCL) = p.Nmotor/Nmax(p.Vmotor,com);
            Thrust(iPC,iCL) = p.Tprop;
           TWratio(iPC,iCL) = p.Tprop/com.m.gross/com.g;
             Power(iPC,iCL) = p.Pmotor;
               BAR(iPC,iCL) = p.Ibatt*3600/(com.Capa*3600);
        SolarcellP(iPC,iCL) = p.Vbatt*p.Isolar/P_mpp; # V_solarcell = V_battery
        SolarcellV(iPC,iCL) =         p.Vbatt /V_mpp; # V_solarcell = V_battery
         eta_Motor(iPC,iCL) = p.Emotor;
          eta_Prop(iPC,iCL) = p.Eprop;
    end
end


menu1 = 'Velocity(m/sec)';
menu2 = 'Climb rate(m/sec)';
menu3 = 'RPM';
menu4 = 'Normalized Angular Velocity';
menu5 = 'Thrust(N)'
menu6 = 'Thrust to Weight Ratio';
menu7 = 'Motor Power Output(W)';
menu8 = 'Battery Absorption Rate(1/h)';
menu9 = 'Solarcell power normalised by the value at MPP';
menu10= 'Solarcell voltage normalised by the value at MPP';
menu11= 'Motor efficiency';
menu12= 'Propeller efficiency';
menu13= 'Static parameters';

goforit=1;
plotsect = 1;

multiplot(1,2);

while goforit
	switch menu('Which data do you want?'...
            ,menu1,menu2,menu3,menu4,menu5,menu6,menu7,menu8,menu9,menu10,menu11,menu12,menu13,'Exit CEMAP')
		case 1
			plotsect = cleargraph(menu1,plotsect,PC,CL); contour(PC,CL,Velocity,10);
		case 2
			plotsect = cleargraph(menu2,plotsect,PC,CL); contour(PC,CL,Climbrate,16);
		case 3
			plotsect = cleargraph(menu3,plotsect,PC,CL); contour(PC,CL,RPM,16);
		case 4
			plotsect = cleargraph(menu4,plotsect,PC,CL); contour(PC,CL,Omega,16);
		case 5
			plotsect = cleargraph(menu5,plotsect,PC,CL); contour(PC,CL,Thrust,10);
		case 6
			plotsect = cleargraph(menu6,plotsect,PC,CL); contour(PC,CL,TWratio,10);
		case 7
			plotsect = cleargraph(menu7,plotsect,PC,CL); contour(PC,CL,Power,10);
		case 8
			plotsect = cleargraph(menu8,plotsect,PC,CL); contour(PC,CL,BAR,16);
		case 9
			plotsect = cleargraph(menu9,plotsect,PC,CL); contour(PC,CL,SolarcellP,10);
		case 10
			plotsect = cleargraph(menu10,plotsect,PC,CL); contour(PC,CL,SolarcellV,10);
		case 11
			plotsect = cleargraph(menu11,plotsect,PC,CL); contour(PC,CL,eta_Motor,16);
		case 12
			plotsect = cleargraph(menu12,plotsect,PC,CL); contour(PC,CL,eta_Prop,10);
		case 13
			param_static
    	otherwise
    		goforit = 0;
	endswitch
endwhile

oneplot();
