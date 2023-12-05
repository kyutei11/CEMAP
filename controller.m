# parameters of connection wire and power controller
com.R.wire = 0.005;
com.R.connector = 0.01 + 0.01;
com.eta.controller = 0.95;

function dVwire = dVwire(I,com)
  dVwire = -I*(com.R.wire + com.R.connector);
endfunction

# Vin*Iin*eta_controller = Vout*Iout, PowCtrl = Vout/Vin
# I set Iin as an unknown parameter because other parameters do not take the value of inf
# in the case of PowCtrl == 0
function Iin = I_controlin(PowCtrl,Iout,com)
  Iin  = PowCtrl * Iout / com.eta.controller;
endfunction
