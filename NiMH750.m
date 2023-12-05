#Battery parameters: 750mAh NiMH

com.CellNo = 9;
com.R.battery = 0.05*com.CellNo;
com.m.cell = 0.01;
com.V.cell = 1.25;
com.Capa = 0.750;

function V = Vbattery(I,com)
  V = com.V.cell*com.CellNo - I*com.R.battery;
endfunction

