module ControllerModule
export Controller, dVwire, I_controlin

struct Controller
R_wire::Float64
R_connector::Float64
eta_controller::Float64
end

function Controller(data::Dict)
  required_fields = ["R_wire", "R_connector", "eta_controller"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in Controller data")
    end
  end

  Controller(
             Float64(data["R_wire"]),
             Float64(data["R_connector"]),
             Float64(data["eta_controller"])
             )
end

function dVwire(I, controller::Controller)
  return -I * (controller.R_wire + controller.R_connector)
end

# Vin*Iin*eta_controller = Vout*Iout, PowCtrl = Vout/Vin
# I set Iin as an unknown parameter because other parameters do not take the value of inf
# in the case of PowCtrl == 0
function I_controlin(PowCtrl, Iout, controller::Controller)
  return PowCtrl * Iout / controller.eta_controller
end
end # module

