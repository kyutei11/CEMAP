module MotorModule
export Motor, Imax, Nmax, Tmax, P_Motor, I_Motor, eta_Motor

struct Motor
m_motor::Float64
R_motor::Float64
Kv::Float64
Imin::Float64
end

function Motor(data::Dict)
  required_fields = ["m_motor", "R_motor", "Kv", "Imin"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in Motor data")
    end
  end

  Motor(
        Float64(data["m_motor"]),
        Float64(data["R_motor"]),
        Float64(data["Kv"]),
        Float64(data["Imin"])
        )
end

# parameters related to maximum voltage

function Imax(V, motor::Motor)
  return V / motor.R_motor
end

function Nmax(V, motor::Motor)
  return motor.Kv * V
end

function Tmax(V, motor::Motor)
  return V * (Imax(V, motor) - motor.Imin) / (Nmax(V, motor) / 60)
end

# parameters related to voltage and angular velocity
function P_Motor(V, n, motor::Motor)
  return Tmax(V, motor) * (1.0 - n / Nmax(V, motor)) * (n / 60)
end

function I_Motor(V, n, motor::Motor)
  return (Imax(V, motor) - motor.Imin) * (1.0 - n / Nmax(V, motor)) + motor.Imin
end

function eta_Motor(V, n, motor::Motor)
  return P_Motor(V, n, motor) / (V * I_Motor(V, n, motor))
end
end # module
