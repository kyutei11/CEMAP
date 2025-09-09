module BatteryModule
export Battery, Vbattery

struct Battery
CellNo::Int
R_cell::Float64
m_cell::Float64
V_cell::Float64
Capa::Float64
end

function Battery(data::Dict)
  required_fields = ["CellNo", "R_cell", "m_cell", "V_cell", "Capa"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in Battery data")
    end
  end

  Battery(
          Int(data["CellNo"]),
          Float64(data["R_cell"]),
          Float64(data["m_cell"]),
          Float64(data["V_cell"]),
          Float64(data["Capa"])
          )
end

function Vbattery(I, bat::Battery)
  return (bat.V_cell - I * bat.R_cell) * bat.CellNo
end

end # module
