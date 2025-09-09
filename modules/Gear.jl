module GearModule
export Gear

struct Gear
m_gear::Float64
eta_gear::Float64
t_spar::Float64
t_pinion::Float64
end

function Gear(data::Dict)
  required_fields = ["m_gear", "eta_gear", "t_spar", "t_pinion"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in Gear data")
    end
  end

  Gear(
       Float64(data["m_gear"]),
       Float64(data["eta_gear"]),
       Float64(data["t_spar"]),
       Float64(data["t_pinion"])
       )
end
end # module
