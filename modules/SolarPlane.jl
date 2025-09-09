module SolarPlaneModule
export SolarPlane, LD, CD

struct SolarPlane
m_wing::Float64
m_empn::Float64
m_mech::Float64
b::Float64
S::Float64
AR::Float64
A_fuse::Float64
Cd0_wing::Float64
Cdp_fuse::Float64
CL_min::Float64
CL_max::Float64
end

function SolarPlane(data::Dict)
  required_fields = ["m_wing",
                     "m_empn", # fuse+tail
                     "m_mech",
                     "b", "S", "A_fuse",
                     "Cd0_wing", "Cdp_fuse", "CL_min", "CL_max"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in SolarPlane data")
    end
  end

  AR = data["b"]^2 / data["S"]
  SolarPlane(
             Float64(data["m_wing"]),
             Float64(data["m_empn"]),
             Float64(data["m_mech"]),
             Float64(data["b"]),
             Float64(data["S"]),
             AR,
             Float64(data["A_fuse"]),
             Float64(data["Cd0_wing"]),
             Float64(data["Cdp_fuse"]),
             Float64(data["CL_min"]),
             Float64(data["CL_max"])
             )
end

function CD(CL, solarplane::SolarPlane)
  #                            2Dwing                  CDi
  return solarplane.Cd0_wing + 0.015 * (CL - 0.25)^2 + CL^2 / (0.9 * pi * solarplane.AR) + solarplane.Cdp_fuse * solarplane.A_fuse / solarplane.S
end

function LD(Vc, CL, solarplane::SolarPlane, common)
  q = 0.5 * common.rho * Vc^2
  L = q * CL * solarplane.S
  D = q * CD(CL, solarplane) * solarplane.S
  return L, D
end

end # module
