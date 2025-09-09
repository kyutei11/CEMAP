module PropellerModule
export Propeller, J_Prop, eta_Prop, CT_Prop, T_Prop, P_Prop

struct Propeller
m_prop::Float64
Dprop::Float64
Pprop::Float64
etamax_Prop::Float64
J0::Float64
end

function Propeller(data::Dict)
  required_fields = ["m_prop", "Dprop", "Pprop", "etamax_Prop"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in Propeller data")
    end
  end

  J0 = 1.35 * data["Pprop"] / data["Dprop"]
  Propeller(
            Float64(data["m_prop"]),
            Float64(data["Dprop"]),
            Float64(data["Pprop"]),
            Float64(data["etamax_Prop"]),
            J0
            )
end

function J_Prop(Velo, n, propeller::Propeller)
  return Velo / (propeller.Dprop * n / 60.0)
end

function eta_Prop(Velo, n, propeller::Propeller)
  J = J_Prop(Velo, n, propeller)
  return propeller.etamax_Prop * (J / propeller.J0) * (1 - (J / propeller.J0)^2) * (3*sqrt(3)) / 2
end

function CT_Prop(Velo, n, propeller::Propeller)
  J = J_Prop(Velo, n, propeller)
  if J < propeller.J0 / 3
    return 0.16 * propeller.Pprop / propeller.Dprop
  else
    return 0.16 * (propeller.Pprop / propeller.Dprop) * (1 - (J - propeller.J0 / 3) / (propeller.J0 * 2 / 3))
  end
end

function T_Prop(Velo, n, propeller::Propeller, common)
  return CT_Prop(Velo, n, propeller) * common.rho * (n / 60)^2 * propeller.Dprop^4
end

function P_Prop(Velo, n, propeller::Propeller, common)
  return Velo * T_Prop(Velo, n, propeller, common) / eta_Prop(Velo, n, propeller)
end
end # module
