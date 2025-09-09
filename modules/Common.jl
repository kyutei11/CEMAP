module CommonModule
export Common

struct Common
rho::Float64
g::Float64
m_gross::Float64
input_static::Bool
input_CL::Float64
input_Powercontrol::Float64
end

function Common(data::Dict)
  required_fields = ["rho", "g"]
  for field in required_fields
    if !haskey(data, field)
      error("Missing required field '$field' in Common data")
    end
  end

  Common(
         Float64(data["rho"]),
         Float64(data["g"]),
         0.0,  # m_grossは後で設定
         true,
         0.0,
         0.0
         )
end

end
