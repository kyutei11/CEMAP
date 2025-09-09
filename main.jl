# main.jl
using JSON
using NLsolve
using Plots
using LinearAlgebra

# Include module files
include("modules/Common.jl")
include("modules/SolarPlane.jl")
include("modules/Battery.jl")
include("modules/SolarCell.jl")
include("modules/Motor.jl")
include("modules/Controller.jl")
include("modules/Gear.jl")
include("modules/Propeller.jl")
include("utils/PlotUtils.jl")

using .CommonModule
using .SolarPlaneModule
using .BatteryModule
using .SolarCellModule
using .MotorModule
using .ControllerModule
using .GearModule
using .PropellerModule
using .PlotUtils

# ファイル読み込みとデータ準備
config_file = "inputs/config.json"
if !isfile(config_file)
    error("Config file not found: $config_file")
end
config = JSON.parsefile(config_file)

# 各コンポーネントのデータを読み込む関数
function load_component_data(filename::String, component_name::String)
    full_path = "inputs/$filename"
    if !isfile(full_path)
        error("$component_name config file not found: $full_path")
    end
    config = JSON.parsefile(full_path)
    if !(config isa Dict)
        error("$component_name config must be a JSON object (dictionary), got: $(typeof(config))")
    end
    return haskey(config, component_name) ? config[component_name] : config
end

# データを読み込んでコンポーネントを初期化
common_data = load_component_data(config["common"], "Common")
solarplane_data = load_component_data(config["solarplane"], "SolarPlane")
battery_data = load_component_data(config["battery"], "Battery")
solarcell_data = load_component_data(config["solarcell"], "SolarCell")
motor_data = load_component_data(config["motor"], "Motor")
controller_data = load_component_data(config["controller"], "Controller")
gear_data = load_component_data(config["gear"], "Gear")
propeller_data = load_component_data(config["propeller"], "Propeller")

# ログ出力
println("Using common file: $(config["common"])")
println("Using solarplane file: $(config["solarplane"])")
println("Using battery file: $(config["battery"])")
println("Using solarcell file: $(config["solarcell"])")
println("Using motor file: $(config["motor"])")
println("Using controller file: $(config["controller"])")
println("Using gear file: $(config["gear"])")
println("Using propeller file: $(config["propeller"])")

# コンポーネントの初期化
common = Common(common_data)
solarplane = SolarPlane(solarplane_data)
battery = Battery(battery_data)
solarcell = SolarCell(solarcell_data)
motor = Motor(motor_data)
controller = Controller(controller_data)
gear = Gear(gear_data)
propeller = Propeller(propeller_data)

# Common構造体に追加のパラメータを設定
common = Common(
                common.rho,
                common.g,
                solarplane.m_wing + solarplane.m_empn+solarplane.m_mech +
                battery.m_cell * battery.CellNo +
                solarcell.m_cell * solarcell.cascade*solarcell.parallel +
                gear.m_gear + propeller.m_prop + motor.m_motor,
                true,  # input_static
                0.0,   # input_CL
                0.0    # input_Powercontrol
                )

# 以下、提供されたコードをそのまま統合（一部修正あり）

# Solve nonlinear system
function solveParam(x, common, solarplane, battery, solarcell, motor, controller, gear, propeller)
    resid = zeros(length(x))
    p = Dict{String, Float64}()

    p["Ibatt"] = x[1]
    p["Nprop"] = x[2]
    if common.input_static
        p["Velo"] = 0.01
        PowCtrl = 1.0
    else
        p["Velo"] = x[3]
        p["gamma"] = x[4]
        CL = common.input_CL
        PowCtrl = common.input_Powercontrol
    end

    p["Vbatt"] = Vbattery(p["Ibatt"], battery)
    p["Isolar"] = I_solarcell(p["Vbatt"], solarcell)
    p["Vmotor"] = (p["Vbatt"] + dVwire(p["Ibatt"] + p["Isolar"], controller)) * PowCtrl
    p["Nmotor"] = p["Nprop"] * gear.t_spar / gear.t_pinion
    p["Imotor"] = I_Motor(p["Vmotor"], p["Nmotor"], motor)
    p["Pmotor"] = P_Motor(p["Vmotor"], p["Nmotor"], motor)
    p["Emotor"] = MotorModule.eta_Motor(p["Vmotor"], p["Nmotor"], motor)

    p["Tprop"] = T_Prop(p["Velo"], p["Nprop"], propeller, common)
    p["Eprop"] = PropellerModule.eta_Prop(p["Velo"], p["Nprop"], propeller)
    if p["Eprop"] < 0
        p["Eprop"] = 0
    end

    resid[1] = p["Ibatt"] + p["Isolar"] - I_controlin(PowCtrl, p["Imotor"], controller)
    resid[2] = p["Pmotor"] * gear.eta_gear - P_Prop(p["Velo"], p["Nprop"], propeller, common)

    if !common.input_static
        L, D = LD(p["Velo"], CL, solarplane, common)
        resid[3] = L * cos(p["gamma"]) - D * sin(p["gamma"]) - common.m_gross * common.g
        resid[4] = p["Tprop"] - (D + common.m_gross * common.g * sin(p["gamma"]))
    end

    return resid, p
end

# Solve static parameters
static_common = Common(common.rho, common.g, common.m_gross, true, common.input_CL, common.input_Powercontrol)
x_static_init = [0.2, 5000.0]
sol_static = nlsolve((out, x) -> out .= solveParam(x, static_common, solarplane, battery, solarcell, motor, controller, gear, propeller)[1], x_static_init, ftol=1e-6)
if !sol_static.f_converged
    error("Nonlinear solver did not converge for static parameters")
end
x_static = sol_static.zero
_, param_static = solveParam(x_static, static_common, solarplane, battery, solarcell, motor, controller, gear, propeller)

# Get MPP of solarcell
sol_mpp = nlsolve((out, x) -> resid_mpp!(out, x, solarcell), [1.0, 1.0, 1.0], ftol=1e-9)
if !sol_mpp.f_converged
    error("Nonlinear solver did not converge for MPP")
end
I_mpp, V_mpp, P_mpp = let param = sol_mpp.zero
    I = param[1] * solarcell.Isc * solarcell.parallel
    V = param[2] * solarcell.Voc * solarcell.cascade - 0.6
    P = I * V
    (I, V, P)
end

# Parameter matrix
CL = collect(solarplane.CL_min:0.1:solarplane.CL_max)  # collectを追加して配列に変換
PC = collect(0.0:0.1:1.0)  # 型をFloat64に統一
PC[1] = 1e-6
nCL = length(CL)
nPC = length(PC)

Velocity = zeros(nPC, nCL)
Climbrate = zeros(nPC, nCL)
RPM = zeros(nPC, nCL)
Omega = zeros(nPC, nCL)
Thrust = zeros(nPC, nCL)
TWratio = zeros(nPC, nCL)
motor_power = zeros(nPC, nCL)
BAR = zeros(nPC, nCL)
SolarcellP = zeros(nPC, nCL)
SolarcellV = zeros(nPC, nCL)
motor_efficiency = zeros(nPC, nCL)
prop_efficiency = zeros(nPC, nCL)

# Compute parameter matrices
let x = [0.2, 5000.0, 5.0, 0.0]  # Initial guess for the first iteration
    for iPC = nPC:-1:1
        for iCL = nCL:-1:1
            # Create a new Common instance for this iteration
            loop_common = Common(common.rho, common.g, common.m_gross, false, CL[iCL], PC[iPC])

            # Solve nonlinear system using current x
            sol = nlsolve((out, x) -> out .= solveParam(x, loop_common, solarplane, battery, solarcell, motor, controller, gear, propeller)[1], x, ftol=1e-6)
            if !sol.f_converged
                println("Nonlinear solver did not converge at CL=$(CL[iCL]), PC=$(PC[iPC])")
                continue  # 収束しない場合はスキップ
            end
            # Update initial guess for the next iteration
            x = sol.zero
            _, p = solveParam(x, loop_common, solarplane, battery, solarcell, motor, controller, gear, propeller)

            Velocity[iPC, iCL] = p["Velo"]
            Climbrate[iPC, iCL] = p["Velo"] * sin(p["gamma"])
            RPM[iPC, iCL] = p["Nmotor"]
            Omega[iPC, iCL] = p["Nmotor"] / Nmax(p["Vmotor"], motor)
            Thrust[iPC, iCL] = p["Tprop"]
            TWratio[iPC, iCL] = p["Tprop"] / (common.m_gross * common.g)
            motor_power[iPC, iCL] = p["Pmotor"]
            BAR[iPC, iCL] = p["Ibatt"] * 3600 / (battery.Capa * 3600)
            SolarcellP[iPC, iCL] = p["Vbatt"] * p["Isolar"] / P_mpp
            SolarcellV[iPC, iCL] = p["Vbatt"] / V_mpp
            motor_efficiency[iPC, iCL] = p["Emotor"]
            prop_efficiency[iPC, iCL] = p["Eprop"]
        end
    end
end


# Interactive menu for plotting
function interactive_plot(PC, CL, param_static)
  menu_options = [
                  ("Velocity (m/sec)", Velocity, 10),
                  ("Climb rate (m/sec)", Climbrate, 16),
                  ("RPM", RPM, 16),
                  ("Normalized Angular Velocity", Omega, 16),
                  ("Thrust (N)", Thrust, 10),
                  ("Thrust to Weight Ratio", TWratio, 10),
                  ("Motor Power Output (W)", motor_power, 10),
                  ("Battery Absorption Rate (1/h)", BAR, 16),
                  ("Solarcell power normalised by MPP", SolarcellP, 10),
                  ("Solarcell voltage normalised by MPP", SolarcellV, 10),
                  ("Motor efficiency", motor_efficiency, 16),
                  ("Propeller efficiency", prop_efficiency, 10),
                  ("Static parameters", nothing, 0),
                  ("Exit CEMAP", nothing, 0)
                  ]

  goforit = true

  while goforit
    println("\nWhich data do you want to plot?")
    for (i, (title, _, _)) in enumerate(menu_options)
      println("$i: $title")
    end
    print("Enter choice (1-$(length(menu_options))): ")
    choice = try
      parse(Int, readline())
    catch
      println("Invalid input, please enter a number.")
      continue
    end

    if choice < 1 || choice > length(menu_options)
      println("Invalid choice, please try again.")
      continue
    end

    title, data, levels = menu_options[choice]

    if title == "Exit CEMAP"
      goforit = false
      println("Exiting CEMAP. Farewell!")
    elseif title == "Static parameters"
      println("Static parameters:")
      for (key, value) in param_static
        println("$key: $value")
      end
    else
      # Create a single plot
      plt = plot()
      plot_contour!(plt, PC, CL, data, title, levels)
      display(plt)
    end
  end
end

# Run interactive plotting
interactive_plot(PC, CL, param_static)
