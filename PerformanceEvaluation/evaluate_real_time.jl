# [CubeSatBox/PerformanceEvaluation/evaluate.jl]

"""
     Contains the code intended to be run on the laptop during evaluation 
    of the sun vector estimation, in conjunction with `evaluate.py`. This 
    version runs in real time and displays the error, which is helpful for 
    debugging
"""

using StaticArrays, SatelliteDynamics, EarthAlbedo 
using Distributions, LinearAlgebra, Plots, JLD2 
using LibSerialPort


path = "../SensorCalibration/src/MissionSim/"
include(path * "mag_field.jl");     # Contains IGRF13 stuff 
include(path * "quaternions.jl")

include(path * "CustomStructs.jl");         using .CustomStructs
include(path * "Simulator/Simulator.jl");   using .Simulator
include(path * "Estimator/Estimator.jl");   using .Estimator 
include(path * "Controller/Controller.jl"); using .Controller

include("../Testbox_Interface.jl");         using .Interface


function get_initial_state(; _Re = 6378136.3, detumbled = false, bias_less = false) 
    ecc = 0.0001717 + 0.00001 * randn()
    inc = 51.6426 + randn()
    Ω   = 178.1369 + randn()
    ω   = 174.7410 + randn()
    M   = 330.7918 + 100 + randn()   # +94/95 is just before sun, -40 is just before eclipse
    sma = (_Re + 421e3) / (1 + ecc)  # Apogee = semi_major * (1 + ecc)

    oe0 = [sma, ecc, inc, Ω, ω, M]   # Initial state, oscullating elements
    eci0 = sOSCtoCART(oe0, use_degrees = true) # Convert to Cartesean

    r₀ = SVector{3, Float64}(eci0[1:3])
    v₀ = SVector{3, Float64}(eci0[4:6])
    q₀ = randn(4);  q₀ = SVector{4, Float64}(q₀ / norm(q₀))
    ω₀ = (detumbled) ? SVector{3, Float64}(0.05 * randn(3)) : SVector{3, Float64}(0.5 * randn(3))
    β₀ = (bias_less) ? SVector{3, Float64}(0.0, 0.0, 0.0)  : SVector{3, Float64}(rand(Normal(0.0, deg2rad(2.0)), 3))
    
    T_orbit = orbit_period(oe0[1])
    x = STATE(r₀, v₀, q₀, ω₀, β₀)

    return x, T_orbit 
end

function get_albedo(scale = 1) 

    function load_refl(path = "data/refl.jld2", scale = 1)
        temp = load(path)
    
        refl = REFL( temp["data"][1:scale:end, 1:scale:end], temp["type"], temp["start_time"], temp["stop_time"])
    
        return refl
    end
    lat_step = 1.0 * scale
    lon_step = 1.25 * scale

    refl = load_refl(path * "data/refl.jld2", scale)  
    cell_centers_ecef = get_albedo_cell_centers(lat_step, lon_step) 
    return Simulator.ALBEDO(refl, cell_centers_ecef)
end;

function get_albedo_cell_centers(lat_step = 1, lon_step = 1.25)
    """
        Returns the cell centers for the grid covering the surface of the Earth in Cartesian ECEF, to be used in later estimations of Earth's albedo,
            by looping through each cell's LLA coordinate and converting to ECEF 

        Arguments:
        - lat_step: (Optional) The step size (in degrees) to take across the latitude domain. Defaults to 1*        | Scalar 
        - lon_step: (Optional) The step size (in degrees) to take across the longitude domain. Defaults to 1.25*    | Scalar

        Returns:
        - cells_ecef: Matrix containing [x,y,z] coordinate for each latitude, longitude point.
                        Of form [lat, lon, [x,y,z]]                                                                 | [num_lat x num_lon x 3]
    """
    alt = 0.0 # Assume all cells are on surface of earth
    num_lat = Int(round((180 - lat_step) / lat_step) + 1)
    num_lon = Int(round((360 - lon_step) / lon_step) + 1)

    lon_offset = lon_step + (360 - lon_step) / 2   # Centers at 0 (longitude: [1.25, 360] => [-179.375, 179.375])
    lat_offset = lat_step + (180 - lat_step) / 2   # Centers at 0 (latitude:  [1.00, 180] => [-89.5, 89.5])

    cells_ecef = zeros(num_lat, num_lon, 3) # Lat, Lon, [x,y,z]
    for lat = 1:num_lat 
        for lon = 1:num_lon
            geod = [(lon * lon_step - lon_offset), (lat * lat_step - lat_offset), alt]
            ecef = sGEODtoECEF(geod, use_degrees = true)

            cells_ecef[Int(lat), Int(lon), :] = ecef
        end
    end

    return cells_ecef 
end;


# NOTE: Pull the estimated version from path * state_machine.jl
function estimate_sun_vector(Ĩ, diodes::DIODES{N, T}) where {N, T}
    """ Simple method for estimating the sun vector that converts each surface normal into cartesian coordinates 
    and combines terms to get the strenght along each axis. NOT final version. """

    sph2cart(α, ϵ, ρ) = [ρ * sin(pi/2 - ϵ)*cos(α); ρ * sin(pi/2 - ϵ) * sin(α); ρ * cos(pi/2 - ϵ)]
    
    sx, sy, sz = 0.0, 0.0, 0.0

    for i = 1:6 
        d = Ĩ[i] / diodes.calib_values[i]
        x, y, z = sph2cart(diodes.azi_angles[i], diodes.elev_angles[i], d)
        sx += x 
        sy += y
        sz += z
    end

    ŝᴮ = [sx, sy, sz]
    return SVector{3, T}(round.(ŝᴮ / norm(ŝᴮ), digits = 3))
end

function get_err(v₁, v₂) 
    """ Gets the angle between two vectors """

    v₁ /= norm(v₁)
    v₂ /= norm(v₂)
    dp = v₁' * v₂ 
    inner = ( (dp >  1.0) && (dp ≈  1.0)) ?  1.0 :          # Deal with numerical errors
            ( (dp < -1.0) && (dp ≈ -1.0)) ? -1.0 : dp 

    err = rad2deg(acos(inner))
    return err
end


function run_sun(mcu_port::SerialPort, sat_port::SerialPort; N = 100)
    """
          Generates a random orbit, and then updates it over time. 
        Uses the updated state to generate the body and inertial frame sun 
        vectors, and updates the CubeSat test box accordingly. Reads in the 
        estimated sun vector from the satellite and compares it to the true 
        value to evaluate the performance. 
    """

    @info "Starting `run sun` demo"
    
    # Set up sat, user strings 
    sat_msg = ""  # String input from the CubeSat 
    usr_msg = ""  # String input from the user (Used for exiting out of the demo)

    # Initialize state, parameters, storage arrays (TODO - make it more random!)
    x₀, _ = get_initial_state(; detumbled = true, bias_less = true)
    q₀ = [1, 0, 0, 0]  # +X 
    # q₀ = [0, 0, 0, 1]  # -X 
    # q₀ = [1, 0, 0, 1]  # +Y 
    # q₀ = [1, 0, 0, -1] # -Y 
    # q₀ = [1, 0, 1, 0]  # +Z 
    # q₀ = [1  0, -1, 0] # -Z
    q₀ = SVector{4, Float64}(q₀ / norm(q₀))
    x₀ = STATE(x₀.r, x₀.v, x₀.q, SVector{3, Float64}(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)), x₀.β)
    J  = SMatrix{3, 3, Float64, 9}([1.0 0 0; 0 1 0; 0 0 1])
    t₀  = Epoch(2020, 1, 1)
    u  = SVector{3, Float64}(0.0, 0.0, 0.0)
    dt = 1.0

    diodes = DIODES( SVector{6, Float64}(ones(6)),    SVector{6, Float64}(0.0,  pi, pi/2, -pi/2,  0.0,   0.0),    SVector{6, Float64}(0.0, 0.0,  0.0,   0.0, pi/2, -pi/2)   )
    # diodes = DIODES(; ideal = true);   # Uncalibrated!

    errs = zeros(N)
    times = zeros(N)

    sᴵ, sᴮ = zeros(3), zeros(3)
    x = x₀; t = t₀
    for i = 1:N 
        (i % 50 == 0) && @show i

        # Update State 
        x = rk4(J, x, u, t, dt; σβ = 0.0);
        ᴮQᴵ = (H' * L(x.q) * R(x.q)' * H)'
        sᴵ .= sun_position(t) - x.r
        if (i < 15) 
            sᴮ .= [1, 0, 0]
        elseif (i < 30)
            sᴮ .= [-1, 0, 0]
        elseif (i < 45)
            sᴮ .= [0, 1, 0]
        elseif (i < 60)
            sᴮ .= [0, -1, 0]
        elseif (i < 75)
            sᴮ .= [0, 0, 1]
        else 
            sᴮ .= [0, 0, -1]
        end

        sᴮ /= norm(sᴮ)   # Make it unit
        

        # Send vector to lights 
        px, py, pz = round.(sᴮ, digits = 3) 
        msg = "[$px, $py, $pz]"  # Match the desired string format
        write(mcu_port, "$msg")        # Send position to Arduino
    
        @async sat_msg *= String(nonblocking_read(sat_port)) 
        @async usr_msg *= readline(keep = true)

        # Get estimate 
        if (occursin("\n", sat_msg))
            lines = split(sat_msg, "\n") # Split into lines 
            while length(lines) > 1 
                line = popfirst!(lines)

                # Verify it is an array and not an error message 
                if line[1] == '['
                    Ĩ = str2vec(line) 
                    est = estimate_sun_vector(Ĩ, diodes)
                    # if (norm(est) ≉ 1.0) || (norm(sᴮ) ≉ 1.0)   # Not unit because of rounding 
                    #     @warn "Warning! Vectors not unit!"
                    # end

                    err = get_err(sᴮ, est)
                    errs[i] = err
                    times[i] = t - t₀
                    println("\tTrue: $msg \tEst: $est  \tErr (deg): $err")
                    println("\t$Ĩ")
                else 
                    println("\t $line")
                end
            end
            sat_msg = "" 
        end 
        

        occursin("\e", usr_msg) && break
        # Compare to truth 

        t += dt
        sleep(0.5)
    end

    @info "Closing simulation!"

    times = times[errs .!= 0.0]
    errs  = errs[ errs .!= 0.0]
    display(plot(times, errs))

    μ = mean(errs)
    σ = std( errs)

    println("Statistics:")
    println("Mean: $μ  \t St Dev:  $σ")

end

console_interface(run_sun)
 