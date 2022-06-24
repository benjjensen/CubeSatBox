# [CubeSatBox/PerformanceEvaluation/simulate_sun.jl]

"""
     Contains the code intended to be run on the laptop during evaluation 
    of the sun vector estimation, in conjunction with `evaluate.py`
"""

###############
# SETUP STUFF #
###############

using StaticArrays, SatelliteDynamics, EarthAlbedo 
using Distributions, LinearAlgebra, Plots, JLD2 
using LibSerialPort


path = "../../SensorCalibration/src/MissionSim/"
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

    refl = load_refl("data/refl.jld2", scale)  
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

################
# Actual stuff #
################

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

function run_sun(mcu_port::SerialPort, sat_port::SerialPort; N = 500)
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
    x₀ = STATE(x₀.r, x₀.v, q₀, x₀.ω, x₀.β) # SVector{3, Float64}(deg2rad(10.0), deg2rad(0.0), deg2rad(0.0)), x₀.β)
    J  = SMatrix{3, 3, Float64, 9}([1.0 0 0; 0 0.9 0; 0 0 1.1])

    u  = SVector{3, Float64}(0.0, 0.0, 0.0)
    dt = 0.2

    t₀ = Epoch(2020, 1, 1)
    sᴮs   = zeros(N, 3)
    Ĩs    = zeros(N, 6)
    ts    = [t₀ for i = 1:N]    # Times for the SATELLITE output
    sᴮ = zeros(3)
    t = t₀
    x = x₀
    xs = [x₀ for i = 1:N]

    # Clear out the buffer
    @async sat_msg *= String(nonblocking_read(sat_port)) 
    yield()
    sleep(0.25)
    sat_mst = ""  
    for i = 1:N 

        # Update State 
        x = rk4(J, x, u, t, dt; σβ = 0.0);
       
        # Update Sun vector & send to arduino
        ᴮQᴵ = (H' * L(x.q) * ts(x.q)' * H)'
        sᴵ = sun_position(t); sᴵ /= norm(sᴵ)
        sᴮ .= ᴮQᴵ * sᴵ
        sᴮ /= norm(sᴮ)      # Should already be unit, but...

        px, py, pz = round.(sᴮ, digits = 3) 
        msg = "[$px, $py, $pz]"    # Match the desired string format
        write(mcu_port, "$msg")    # Send position to Arduino
    

        # Read in measurements 
        @async sat_msg *= String(nonblocking_read(sat_port)) 
        sleep(dt)

        # Process data and store
        if (occursin("\n", sat_msg))
            lines = split(sat_msg, "\n") # Split into lines 
            while length(lines) > 1 
                line = popfirst!(lines)

                # Verify it is an array and not an error message 
                if line[1] == '['
                    Ĩ = str2vec(line) 
                    Ĩs[i, :] .= Ĩ 
                    sᴮs[i, :] .= sᴮ
                    xs[i] = x
                    ts[i] = t 


                    println("\tI: $(round.(Ĩ, digits = 3))")
                else 
                    println("\t $line")
                end
            end
            sat_msg = "" 
        end 
        

        occursin("\e", usr_msg) && break
        t += dt
    end

    @info "Closing simulation!"
    
    # @save "data.jld2" Ĩs=Ĩs sᴮs=sᴮs xs=xs ts=ts dt=dt
end


console_interface()
