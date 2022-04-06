# [ArduinoComm/LightDemo/main.jl]

#   Series of demos involving Julia commanding LED's through an Arduino.
# Includes examples with Albedo accounted for, with orbiting, and possibly 
# more if I get around to it.  

@info "Starting"

using LibSerialPort     # Allows for Serial Communication with Arduino 
using LinearAlgebra, Plots 
using SatelliteDynamics # Allows for eclipse function, may not work on Windows


function console_interface(port = "/dev/ttyACM0", baud_rate = 115200)
    """
        Interface function that opens serial communication and starts light demo 
    """

    mcu = open(port, baud_rate)  # Open up connection to microcontroller 

    return simple_light_demo(mcu)  # Simulates simple rotating sat for a few seconds, no albedo or orbit 
end


function simple_light_demo(sp::SerialPort; max_time = 20)
    """
          Simulates a satellite for a few seconds and updates the testbox 
        LEDS to match accordingly. Note that the Earth's Albedo is NOT included here, nor 
        is the satellite actually orbiting; it is just tumbling in a fixed spot. 
        
        
    """

    
    @info "Starting Simple Light Demo. Running for $max_time seconds."

    # Initialize relevant variables 
    sun = [0; 150e9; 0]             # Sun position  (Note we just leave the satellite at the center)
    J   = [1 0 0; 0 2 0; 0 0 3]     # Satellite Inertia Matrix (completely arbitrary)

    x =  [1; 0; 0; 0;               # Initial quaternion (unrotated)
           0.5; 0.75; 0.33]         # Initial angular velocity

    t, h = 0.0, 0.1                 # Initial time, time step 

    
    count = 1
    sun_hist = zeros(3, length(0:h:max_time))

    while (t < max_time)
        x = rk4(simple_dynamics, x, J, h)      # Update State (AKA rotate)
        
        # Rotate sun into body vector
        q = @view x[1:4]            
        sᴮ = H' * L(q)' * R(q) * H * sun   # Note this is sᴺ -> sᴮ 

        sᴮ = sᴮ / norm(sᴮ)                 # Normalize

        sun_hist[:, count] .= round.(sᴮ, digits = 3)  # Update history 

        # Update Lights
        px, py, pz = sun_hist[:, count]  # Get (rounded) displacements to sun
        msg = "[$px, $py, $pz]"   # Match the desired string format 
        write(sp, "$msg")         # Send position to Arduino


        # Update loop variables
        count += 1  # Increment loop count
        t += h      # Increment time
        sleep(h)
    end

    @info "Loop Finished!"
    display(plot(sun_hist', title = "Sun Hist (Simple Light Demo)", xlabel = "Time (s)", ylabel = "Dist (unit)", label = ["X" "Y" "Z"]))
    close(sp)      # Close Serial port
end

function demo_with_orbit(sp; max_time = 16, μ = 3.9860044188e14, speed_scale = 360) 
    """
          Simulates a satellite orbit, running for a few seconds and updating the 
        LED commands in the test box. Note that the satellite IS orbiting (including
        an eclipse period) but there is NO albedo from the Earth
    """

    @info "Starting Light Demo with Orbit. Running for $(max_time * 6) minutes at $(speed_scale)x speed."

    # Initialize relevant variables 
    sun = [0; 150e9; 0]             # Sun position  (Note we just leave the satellite at the center)
    J   = [1 0 0; 0 2 0; 0 0 3]     # Satellite Inertia Matrix (completely arbitrary)

    t, h = 0.0, 1.0                   # Initial time, time step 
    max_time *= speed_scale

    p₀ = [0; 7e6; 0]                    # Initial position 
    v₀ = sqrt(μ / norm(p₀)) * [1; 0; 0] # Initial velocity 
    q₀ = [1; 0; 0; 0]                   # Initial quaternion 
    ω₀ = [0.005; 0.001; 0.0015]              # Initial angular velocity 
    
    x₀ = [p₀; q₀; v₀; ω₀]

    N = length(0:h:max_time)
    count = 1
    sun_hist = zeros(3, N - 1)
    sat_hist = zeros(3, N - 1) 
    
    x = x₀

    while (t < max_time)
        x = rk4(simple_orbit_dynamics, x, J, h)

        # Rotate sun into body vector 
        q = @view x[4:7]
        sᴮ = H' * L(q)' * R(q) * H * sun   # Note this is sᴺ -> sᴮ
        sᴮ /= norm(sᴮ)

        η = eclipse_conical(-x[1:3], sun) # still wrong
        sᴮ *= η

        # Update Histories
        sun_hist[:, count] .= round.(sᴮ, digits = 3) 
        sat_hist[:, count] .= x[1:3]

        # Update Lights
        # px, py, pz = sun_hist[:, count]  # Get (rounded) displacements to sun
        # msg = "[$px, $py, $pz]"   # Match the desired string format 
        # write(sp, "$msg")         # Send position to Arduino


        # Update loop variables
        count += 1  # Increment loop count
        t += h      # Increment time
        sleep(h / speed_scale)
    end

    @info "Loop finished!"
    # close(sp)

    display(plot(sun_hist', title = "Sun Hist (Simple Orbit Demo)", xlabel = "Time (s)", ylabel = "Dist (unit)", label = ["X" "Y" "Z"]))
    display(plot(sat_hist', title = "Sat Hist (Simple Orbit Demo)", xlabel = "Time (s)", ylabel = "Position (m)", label = ["X" "Y" "Z"]))
    return sun_hist, sat_hist
end

# HELPER FUNCTIONS 
function check_open_ports() 
    """ 
        Checks to see which ports are available 
    """

    println("Usage: $(basename(@__FILE__)) port baud rate")
    println("Available ports: ")
    list_ports()
end


# Quaternion Functions 
hat(v) = [ 0   -v[3]  v[2];
          v[3]   0   -v[1];
         -v[2]  v[1]   0  ]

L(q) = [ q[1] -q[2:4]'; q[2:4]  (q[1]*I + hat(q[2:4])) ]
R(q) = [ q[1] -q[2:4]'; q[2:4]  (q[1]*I - hat(q[2:4])) ]

const H = [0 0 0; 1 0 0; 0 1 0; 0 0 1.0]

function simple_dynamics(x, J) 
    """
        Euler's Equation with no control input (simulates random tumbling)
    """

    q = x[1:4]
    ω = x[5:7]

    q̇ = .5 * (L(q) * [0;ω])
    α = J \ (-cross(ω, J * ω))
    return [q̇; α]
    
end

function simpler_orbit(x, J; μ = 3.9860044188e14)
    r, v = x[1:3], x[4:6]
    a = (-μ/(norm(r)^3)) * r

    return [v; a]
end

function simple_orbit_dynamics(x, J; μ = 3.9860044188e14)

    ẋ = zeros(eltype(x), size(x, 1))
    r, q, v, ω = x[1:3], x[4:7], x[8:10], x[11:13]
    ẋ[1:3]   .= v
    ẋ[4:7]   .= .5 * (L(q) * [0; ω])
    ẋ[8:10]  .= (-μ / (norm(r)^3)) * r 
    ẋ[11:13] .= J \ (-cross(ω, J * ω))

    return ẋ
end

function rk4(f, x, J, h)
    k1 = h * f(x, J)
    k2 = h * f(x + k1/2, J)
    k3 = h * f(x + k2/2, J)
    k4 = h * f(x + k3, J)

    x_next = x + (1/6) * (k1 + 2*k2 + 2*k3 + k4)

    return x_next
end

function rk4_quat(f, x, J, h)
    k1 = h * f(x, J)
    k2 = h * f(x + k1/2, J)
    k3 = h * f(x + k2/2, J)
    k4 = h * f(x + k3, J)

    x_next = x + (1/6) * (k1 + 2*k2 + 2*k3 + k4)

    x_next[1:4] /= norm(x_next[1:4])  # Normalize quaternion 

    return x_next
end

println("Starting")
console_interface()   # Run it all
# sun, sat = demo_with_orbit(0);

# Run with albedo, save, and then just plot?



