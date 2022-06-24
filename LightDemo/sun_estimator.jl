# [CubeSatBox/LightDemo/sun_estimator.jl]

"""
        Interactive demo of cubeSat test box's LED feature that 
    also outputs the unit vector in the direction of the sun, so that 
    we can evaluate how accurately the satellite can estimate the sun 
    vector. 

    Intended to be run with microcontroller/microcontroller.ino (to 
    operate the LEDs), as well as sun_estimator.py (to 
    estimate the sun vector on the cube satellite)

"""

using LibSerialPort         # Allows for serial communication with Arduino 
using LinearAlgebra 
using GLMakie 



function console_interface(;ardu_port = "/dev/ttyACM0", ardu_rate = 115200, sat_port = "/dev/ttyACM1", sat_rate = 115200)
    """ Interface function that opens serial communication and starts light demo """

    LibSerialPort.open(ardu_port, ardu_rate) do mcu 
        LibSerialPort.open(sat_port, sat_rate) do sat 
            interactive_sun(mcu, sat)
            # test_double_port(mcu, sat)
        end
    end
    # mcu = open(port, baud_rate)  # Open up connection to microcontroller 

    # return interactive_sun(mcu) 
end


function str2vec(s)
    """Takes in a string of form '[x, y, z]' and returns a vector """

    s = strip(s, ['[', ']', '\r'] )  # Remove the '[' and ']' characters 
    vals = parse.(Float64, split(s, ",")) # Split by ',' and convert to Floats 

    return vals
end

function test_double_port(mcu_port::SerialPort, sat_port::SerialPort) 

    @info "Starting test_double_port"

    # Clear the buffer...?
    trash = String(nonblocking_read(mcu_port))
    trash = String(nonblocking_read(sat_port))

    mcu_msg = "" 
    sat_msg = "" 
    while true
        println("\n------------------")

        # @async mcu_msg *= String(nonblocking_read(mcu_port))  
        # @async sat_msg *= String(nonblocking_read(sat_port))
        mcu_msg *= String(nonblocking_read(mcu_port))  
        sat_msg *= String(nonblocking_read(sat_port))

        if (occursin("\n", sat_msg))
            lines = split(sat_msg, "\n")
            while (length(lines) > 1) 
                println(popfirst!(lines))
            end
            sat_msg = ""
        end

        if (occursin("\n", mcu_msg))
            lines = split(mcu_msg, "\n")  # Split into lines 
            while (length(lines) > 1)
                println(popfirst!(lines))
            end
            mcu_msg = ""
        end




        sleep(1)

    end

end


function interactive_sun(mcu_port::SerialPort, sat_port::SerialPort)
    """
          Generates an interactive plot showing a cubeSat and the sun. 
        A user can move the location of the sun, and see the corresponding LEDs 
        light up on the test box. Note there is no eclipse or albedo from the Earth
    """

    @info "Starting 'Interactive Sun' demo. Running until you exit the plot"
    @info "\t(Note that this can take a while the first time...)"


    function sph2cart(θ, ϕ, ρ)
        x = ρ * sin(ϕ) * cos(θ)
        y = ρ * sin(ϕ) * sin(θ)
        z = ρ * cos(ϕ)

        return [x, y, z]
    end

    ### Generate visualization

    # Generate Cube 
    r = LinRange(-0.1, 0.1, 10);
    cube = [(x.^2 + y.^2 + z.^2) for x = r, y = r, z = r];
    fig, ax = volume(cube)

    # Add Sliders 
    θ₀, ϕ₀ = 0, 90
    sl_θ = Slider(fig[2, 1], range = 0:1:180, startvalue = θ₀)
    sl_ϕ = Slider(fig[3, 1], range = 0:1:360, startvalue = ϕ₀)


    # Initialize relevant variables 
    ρ = 25                          # Radius for spherical coordinates
    s = sph2cart(θ₀, ϕ₀, ρ)         # Initial Position
    sᴮ = Observable(Point3f[s])     # Make it observable so we can interact

    # Specify action on slider move
    lift(sl_θ.value, sl_ϕ.value) do θ, ϕ
        θʳ, ϕʳ = deg2rad(θ), deg2rad(ϕ)

        s = sph2cart(θʳ, ϕʳ, ρ)

        sᴮ[] = [Point3f(s)]        # Pushes update to the plot
    end 

    scatter!(sᴮ, markersize = 10000, color = :yellow)

    display(fig)

    # mcu_msg = ""   # String input from the Arduino 
    sat_msg = ""   # String input from the CubeSat
    sun_vec = zeros(3)

    @info "\tStarting loop..."
    while (events(fig).window_open[])

        ### UPDATE PLOT, LIGHTS
        # Update sun location 
        sun_vec .= round.(s / norm(s), digits = 3)  # Make it unit
        px, py, pz = sun_vec 
        msg = "[$px, $py, $pz]"  # Match the desired string format
        write(mcu_port, "$msg")        # Send position to Arduino
    
        ### PRINT OUT STUFF

        # sat_msg = String(read(sat_port))
        @async sat_msg *= String(nonblocking_read(sat_port))
        

        if (occursin("\n", sat_msg))
            println("\n------------------")
            lines = split(sat_msg, "\n")  # Split into lines 
            while (length(lines) > 1)
                line = popfirst!(lines)

                if line[1] == '['  # Verify it is an array and not an error message 
                    est = str2vec(line)
                    err = rad2deg(acos(sun_vec' * est))   # Get angular distance
                    print("\t$msg \t$est   \t$err")
                else
                    println("\t $line")
                end
                
            end
            sat_msg = ""
        end

        sleep(0.1)
    end

    @info "Terminating Simulation!"
end



# HELPER FUNCTIONS 
function check_open_ports() 
    """ 
        Checks to see which ports are available (as well as info about each)
    """

    println("Usage: $(basename(@__FILE__)) port baud rate")
    println("Available ports: ")
    list_ports()
end

# console_interface()