# [ArduinoComm/LightDemo/interactive_demo.jl]

"""
      Interactive demo of cubeSat test box's LED feature.
    Julia generates an interactive GLMakie plot that allows a user to 
    move the sun, and then updates the box LEDs accordingly. 

    (Works with microcontroller/microcontroller.ino)


    TODO: 
    - Figure out how to get the quit function to work, or add in an exit button 
"""

using LibSerialPort         # Allows for serial communication with Arduino 
using LinearAlgebra 
using GLMakie 

function console_interface(port = "/dev/ttyACM1", baud_rate = 115200)
    """
        Interface function that opens serial communication and starts light demo 
    """

    mcu = open(port, baud_rate)  # Open up connection to microcontroller 

    return interactive_sun(mcu) 
end

function interactive_sun(sp::SerialPort)
    """
          Generates an interactive plot showing a cubeSat and the sun. 
        A user can move the location of the sun, and see the corresponding LEDs 
        light up on the test box. Note there is no eclipse or albedo from the Earth
    """

    @info "Starting 'Interactive Sun' demo. Running until ESC [return] is pressed."
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

    # on(events(fig).keyboardbutton) do event
    #     event.key == Keyboard.q  &&  GLFW.SetWindowShouldClose(glfw_window, true) # this will close the window after all callbacks are finished
    #     # ispressed(key, Keyboard.q) && GLMakie.destroy!(GLMakie.global_g1_screen())
    # end



    # Initialize relevant variables 
    ρ = 25                          # Radius for spherical coordinates
    s = sph2cart(θ₀, ϕ₀, ρ)         # Initial Position
    sᴮ = Observable(Point3f[s])     # Make it observable so we can interact

    # Specify action on slider move
    lift(sl_θ.value, sl_ϕ.value) do θ, ϕ
        θʳ, ϕʳ = deg2rad(θ), deg2rad(ϕ)
        @show θ, ϕ

        s = sph2cart(θʳ, ϕʳ, ρ)
        sᴮ[] = [Point3f(s)]        # Pushes update to the plot
    end 

    scatter!(sᴮ, markersize = 10000, color = :yellow)

    display(fig)

    user_input = ""
    LEDs = zeros(3)

    @info "\tStarting loop..."
    while (events(fig).window_open[])
        # Poll for new data without blocking
        # @sync user_input = readline(keep = true)
        # occursin("\e", user_input) && exit()  # Escape when \e is input   # <--- causes problems, maybe add an observable "exit" button...



        # Update sun location 
        LEDs .= round.(s / norm(s), digits = 3)  # Make it unit
        px, py, pz = LEDs 
        msg = "[$px, $py, $pz]"  # Match the desired string format 
        write(sp, "$msg")        # Send position to Arduino


        sleep(0.1)
    end

    @info "Terminating Simulation!"
    close(sp)
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

console_interface()