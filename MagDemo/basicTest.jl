# [CubeSatBox/MagDemo/basicTest.jl]

""" Basic Test:

      Simple script designed to just connect to magnetometers and 
    print out the resulting measurements. Intended to be used in conjunction 
    with the basicTest/basicTest.ino Arduino script
"""

using LibSerialPort 
using Plots

function console_interface(port = "/dev/ttyACM0", baud_rate = 115200) 
    LibSerialPort.open(port, baud_rate) do prt 
        read_magnetometers(prt)
    end
end

function read_magnetometers(sp::SerialPort)
    sens_msg = ""
    usr_input = "" 
    
    @info "Starting 'Read Magnetometers' loop. Press ESC [return] to quit"

    while true 
        @async sens_msg *= String(nonblocking_read(sp))
        @async usr_input = readline(keep = true)

        occursin('\e', usr_input) && exit() # Exit when \e is input (maybe use " "?)

        if occursin("\n", sens_msg)
            lines = split(sens_msg, "\n")  
            while length(lines) > 1
                println(popfirst!(lines))
            end 
            sens_msg = lines[1]

        end


        sleep(0.0001)  # Give queued tasks a chance to run 
    end
end

function check_open_ports() 
    """ 
        Checks to see which ports are available 
    """

    println("Usage: $(basename(@__FILE__)) port baud rate")
    println("Available ports: ")
    list_ports()
end;
