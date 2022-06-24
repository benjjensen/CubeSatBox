""" Interface

      This contains a module used to interface with the devices over 
    a USB port (usually either an Arduino or the CubeSat). Several common 
    functions are provided, so that all that needs to be written is the specific 
    program to run. 

      Includes some simple example functions to send/receive messages from a device.
    Allows for communications with two devices at the same time. 
    
    Still a work in progress
"""
module Interface

    using LibSerialPort 

    export interface, double_interface

    export str2vec, vec2str, check_open_ports

    ### INTERFACE FUNCTIONS ###

    """ interface(f; port, rate)

          Simple interface function that connects to a device at `port` using a baudrate of 
        `rate` and then calls function `f`. Only allows for one device; for two, use 
        `double_interface`. 

        Automatically closes the port when finished or due to a failure.

    """
    function interface(f = default; port = "/dev/ttyACM0", rate = 115200, kwargs...)

        LibSerialPort.open(port, rate) do dev 
            f(dev; kwargs...)
        end
    end

    """ double_interface(f; port1, rate1, port2, rate2)

          Simple interface function that connects to two devices, using their respective baudrates.
        Once connected, this calls function `f`. Only works for two devices 
    """
    function double_interface(f = default; port1 = "/dev/ttyACM0", rate1 = 115200, port2 = "/dev/tty/ACM1", rate2 = 115200, kwargs...)
        
        LibSerialPort.open(port1, rate1) do dev1 
            LibSerialPort.open(port2, rate2) do dev2 
                f(dev1, dev2; kwargs...)
            end
        end
    end


    ### EXAMPLE FUNCTIONS TO RUN ON DEVICE ###

    """ default(dev)

          Simple function that displays a message on a device until 
        'Esc' is pressed. 
    """
    function default(dev::SerialPort)

        usr_msg = "" 
        println("To Exit, hit `esc` (and maybe enter too)")

        while true
            @async usr_msg *= readline(keep = true)            
            println("Saying hello...")
            write(dev, "Hello Device!")
            sleep(1.0)
            yield()

            occursin("\e", usr_msg) && break  # Check if 'Esc' has been pressed to close
            
            if usr_msg != ""
                @show usr_msg 
            end
        end
    end

    """ default(dev1, dev2)

          Simple function that displays a message on both devices until 
        'Esc' is pressed. 
    """ 
    function default(dev1::SerialPort, dev2::SerialPort)

        usr_msg = "" 

        while true 
            @async usr_msg *= readline(keep = true)
            println("Saying hello...")
            write(dev1, "Hello Device 1!")
            write(dev2, "Hello Device 2!")

           sleep(0.5)
           occursin("e", usr_msg) && break # Check if 'Esc' has been pressed to close  
        end
    end

    """ send_msg(dev::SerialPort)

          Simple function that reads user input and sends it to the 
        provided device. Terminates when 'Esc' is input. 
    """
    function send_msg(dev::Interface.SerialPort)

        while true
            usr_msg = readline(keep = false)  # Removes the \n character
            occursin("\e", usr_msg) && break  # Check if 'Esc' has been pressed to close
            
            msg = usr_msg * "\r" * "\n"
            msg = codeunits("$msg")  # Convert to hex

            write(dev, msg)
            sleep(0.25)
        end
    end

    # # # Under Construction # # #
    function send_and_receive(dev::Interface.SerialPort)

        @info "Starting `send_and_receive` loop. Press ESC [return] to quit"
        dev_msg = String(read(dev))  # Clear the line

        while true 

            ### SEND MESSAGE ###

            ## Get message from user
            usr_msg = readline(keep = false) 
            occursin("\e", usr_msg) && break    # Exit if 'Esc' is pressed 

            ## Send to device 
            # msg = "\r" * "\n" * usr_msg  
            msg = usr_msg * "\r" * "\n"
            msg = codeunits("$msg")
            write(dev, msg)

            sleep(0.25)

            ### RECEIVE MESSAGE ###  WARNING - Manages to catch the message you sent, too
            dev_msg = String(read(dev))
            dev_msg = strip(dev_msg, ['\r', '\n'])
            @show dev_msg
        end
    end


    ####################
    # HELPER FUNCTIONS #
    ####################

    """ str2vec(s)

          Takes in a string of form '[n₁, n₂, ..., nₙ]' and returns it as 
        a vector.
    """
    function str2vec(s)

        s = strip(s, ['[', ']', '\r'])      # Remove the '[' and ']' characters, as well as the return '\r'
        v = parse.(Float64, split(s, ","))  # Split by ',' and convert to Floats

        return v
    end

    """ vec2str(v)

        Takes in a vector and converts it to a string of form '[n₁, n₂, ..., nₙ]'
    """
    function vec2str(v)
        s = "["
        for i in v
            s *= "$i, "
        end
        s = s[1:end-2] * "]"
    end

    """ Checks to see which ports are available """
    function check_open_ports() 
        println("Usage: $(basename(@__FILE__)) port baud rate")
        println("Available ports: ")
        list_ports()
    end







    ##############################################
    # Simple functions to run on the python side #
    ##############################################

    """ Waits for input, and lights up when certain values are input """
    """ 
        import time   
        from pycubed import cubesat 
        import ulab.numpy as np 
        
        while True:
            cubesat.RGB = WHITE
            c = input()
            cubesat.RGB = BLUE   
        
            if c == "a"
                cubesat.RGB = GREEN 
            elif c == "1":
                cubesat.RGB = BLUE        
            else:   
                cubesat.RGB = RED     
        
            time.sleep(0.25)
    """

    """ Takes in a vector of times to hold each color (e.g., [1.0, 2.2, 0.5] seconds)"""
    """
        import time   
        from pycubed import cubesat 
        import ulab.numpy as np 
        
        while True:
            cubesat.RGB = WHITE
            c = input()
            cubesat.RGB = BLUE   
        
            if (len(c) > 1) and (c[0] == "["): 
                v = c.strip("[]")
                v = v.split(",")
                v = [float(v[i]) for i in range(len(v))]
        
                cubesat.RGB = GREEN 
                time.sleep(v[0])
                cubesat.RGB = BLUE 
                time.sleep(v[1])
                cubesat.RGB = TEAL 
                time.sleep(v[2])
        
            else:   
                cubesat.RGB = RED     
        
            time.sleep(0.25)
    """

end