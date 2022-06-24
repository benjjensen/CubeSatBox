# [CubeSatBox/CommsDemo/SerialCommunication.jl]

using LibSerialPort 
# using MsgPack  #<--- TODO Check it out 
using Plots


function console_interface(port = "/dev/ttyACM0", baud_rate = 115200)

    t = []
    LibSerialPort.open(port, baud_rate) do prt 
        read_write_port(prt)
    end

    return t
end

function read_from_port(sp::SerialPort) 
    msg = "" 

    @info "Starting 'Read from port' loop. Press ESC [return] to quit"

    while true 
        @async msg *= String(nonblocking_read(sp))

        occursin("\e", msg) && exit()  # Exit when \e is input

        if occursin("\n", msg) 
            lines = split(msg, "\n") # Print out each line 
            while (length(lines) > 1)
                println(popfirst!(lines))
            end
            msg = lines[1]
        end


        sleep(0.0001) # Give queued tasks a chance to run 

    end
end

function str2vec(s)
    sTrim = s[2:end-2]   # Remove brackets <--- try 'strip()'
    vals = parse.( Float64, split(sTrim, ",")) # Split by comma and convert to float 
    return vals
end

function read_write_port(sp::SerialPort) 
    ser_msg = "" 
    usr_msg = ""

    @info "Starting 'Read from port' loop. Press ESC [return] to quit"

    vecs = []

    while true 
        @async usr_msg = readline(keep = true)
        @async ser_msg *= String(nonblocking_read(sp))

        # NOTE - INPUT DOES NOT WORK (unsure why...)
        occursin("\e", usr_msg) && exit()  # Exit when \e is input

        # Send user input to device with ENTER 
        if endswith(usr_msg, '\n')
            @show "Sending $usr_msg"
            write(sp, "$usr_msg")
            usr_msg = "" 
        end

        # Print input
        if occursin("\n", ser_msg) 
            lines = split(ser_msg, "\n") # Print out each line 
            while (length(lines) > 1)
                str = popfirst!(lines)
                if str[1] == '['
                    push!(vecs, str2vec(str))
                end
                println(str)
            end

            ser_msg = lines[1]

            if size(vecs, 1) > 20
                println("Breaking!")
                break
            end
        end


        sleep(0.0001) # Give queued tasks a chance to run 

    end

    return vecs
end

function read_msgpack_from_port(sp::SerialPort) 
    ser_msg = "" 
    usr_msg = ""

    @info "Starting 'Read from port' loop. Press ESC [return] to quit"

    msgpack = pyimport("msgpack")

    while true 
        @async ser_msg = nonblocking_read(sp) #String(nonblocking_read(sp))
 
        try 
            println(msgpack.unpack(ser_msg))
        catch
            t = 2
        end
        @async usr_msg = readline(keep = true)

        occursin("\e", usr_msg) && exit()  # Exit when \e is input


        # # Send user input to device with ENTER 
        # if endswith(usr_msg, '\n')
        #     write(sp, "$usr_msg")
        #     usr_msg = "" 
        # end

        # Print input
        # if occursin("\n", ser_msg) 
           
        #     println( msgpack.unpack(ser_msg))

        #     # lines = split(ser_msg, "\n") # Print out each line 
        #     # while (length(lines) > 1)
        #     #     println( (popfirst!(lines)) )
        #     # end
        #     # ser_msg = lines[1]
        # end


        sleep(0.0001) # Give queued tasks a chance to run 

    end
end



# HELPER FUNCTIONS 

