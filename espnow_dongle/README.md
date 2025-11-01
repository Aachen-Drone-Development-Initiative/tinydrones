# ESP-Now Dongle

The dongle can (in theroy) be any esp32 which supports Serial/JTAG communication, right now everything is configured to work with the `esp32-c6`

## Communicating with the dongle from Julia

Assuming that Julia's working directory is this directory (though you can do this from any directory).

```julia
julia> include("./julia_interface/espnow_dongle.jl")
julia> Dongle.open("/dev/ttyACM0") # choose your actual port-name here
julia> Dongle.read_packet()
```

More examples will be added once the functionality is tested :)
