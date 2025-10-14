
module Serialportlib

const SERIALPORTLIB_MAX_SERIALPORT_IDX = 1000
const DEFAULT_SP_IDX = 0

@enum Serial_Port_Flags begin
    DTR = 0
    RTS = 1
end

serialportlib = "" # initialized in __init__()

function open(portname::String, baudrate; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_open(sp_idx::UInt32, portname::Cstring, baudrate::UInt32)::Bool
end

function close(; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_close(sp_idx::UInt32)::Bool
end

function is_open(; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_is_open(sp_idx::UInt32)::Bool
end

function set_flag(flag::Serial_Port_Flags, value::Bool; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_set_flag(sp_idx::UInt32, flag::Cint, value::Bool)::Bool
end

function bytes_available_to_read(; sp_idx=DEFAULT_SP_IDX)::UInt64
    @ccall serialportlib.serialport_bytes_available_to_read(sp_idx::UInt32)::Int64
end

"Try to read as many as 'out_size' bytes. Returns how many bytes were actually read."
function read_byte(; sp_idx=DEFAULT_SP_IDX)::UInt8
    @ccall serialportlib.serialport_read_byte(sp_idx::UInt32)::UInt8
end

"Try to read as many as 'out_size' bytes. Returns how many bytes were actually read."
function read_bytes(bytes_count; sp_idx=DEFAULT_SP_IDX)::Vector{UInt8}
    bytes = Vector{UInt8}(undef, bytes_count)
    bytes_received = @ccall serialportlib.serialport_read_bytes(sp_idx::UInt32, bytes::Ptr{Cvoid}, bytes_count::UInt64)::Int64
    resize!(bytes, bytes_received)
    return bytes
end

function read_bytes_as_object(::Type{Out}; sp_idx=DEFAULT_SP_IDX) where { Out }
    isbitstype(Out) || throw(ArgumentError("Object type for `read_bytes_as_object` must be isbits"))
    
    out_size = sizeof(Out)
    bytes_available_to_read(sp_idx=sp_idx) < out_size || return nothing
        
    out_ref = Ref{Out}() # allocates memory for the object of type Out
    
    bytes_received = GC.@preserve out_ref begin
        out_ptr = Base.unsafe_convert(Ptr{Out}, out_ref)
        @ccall serialportlib.serialport_read_bytes(sp_idx::UInt32, out_ptr::Ptr{Cvoid}, out_size::UInt64)::Int64
    end
    
    bytes_received >= 0 || return nothing
    @assert(bytes_received == out_size)
    
    return out_ref[]
end

function read_bytes_as_object_packed(::Type{Out}; sp_idx=DEFAULT_SP_IDX) where { Out }
    isbitstype(Out) || throw(ArgumentError("Object type for `read_bytes_as_object_packed` must be isbits"))
    
    out_packedsize = Base.packedsize(Out)
    # println("bytes available: $(bytes_available_to_read(sp_idx=sp_idx))")
    bytes_available_to_read(sp_idx=sp_idx) >= out_packedsize || return nothing
    out_packed_ref = Ref{NTuple{out_packedsize, UInt8}}() # allocates memory for the packed bytes of the object of type Out
    
    bytes_received = GC.@preserve out_packed_ref begin
        out_packed_ptr = Base.unsafe_convert(Ptr{UInt8}, out_packed_ref)
        @ccall serialportlib.serialport_read_bytes(sp_idx::UInt32, out_packed_ptr::Ptr{Cvoid}, out_packedsize::UInt64)::Int64
    end
    
    bytes_received >= 0 || return nothing
    @assert(bytes_received == out_packedsize)

    return reinterpret(Out, out_packed_ref[]) # 'reinterpret(::Type{Out}, x::In)' converts packed bytes to an unpacked isbits struct
end

"""
Comsumes bytes from the serialport until it finds the delimiter. The next bytes read are the ones just after the delimiter.
You can safely read your data when 'true' is returned. 'false' often means that all available bytes were exhausted.
This is useful for when your data-stream becomes misaligned because a byte got missing etc.
"""
function skip_behind_next_delimiter(delimiter::BytesVectorT; sp_idx=DEFAULT_SP_IDX)::Bool where { BytesVectorT <: AbstractVector{UInt8} }
    @ccall serialportlib.serialport_skip_behind_next_delimiter(sp_idx::UInt32, delimiter::Ptr{Cvoid}, length(delimiter)::UInt64)::Bool
end

function skip_behind_next_delimiter(delimiter::DelimT; sp_idx=DEFAULT_SP_IDX)::Bool where { DelimT }
    isbitstype(DelimT) || throw(ArgumentError("Delimiter type for `skip_behind_next_delimiter` must be isbits"))
    delim_ref = Ref{DelimT}(delimiter)
    return GC.@preserve delim_ref begin
        delim_ptr = Base.unsafe_convert(Ptr{DelimT}, delim_ref)
        @ccall serialportlib.serialport_skip_behind_next_delimiter(sp_idx::UInt32, delim_ptr::Ptr{Cvoid}, sizeof(DelimT)::UInt64)::Bool
    end
end

function write_byte(byte; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_write_byte(sp_idx::UInt32, byte::UInt8)::Bool
end

function write_bytes(bytes::BytesVectorT; sp_idx=DEFAULT_SP_IDX)::Int64 where { BytesVectorT <: AbstractVector{UInt8} }
    @ccall serialportlib.serialport_write_bytes(sp_idx::UInt32, bytes::Ptr{Cvoid}, length(bytes)::UInt64)::Int64
end

function write_bytes_from_object(object::In; sp_idx=DEFAULT_SP_IDX)::Int64 where { In }
    isbitstype(In) || throw(ArgumentError("Object type for `write_bytes_from_object` must be isbits"))
    in_ref = Ref{In}(object)
    return GC.@preserve in_ref begin
        in_ptr = Base.unsafe_convert(Ptr{In}, in_ref)
        @ccall serialportlib.serialport_write_bytes(sp_idx::UInt32, in_ptr::Ptr{Cvoid}, sizeof(In)::UInt64)::Int64
    end
end

function write_bytes_from_object_packed(object::In; sp_idx=DEFAULT_SP_IDX)::Int64 where { In }
    isbitstype(In) || throw(ArgumentError("Object type for `write_bytes_from_object_packed` must be isbits"))
    packed_size = Base.packedsize(object)
    in_packed = reinterpret(NTuple{packed_size, UInt8}, object)
    in_packed_ref = Ref{NTuple{packed_size, UInt8}}(in_packed)
    return GC.@preserve in_packed_ref begin
        in_packed_ptr = Base.unsafe_convert(Ptr{UInt8}, in_packed_ref)
        @ccall serialportlib.serialport_write_bytes(sp_idx::UInt32, in_packed_ptr::Ptr{Cvoid}, packed_size::UInt64)::Int64
    end
end

# gets called when this module (Serialportlib) is loaded
function __init__()
    
    if Sys.islinux()
        global serialportlib  = "$(@__DIR__)/libserialportlib.so"
        serialportlib_cpp     = "$(@__DIR__)/serialportlib.cpp"
        
        requires_rebuild = true
        if !isfile(serialportlib)
            @info("'libserialportlib.so' not found in the folder which contains 'serialportlib.jl', building from source with GCC (g++)...")
        elseif stat(serialportlib).mtime < stat(serialportlib_cpp).mtime
            @info("'libserialportlib.so' older than 'serialportlib.cpp' rebuilding from source with GCC (g++)...")
        else
            requires_rebuild = false
        end
        
        if requires_rebuild
            try
                # Change to directory of this file, build the shared library and go back to the original working directory
                working_dir = pwd()
                cd(@__DIR__)
                run(`g++ -fPIC -fvisibility=hidden -ggdb -c -Wall -Wextra serialportlib.cpp`, wait=true)
                run(`g++ -shared -o libserialportlib.so serialportlib.o`, wait=true)
                cd(working_dir)
            catch e
                @error("Failed to build `libserialportlib.so`. Please include 'serialportlib.jl' again, after resolving the error(s).")
            else
                global serialportlib = "$(@__DIR__)/libserialportlib.so"
                @assert(isfile(serialportlib))
                @info("Successfully built 'libserialportlib.so'!")
            end
        end
    else
        @error("Your operating system with kernel '$(Sys.KERNEL)' is not supported by Serialportlib! Only Linux is supported.")
    end
end

end # module Serialportlib
