
include("../../plotlib/plotlib.jl")
include("../../serialportlib/serialportlib.jl")

struct MPU6050_Data
    time_us::Int64
    omega_x::Float32
    omega_y::Float32
    omega_z::Float32
    acc_x::Float32
    acc_y::Float32
    acc_z::Float32
end

if Serialportlib.connect("/dev/ttyACM0", 115200)
    Serialportlib.set_delimiter([0xf4, 0xf3, 0xf2, 0xf1])

    Plotlib.panel_add_topbottom_tile(0)
    Plotlib.panel_add_leftright_tile(2)

    Plotlib.set_plotter_name(0, "Accel m/s^2")
    Plotlib.plotter_add_plot(0, 0)
    Plotlib.set_plot_name(0, "X")
    Plotlib.plotter_add_plot(0, 1)
    Plotlib.set_plot_name(1, "Y")
    Plotlib.plotter_add_plot(0, 2)
    Plotlib.set_plot_name(2, "Z")
    Plotlib.plotter_track_latest(0, 3000)
    Plotlib.show_plotter(0)

    Plotlib.set_plotter_name(1, "Omega rad/s")
    Plotlib.plotter_add_plot(1, 3)
    Plotlib.set_plot_name(3, "X")
    Plotlib.plotter_add_plot(1, 4)
    Plotlib.set_plot_name(4, "Y")
    Plotlib.plotter_add_plot(1, 5)
    Plotlib.set_plot_name(5, "Z")
    Plotlib.plotter_track_latest(1, 3000)
    Plotlib.show_plotter(1)

    Plotlib.plotter3d_add_plot3d(0, 0)
    Plotlib.set_plot3d_name(0, "Accel Vector")
    Plotlib.plot3d_as_lines(0)
    Plotlib.plotter3d_track_point(0,  0, 0, 0) # stay fixed to the origin
    Plotlib.plotter3d_add_plot3d(0, 1)
    
    Plotlib.set_plot3d_name(1, "Origin")
    Plotlib.plot3d_as_lines(1);
    Plotlib.plot3d_append_vertex_with_color(1,  0, 0, 0, Plotlib.RED);
    Plotlib.plot3d_append_vertex_with_color(1,  5, 0, 0, Plotlib.RED);
    Plotlib.plot3d_append_vertex_with_color(1,  0, 0, 0, Plotlib.GREEN);
    Plotlib.plot3d_append_vertex_with_color(1,  0, 5, 0, Plotlib.GREEN);
    Plotlib.plot3d_append_vertex_with_color(1,  0, 0, 0, Plotlib.BLUE);
    Plotlib.plot3d_append_vertex_with_color(1,  0, 0, 5, Plotlib.BLUE);


    Plotlib.interactive()

    Plotlib.show_plotter3d(0)

    while true
        mpu_data = Serialportlib.get_next_object(MPU6050_Data)
        if (mpu_data != nothing)
            time_s = mpu_data.time_us * 1e-6
            Plotlib.plot_append_point(0, time_s, mpu_data.acc_x)
            Plotlib.plot_append_point(1, time_s, mpu_data.acc_y)
            Plotlib.plot_append_point(2, time_s, mpu_data.acc_z)
            
            Plotlib.plot_append_point(3, time_s, mpu_data.omega_x)
            Plotlib.plot_append_point(4, time_s, mpu_data.omega_y)
            Plotlib.plot_append_point(5, time_s, mpu_data.omega_z)

            Plotlib.plot3d_fill_vertices_x_y_z(0,  [0, mpu_data.acc_x], [0, mpu_data.acc_y], [0, mpu_data.acc_z])
        end
        @ccall usleep(1000::UInt64)::Int32
    end
end
