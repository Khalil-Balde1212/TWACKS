from control_plotter import ControlPlotter


_plotter = ControlPlotter(print_period=0.5)


prev_error, ierror, derror = 0.0, 0.0, 0.0
def run(firmware) -> None:
    global prev_error, ierror, derror

    kp, ki, kd = 30, 14, 0.2
    # PID
    error = firmware.getPitch() - 0.0
    ierror += error * firmware.model.opt.timestep
    derror = (error - prev_error) / firmware.model.opt.timestep
    prev_error = error
    
    output = kp * error + ki * ierror + kd * derror
    output *= 1000
    firmware.setTargetSpeeds(output, output)

    pitch = float(firmware.getPitch())
    _plotter.record(firmware, control_output=float(output), pitch=pitch)


if __name__ == "__main__":
    import visual

    visual.main()