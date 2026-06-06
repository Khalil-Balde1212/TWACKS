import firmwareSim
import mujoco

def run(firmware) -> None:
    kp, ki, kd = -10000.0, 0.0, 10.0
    perroer, ierror, derror = 0.0, 0.0, 0.0

    # PID
    perror = firmware.getYaw() - 0.0
    ierror += perror * firmware.model.opt.timestep
    derror = (perror - perroer) / firmware.model.opt.timestep
    
    output = kp * perror + ki * ierror + kd * derror
    firmware.setTargetSpeeds(output, output)