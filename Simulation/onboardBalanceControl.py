import firmwareSim
import mujoco
import atexit
from pathlib import Path

from matplotlib import pyplot as plt


_initialized = False
_left_dof = -1
_right_dof = -1
_last_print_time = 0.0
_print_period = 0.5
_records = {
    "t": [],
    "yaw": [],
    "u": [],
    "left_cmd": [],
    "right_cmd": [],
    "left_vel": [],
    "right_vel": [],
}


def _joint_dof_index(model: mujoco.MjModel, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return -1
    return int(model.jnt_dofadr[joint_id])


def _save_plot() -> None:
    if len(_records["t"]) == 0:
        return

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(_records["t"], _records["yaw"], label="yaw [rad]", color="tab:blue")
    axes[0].set_ylabel("Yaw [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(_records["t"], _records["left_cmd"], label="left cmd", color="tab:green")
    axes[1].plot(_records["t"], _records["right_cmd"], label="right cmd", color="tab:red")
    axes[1].plot(_records["t"], _records["u"], label="control output", color="tab:orange", alpha=0.8)
    axes[1].set_ylabel("Command")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].plot(_records["t"], _records["left_vel"], label="left vel [rad/s]", color="tab:green")
    axes[2].plot(_records["t"], _records["right_vel"], label="right vel [rad/s]", color="tab:red")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Wheel vel")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.tight_layout()
    out_path = Path("records/control_record.png").resolve()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[record] Saved plot: {out_path}")


def _init_once(firmware) -> None:
    global _initialized, _left_dof, _right_dof
    if _initialized:
        return

    _left_dof = _joint_dof_index(firmware.model, "leftWheel")
    _right_dof = _joint_dof_index(firmware.model, "rightWheel")
    atexit.register(_save_plot)
    _initialized = True

def run(firmware) -> None:
    global _last_print_time
    _init_once(firmware)

    kp, ki, kd = -10000.0, 0.0, 10.0
    perroer, ierror, derror = 0.0, 0.0, 0.0

    # PID
    perror = firmware.getYaw() - 0.0
    ierror += perror * firmware.model.opt.timestep
    derror = (perror - perroer) / firmware.model.opt.timestep
    
    output = kp * perror + ki * ierror + kd * derror
    firmware.setTargetSpeeds(output, output)

    t = float(firmware.data.time)
    yaw = float(firmware.getYaw())
    left_cmd = float(firmware.left_wheel_target)
    right_cmd = float(firmware.right_wheel_target)
    left_vel = float(firmware.data.qvel[_left_dof]) if _left_dof >= 0 else 0.0
    right_vel = float(firmware.data.qvel[_right_dof]) if _right_dof >= 0 else 0.0

    _records["t"].append(t)
    _records["yaw"].append(yaw)
    _records["u"].append(float(output))
    _records["left_cmd"].append(left_cmd)
    _records["right_cmd"].append(right_cmd)
    _records["left_vel"].append(left_vel)
    _records["right_vel"].append(right_vel)

    if t - _last_print_time >= _print_period:
        _last_print_time = t
        print(
            f"[t={t:6.3f}] yaw={yaw:+.4f} u={output:+.3f} "
            f"cmdL={left_cmd:+.3f} cmdR={right_cmd:+.3f} "
            f"velL={left_vel:+.3f} velR={right_vel:+.3f}"
        )