from __future__ import annotations

import atexit
from pathlib import Path

import mujoco
from matplotlib import pyplot as plt


class ControlPlotter:
    def __init__(self, print_period: float = 0.5) -> None:
        self._initialized = False
        self._left_dof = -1
        self._right_dof = -1
        self._last_print_time = 0.0
        self._print_period = print_period
        self._records = {
            "t": [],
            "pitch": [],
            "u": [],
            "left_cmd": [],
            "right_cmd": [],
            "left_vel": [],
            "right_vel": [],
        }

    def _joint_dof_index(self, model: mujoco.MjModel, joint_name: str) -> int:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            return -1
        return int(model.jnt_dofadr[joint_id])

    def _save_plot(self) -> None:
        if len(self._records["t"]) == 0:
            return

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        axes[0].plot(self._records["t"], self._records["pitch"], label="pitch [rad]", color="tab:blue")
        axes[0].set_ylabel("Pitch [rad]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")

        axes[1].plot(self._records["t"], self._records["left_cmd"], label="left cmd", color="tab:green")
        axes[1].plot(self._records["t"], self._records["right_cmd"], label="right cmd", color="tab:red")
        axes[1].plot(self._records["t"], self._records["u"], label="control output", color="tab:orange", alpha=0.8)
        axes[1].set_ylabel("Command")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")

        axes[2].plot(self._records["t"], self._records["left_vel"], label="left vel [rad/s]", color="tab:green")
        axes[2].plot(self._records["t"], self._records["right_vel"], label="right vel [rad/s]", color="tab:red")
        axes[2].set_xlabel("Time [s]")
        axes[2].set_ylabel("Wheel vel")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc="best")

        fig.tight_layout()
        out_path = Path("records/control_record.png").resolve()
        fig.savefig(out_path, dpi=140)
        plt.close(fig)
        print(f"[record] Saved plot: {out_path}")

    def init_once(self, firmware) -> None:
        if self._initialized:
            return

        self._left_dof = self._joint_dof_index(firmware.model, "leftWheel")
        self._right_dof = self._joint_dof_index(firmware.model, "rightWheel")
        atexit.register(self._save_plot)
        self._initialized = True

    def record(self, firmware, control_output: float, pitch: float) -> None:
        self.init_once(firmware)

        t = float(firmware.data.time)
        left_cmd = float(firmware.left_wheel_target)
        right_cmd = float(firmware.right_wheel_target)
        left_vel = float(firmware.data.qvel[self._left_dof]) if self._left_dof >= 0 else 0.0
        right_vel = float(firmware.data.qvel[self._right_dof]) if self._right_dof >= 0 else 0.0

        self._records["t"].append(t)
        self._records["pitch"].append(float(pitch))
        self._records["u"].append(float(control_output))
        self._records["left_cmd"].append(left_cmd)
        self._records["right_cmd"].append(right_cmd)
        self._records["left_vel"].append(left_vel)
        self._records["right_vel"].append(right_vel)

        if t - self._last_print_time >= self._print_period:
            self._last_print_time = t
            print(
                f"[t={t:6.3f}] pitch={pitch:+.4f} u={control_output:+.3f} "
                f"cmdL={left_cmd:+.3f} cmdR={right_cmd:+.3f} "
                f"velL={left_vel:+.3f} velR={right_vel:+.3f}"
            )
