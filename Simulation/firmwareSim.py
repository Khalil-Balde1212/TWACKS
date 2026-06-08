import mujoco
import math


class firmware:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.left_wheel_target = 0.0
        self.right_wheel_target = 0.0
        self.max_speed = 20.0
        self.model = model
        self.data = data

    def setTargetSpeeds(self, left_speed: float, right_speed: float) -> None:
        self.left_wheel_target += left_speed * self.model.opt.timestep
        self.right_wheel_target += right_speed * self.model.opt.timestep

        self.left_wheel_target = max(-self.max_speed, min(self.max_speed, self.left_wheel_target))
        self.right_wheel_target = max(-self.max_speed, min(self.max_speed, self.right_wheel_target))

        self.data.ctrl[0] = self.left_wheel_target
        self.data.ctrl[1] = self.right_wheel_target

    def _base_quat_wxyz(self) -> tuple[float, float, float, float]:
        # Free joint layout: qpos[0:3] position, qpos[3:7] orientation quaternion.
        return (
            float(self.data.qpos[3]),
            float(self.data.qpos[4]),
            float(self.data.qpos[5]),
            float(self.data.qpos[6]),
        )

    def _base_euler_zyx(self) -> tuple[float, float, float]:
        # Convert base quaternion (wxyz) to intrinsic ZYX Euler angles.
        w, x, y, z = self._base_quat_wxyz()

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll

    def getYaw(self) -> float:
        yaw, _, _ = self._base_euler_zyx()
        return yaw

    def getPitch(self) -> float:
        _, pitch, _ = self._base_euler_zyx()
        return pitch

    def getRoll(self) -> float:
        _, _, roll = self._base_euler_zyx()
        return roll

    def getYawRate(self) -> float:
        # Free joint angular velocity components are qvel[3:6] = (wx, wy, wz).
        return float(self.data.qvel[5])

    def getPitchRate(self) -> float:
        return float(self.data.qvel[4])

    def getRollRate(self) -> float:
        return float(self.data.qvel[3])


