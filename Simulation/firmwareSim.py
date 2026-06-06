import mujoco


class firmware:
    controls: dict[str, int] = {
        "left_wheel": 0,
        "right_wheel": 1,
    }
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.left_wheel_target = 0.0
        self.right_wheel_target = 0.0
        self.model = model
        self.data = data

        

    def setTargetSpeeds(self, left_speed: float, right_speed: float) -> None:
        self.left_wheel_target += left_speed * self.model.opt.timestep
        self.right_wheel_target += right_speed * self.model.opt.timestep

        self.data.ctrl[0] = self.left_wheel_target
        self.data.ctrl[1] = self.right_wheel_target

    
    def run(self) -> None:
        self.setTargetSpeeds(2, 2)

