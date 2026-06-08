
from pathlib import Path
import time

import mujoco
import mujoco.viewer

import onboardBalanceControl
import firmwareSim


def main() -> None:
    scene_path = Path("./my-robot/scene.xml").resolve()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    data.qpos[2] = 0.15
    mujoco.mj_forward(model, data)
    robot_firmware = firmwareSim.firmware(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            onboardBalanceControl.run(robot_firmware)
            mujoco.mj_step(model, data)

            # Synchronize with real time. Not sim time
            viewer.sync()
            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()