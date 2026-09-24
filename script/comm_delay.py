import time
import math
import time
import numpy as np
import xyber_py as xyber

ctrl = xyber.XyberController.get_instance()
ctrl.create_device("biped", ["can0", "can1", "can2", "can3"])

ctrl.attach_actuator("biped", 2, xyber.ActuatorType.Robstride_02, "left_hip_pitch_joint", 1)
ctrl.attach_actuator("biped", 2, xyber.ActuatorType.Robstride_02, "left_hip_roll_joint", 2)
ctrl.attach_actuator("biped", 2, xyber.ActuatorType.Robstride_05, "left_hip_yaw_joint", 3)
ctrl.attach_actuator("biped", 1, xyber.ActuatorType.Robstride_00, "left_hip_knee_joint", 1)
ctrl.attach_actuator("biped", 1, xyber.ActuatorType.Robstride_00, "left_ankle_joint", 2)
ctrl.attach_actuator("biped", 3, xyber.ActuatorType.Robstride_02, "right_hip_pitch_joint", 1)
ctrl.attach_actuator("biped", 3, xyber.ActuatorType.Robstride_02, "right_hip_roll_joint", 2)
ctrl.attach_actuator("biped", 3, xyber.ActuatorType.Robstride_05, "right_hip_yaw_joint", 3)
ctrl.attach_actuator("biped", 0, xyber.ActuatorType.Robstride_00, "right_hip_knee_joint", 1)
ctrl.attach_actuator("biped", 0, xyber.ActuatorType.Robstride_00, "right_ankle_joint", 2)

ctrl.set_realtime(rt_priority=80, bind_cpu=3)
ctrl.start(cycle_ns=1_538_461) # 650 Hz

#ctrl.enable_all_actuator()
time.sleep(0.1)


# Sign correction per joint (hardware convention vs URDF convention)
JOINT_SIGNS = np.array([
     1.0,   # left_hip_roll
     1.0,   # left_hip_pitch
     1.0,   # left_hip_yaw
     1.0,   # left_hip_knee
     1.0,   # left_ankle
     1.0,   # right_hip_roll
     1.0,   # right_hip_pitch
     1.0,   # right_hip_yaw
     1.0,   # right_hip_knee
     1.0,   # right_ankle
])

def send_commands(ctrl, target_pos, kps, kds):

    """Send MIT-mode position commands to all 8 joints."""
    for i, name in enumerate(JOINT_NAMES):
        ctrl.set_mit_cmd(
            name,
            target_pos[i] * JOINT_SIGNS[i],
            0.0, 0.0,       # vel_ff, torque_ff
            kps[i], kds[i],
        )

def set_all_zero():
    for name in JOINT_NAMES:
        ctrl.set_homing_position(name)
    time.sleep(1.0)

def read_pos():
    posz = 0
    while True:
        ctrl.set_mit_cmd("left_hip_roll_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("right_hip_roll_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("left_hip_pitch_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("right_hip_pitch_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("left_hip_yaw_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("right_hip_yaw_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("left_hip_knee_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("right_hip_knee_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("left_ankle_joint", posz, 0.0, 0.0, 10.0, 0.5)
        ctrl.set_mit_cmd("right_ankle_joint", posz, 0.0, 0.0, 10.0, 0.5)
        lhr = ctrl.get_position("left_hip_roll_joint")
        rhr = ctrl.get_position("right_hip_roll_joint")
        lhp = ctrl.get_position("left_hip_pitch_joint")
        rhp = ctrl.get_position("right_hip_pitch_joint")
        lhy = ctrl.get_position("left_hip_yaw_joint")
        rhy = ctrl.get_position("right_hip_yaw_joint")
        lhk = ctrl.get_position("left_hip_knee_joint")
        rhk = ctrl.get_position("right_hip_knee_joint")
        la = ctrl.get_position("left_ankle_joint")
        ra = ctrl.get_position("right_ankle_joint")
        print("left_hip_pitch_joint: ", lhp)
        print("right_hip_pitch_joint: ", rhp)
        print("left_hip_roll_joint: ", lhr)
        print("right_hip_roll_joint: ", rhr)
        print("left_hip_yaw_joint: ", lhy)
        print("right_hip_yaw_joint: ", rhy)
        print("left_hip_knee_joint: ", lhk)
        print("right_hip_knee_joint: ", rhk)
        print("left_ankle_joint: ", la)
        print("right_ankle_joint: ", ra)
        posz += 0.05
        time.sleep(0.1)

def get_start_pos():
    ctrl.set_mit_cmd("left_hip_roll_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("right_hip_roll_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("left_hip_pitch_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("right_hip_pitch_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("left_hip_yaw_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("right_hip_yaw_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("left_hip_knee_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("right_hip_knee_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("left_ankle_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    ctrl.set_mit_cmd("right_ankle_joint", 0.0, 0.0, 0.0, 10.0, 0.5)
    lhr = ctrl.get_position("left_hip_roll_joint")
    rhr = ctrl.get_position("right_hip_roll_joint")
    lhp = ctrl.get_position("left_hip_pitch_joint")
    rhp = ctrl.get_position("right_hip_pitch_joint")
    lhy = ctrl.get_position("left_hip_yaw_joint")
    rhy = ctrl.get_position("right_hip_yaw_joint")
    lhk = ctrl.get_position("left_hip_knee_joint")
    rhk = ctrl.get_position("right_hip_knee_joint")
    la = ctrl.get_position("left_ankle_joint")
    ra = ctrl.get_position("right_ankle_joint")
    time.sleep(0.05)
    return lhr, rhr, lhp, rhp, lhy, rhy, lhk, rhk, la, ra

def home_robot(kps, kds, duration_s=3.0, dt=0.02):
    ctrl.enable_all_actuator()
    time.sleep(0.1)
    init_pos = np.array([0.3, 0.0, 0.0, 0.7, 0.4, -0.3, 0.0, 0.0, -0.7, -0.4], dtype=np.double)
    """Slowly move to the default standing pose before running the policy."""
    print(f"[Home] Moving to default pose over {duration_s} s …")
    q0     = 0
    steps  = int(duration_s / dt)
    for k in range(steps + 1):
        alpha      = k / steps
        target_pos = (1 - alpha) * q0 + alpha * init_pos
        print(target_pos)
        send_commands(ctrl, target_pos, kps, kds)
        time.sleep(dt)
    time.sleep(0.1)
    print("[Home] Done. Inference will start in 15 s …")
    for i in range(0, 15):
        time.sleep(1)
        print(f"{15 - i} s …")
    
    
kps = np.array([ 80,  80,  30,  80,  50,  80,  80,  30,  80,  50], dtype=np.double)
kds = np.array([2.3, 2.3, 1.0, 2.3, 1.5, 2.3, 2.3, 1.0, 2.3, 1.5], dtype=np.double)
JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_knee_joint",
    "left_ankle_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_knee_joint",
    "right_ankle_joint",
]
#set_all_zero()
#ctrl.enable_all_actuator()
read_pos()
#home_robot(kps, kds)
time.sleep(0.1)
ctrl.disable_all_actuator()
ctrl.stop()