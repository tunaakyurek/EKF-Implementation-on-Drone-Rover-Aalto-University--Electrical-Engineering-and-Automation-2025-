# connect_test.py
from robomaster import robot
import time

def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta')
    print("Connected to RoboMaster S1 at 172.20.10.11!")
    for i in range(5):
        pos = ep_robot.chassis.get_position()
        print(f"[{i}] x={pos[0]:.2f}, y={pos[1]:.2f}, yaw={pos[2]:.1f}")
        time.sleep(1)
    ep_robot.close()

if __name__ == '__main__':
    main()
