from robomaster import robot

ep = robot.Robot()
ep.initialize(conn_type="ap", ip="192.168.2.1")

print("POS:", ep.chassis.get_position())   # (x, y, yaw_deg)
print("ATT:", ep.chassis.get_attitude())   # (pitch, roll, yaw_deg)

ep.close()
