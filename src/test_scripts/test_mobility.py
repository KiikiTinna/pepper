import naoqi

motion_proxy = naoqi.ALProxy("ALMotion", "192.168.0.102", 9559)
motion_proxy.wakeUp()  # Wake up the robot if it's asleep
motion_proxy.moveToward(0.5, 0.0, 0.0)