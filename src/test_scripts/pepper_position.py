import qi  # For interacting with Pepper's APIs

# Connect to the Pepper robot (replace with the correct IP address)
session = qi.Session()
session.connect("tcp://192.168.0.102:9559")

# Get the motion proxy
motion_proxy = session.service("ALMotion")

# Get the robot's current position (x, y, theta)
current_position = motion_proxy.getRobotPosition(True)  # True = world frame, False = robot frame

# Print the current position of Pepper
x, y, theta = current_position
print("Pepper's current position: X = {:.2f}, Y = {:.2f}, Theta = {:.2f}".format(x, y, theta))
