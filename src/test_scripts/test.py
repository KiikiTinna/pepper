from naoqi import ALProxy

try:
    motion = ALProxy("ALMotion", "192.168.0.102", 9559)
    print("Connected to Pepper!")
except Exception as e:
    print("Error connecting to Pepper:", e)
