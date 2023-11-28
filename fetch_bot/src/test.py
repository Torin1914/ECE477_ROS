import math

x = 600

y = math.atan2(math.sin(x), math.cos(x))

print(y)

print(math.radians(175/2))

print(math.pi/2)

c = 636.2999
frame_width = 1280.0
camera_fov_deg = 175.0
r = math.radians((c - (frame_width / 2)) / (frame_width / 2) * (camera_fov_deg / 2))
print(r)