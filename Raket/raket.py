import bpy
import math

# Opening a file
path = open("testpath.txt", "r")
counter = 0
  
# Reading from file
Content = file.read()
CoList = Content.split("\n")
  
for i in CoList:
    if i:
        counter += 1

pathLength = counter
ctx = bpy.context
raket = bpy.data.objects['Cube']
raket.select_set(True) 

ctx.scene.frame_start = 0
ctx.scene.frame_end = (pathLength - 1)

raket.location.x = 0
raket.location.y = 0
raket.location.z = 0

  
for i in range(pathLength):
    currentLocation = path.readline()
    print(currentLocation)
    raket.location.y = 0
    raket.location.x = 0
    raket.location.z = 0   
    raket.keyframe_insert(data_path="location", frame=i)