import bpy
import math
import os

#change working directory to directory with the data file
#cwd = os.getcwd()
#print(cwd)
os.chdir(r'C:\Users\JeanK\OneDrive\Bureaublad\Cansat_code\Cansat_AlphaSpot\Raket')

# Opening a file
data = open('data.txt', 'r')
counter = 0
  
# Reading from file
Content = data.read()
CoList = Content.split("\n")
  
x = 0

bpy.context.scene.render.fps = 4
bpy.context.scene.render.fps_base = 1

for i in CoList:
    if i:
        counter += 1

dataLength = counter
ctx = bpy.context
raket = bpy.data.objects['Cube.001']
raket.select_set(True) 

ctx.scene.frame_start = 0
ctx.scene.frame_end = (dataLength - 1)

raket.location.x = 0
raket.location.y = 0
raket.location.z = 0

  
for i in range(dataLength):
    if(i != 0):
        currentLocation = CoList[i]
        previousLocation = CoList[i-1]
        
        print(currentLocation)
        print(previousLocation)
        
        currentCoordinates  = currentLocation.split(",")
        previousCoordinates = previousLocation.split(",")
        
        print(currentCoordinates)
        print(previousCoordinates)
        
        altitude  = float(currentCoordinates[2])
        
        print(altitude)
        
        longitude = float(currentCoordinates[0]) - float(previousCoordinates[0])
        latitude = float(currentCoordinates[1]) - float(previousCoordinates[1])
        
        print(longitude)
        print(latitude)
        
        y_axis = float(2 * (math.pi) * 6371000) * (longitude/360)
        print(y_axis)
        x_axis = float(2 * (math.pi) * 6371000) * (latitude/360)
        print(x_axis)
        z_axis = float(altitude)
        print(z_axis)
        
    else:
        y_axis = 0.00
        x_axis = 0.00
        z_axis = 0.00
    
    raket.location.x = x_axis
    raket.location.y = y_axis
    raket.location.z = z_axis
    raket.keyframe_insert(data_path="location", frame=i)