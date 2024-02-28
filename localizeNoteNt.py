import photonlibpy as pv
import numpy as np
import ntcore as nt
import math
import socket

instance = nt.NetworkTableInstance.getDefault()
instance.startClient4("Braindance Bearing")
instance.setServerTeam(488)
table = instance.getTable("SmartDashboard")
writeTable = table.getSubTable("Braindance")
# find way to get confidence from photonvision
#confTargets = writeTable.getDoubleArrayTopic("Target Confidence").publish()
xyz = writeTable.getDoubleArrayTopic("Target Coordinate pairs").publish()



VRES = 720
HRES = 1280
MIDH =  int(HRES/2)
# note 10 inch inside diameter 14 inch outside diameter
NOTEKNOWNSIZE = 13.5 #inches

FX = 896.4286516632743 #mm
HFOVRAD = 2 * np.arctan(HRES / (2 * FX))

# Convert FOV from radians to degrees
HFOVDEG = np.degrees(HFOVRAD)

#Returns inches
def calculateDistance(knownSize, currentSizePixels, focalLength):
    return knownSize * focalLength / currentSizePixels


def calcDefOff(fov, res, pixelDiff):
    fovperPixel = fov / res
    return pixelDiff * fovperPixel


offsetsX = [-13.887,13.887,-12.957,12.957]
offsetsZ = [-13.779,-13.779,12.86,12.86]
offsetsHeight = [6.628,6.628,10.540,10.540]
rotXOffset = [280,80,188,172]
rotDownOffset = [-3,-3,-3.77,-3.77]


def turnBearingDistanceToXYZ(bearing, range, cameraIndex):
    dx = offsetsX[cameraIndex]
    dz = offsetsZ[cameraIndex]
    dHeight = offsetsHeight[cameraIndex]
    print(f"dx {dx} dz {dz}")
    a = math.radians(rotXOffset[cameraIndex])
    b = math.radians(rotDownOffset[cameraIndex])
    print(f"a {a} b {b}")


    # Define rotation matrices
    rotMatrixZ = np.array([[math.cos(a), -math.sin(a), 0],
                            [math.sin(a), math.cos(a), 0],
                            [0, 0, 1]])
    
    rotMatrixY = np.array([[math.cos(b), 0, math.sin(b)],
                            [0, 1, 0],
                            [-math.sin(b), 0, math.cos(b)]])
    
    translationMatrix = np.array([[1,0,0,dx],
                                 [0,1,0,offsetsHeight],
                                 [0,0,1,dz],
                                 [0,0,0,1]])
    

    # Calculate the final transformation matrix
    finalTransMatrix = rotMatrixZ @ rotMatrixY
    

    # Calculate object offset relative to camera
    
    camX = math.sin(bearing) * range
    camZ = math.cos(bearing) * range
    
    print(f"Relative to camera: x {camX} z {camZ}")

    relativeMatrix = np.array([[camX], 
                               [0], 
                               [camZ]])  # y is 0 because on the ground
    
    # Transform coordinates using the final transformation matrix
    finalMatrix = finalTransMatrix @ relativeMatrix
    rotX = finalMatrix[0][0]
    rotY = finalMatrix[1][0]
    rotZ = finalMatrix[2][0]

    relativeTransMatrix = np.array([[rotX],[rotY],[rotZ],[1]])
    actualFinalMatrix = translationMatrix @ relativeTransMatrix
    finalX = actualFinalMatrix[0][0]
    finalY = actualFinalMatrix[1][0]
    finalZ = actualFinalMatrix[2][0]
    return finalX,finalY,finalZ

# uneccesary function unless we also add confidences
# def writeToNt(accumulatedResults):
#     confs = []
#     xyzTriplets = []
#     for target in accumulatedResults:
#         conf = target[0] 
#         confs.append(conf) 
#     xyz.set(xyzTriplets)
    



    
# start waiting for photonvision camera updates and put on network tables
def startLoop():
    name = socket.getHostName()
    cameraIndex = None
    match name:
        case "FL":
            cameraIndex = 0
        case "FR":
            cameraIndex = 1
        case "RL":
            cameraIndex = 2
        case "RR":
            cameraIndex = 3
    camera = pv.PhotonCamera(f"Camera{name}")
    
    
    while True:
        # todo get actual results
        
        latestResult = camera.getLatestResult()
        if latestResult.hasTargets():
            hasChanges = True
            targetsYXZ = []
            targetList = latestResult().getTargets()
            # sorting targets based on what takes up the most areas
            sortedList = sorted(targetList,key=lambda target : target.getArea())
            
            for target in sortedList:
                # corner coordiates are unsorted so need to extract width and height
                corners = target.getDetectedCorners()
                minX = min(x for x, y in corners)
                minY = min(y for x, y in corners)
                maxX = max(x for x, y in corners)
                maxY = max(y for x, y in corners)
                widthX = maxX-minX
                heightY = maxY-minY
                centerX = minX + widthX/2.0
                # center y not needed right now
                centerY = minY + heightY/2.0
                distance = calculateDistance(NOTEKNOWNSIZE,widthX,FX)
                bearing = calcDefOff(HFOVRAD,HRES,centerX-MIDH)
                x,y,z = turnBearingDistanceToXYZ(bearing,distance,cameraIndex)
                targetsYXZ.append(f"{y*-1},{x},{z}")
            xyz.set(targetsYXZ)
        else:
            xyz.set([])
        # if this takes up to much cpu might want to add a sleep
        
                

if __name__ == "main":
    startLoop()
    
