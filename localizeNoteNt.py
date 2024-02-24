from photonlibpy.photonCamera import PhotonCamera
import numpy as np
import ntcore as nt
import math

instance = nt.NetworkTableInstance.getDefault()
instance.startClient4("Braindance Bearing")
instance.setServerTeam(488)
table = instance.getTable("SmartDashboard")
writeTable = table.getSubTable("Braindance")
confTargets = writeTable.getDoubleArrayTopic("Target Confidence").publish()
xTargets = writeTable.getDoubleArrayTopic("Target X").publish()
yTargets = writeTable.getDoubleArrayTopic("Target Y").publish()
zTargets = writeTable.getDoubleArrayTopic("Target Z").publish()


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
    radperPixel = fov / res
    return pixelDiff * radperPixel

def processTarget(results):
    if results != None and results[0] != None:
        boxes = results[0].boxes.xywh.cpu()
        confs = results[0].boxes.conf.cpu()
        targets = []
        for box, conf in zip(boxes, confs):
            x,y,w,h = box
            

            diffX = x - MIDH
            bearingRad = calcDefOff(HFOVRAD, HRES, diffX)
            noteRange = calculateDistance(NOTEKNOWNSIZE,w,FX)
            targetX,targetY,targetZ = turnBearingDistanceToXYZ(bearingRad,noteRange)
            targets.append([conf,targetX,targetY,targetZ])
        

    return targets

offsetsX = [14.25,-14.25,-13,13]
offsetsZ = [13.84,13.84,-13.24,-13.24]
rotXOffset = [80,280,188,172]
rotDownOffset = [-3,-3,-3.77,-3.77]


def turnBearingDistanceToXYZ(bearing, range, cameraIndex):
    dx = offsetsX[cameraIndex]
    dz = offsetsZ[cameraIndex]
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
                                 [0,1,0,0],
                                 [0,0,1,dz],
                                 [0,0,0,1]])
    

    # Calculate the final transformation matrix
    finalTransMatrix = rotMatrixZ @ rotMatrixY
    

    # Calculate object offset relative to camera
    
    camX = math.sin(bearing) * range
    camZ = math.cos(bearing) * range
    
    print(f"Relative to camera: x {camX} z {camZ}")

    relativeMatrix = np.array([[camX], [0], [camZ]])  # y is 0 because on the ground
    
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

def writeToNt(accumulatedResults):
    confs = []
    xs = []
    ys = []
    zs = []
    for target in accumulatedResults:
        conf = target[0] 
        confs.append(conf)
        x = target[1]
        xs.append(x)
        y = target[2]
        ys.append(y)
        z = target[3]
        zs.append(z)
    confTargets.set(confs)
    xTargets.set(xs)
    yTargets.set(ys)
    zTargets.set(zs)



    
# start waiting for photonvision camera updates and put on network tables
def startLoop():
    camera1 = PhotonCamera("testCam")
    camera2 = PhotonCamera("testCam")
    camera3 = PhotonCamera("testCam")
    camera4 = PhotonCamera("testCam")
    cams = [camera1,camera2,camera3,camera4]
    lastTimeStamps = [-100,-100,-100,-100]
    lastResults = [[],[],[],[]]
    
    
    while True:
        # todo get actual results
        hasChanged = False
        for i in range(len(cams)):
            currCam = cams[i]
            latestResult = currCam.getLatestResult()
            latestTimeStamp = latestResult.getTimestamp()
            prevTimeStamp = lastTimeStamps[i]
            if latestTimeStamp != prevTimeStamp:
                hasChanged = True
                # new result
                targets = processTarget(latestResult)
                lastResults[i] = targets
        
        if hasChanged:
            AccumulatedResults = []
            for result in lastResults:
                AccumulatedResults.append(result)
            writeToNt(AccumulatedResults)

                

if __name__ == "main":
    startLoop()
