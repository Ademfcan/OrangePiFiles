from photonlibpy.photonCamera import PhotonCamera
import numpy as np
import ntcore as nt

instance = nt.NetworkTableInstance.getDefault()
instance.startClient4("Braindance Bearing")
instance.setServerTeam(488)
table = instance.getTable("SmartDashboard")
writeTable = table.getSubTable("Braindance")
confTargets = writeTable.getDoubleArrayTopic("Target Confidence").publish()
xTargets = writeTable.getDoubleArrayTopic("Target X").publish()
yTargets = writeTable.getDoubleArrayTopic("Target Y").publish()


VRES = 720
HRES = 1280
MIDH =  int(HRES/2)
# note 10 inch inside diameter 14 inch outside diameter
NOTEKNOWNSIZE = 13.5 #inches

FX = 896.4286516632743
HFOVRAD = 2 * np.arctan(HRES / (2 * FX))

# Convert FOV from radians to degrees
HFOVDEG = np.degrees(HFOVRAD)

def calculateDistance(knownSize, currentSizePixels, focalLength):
    return knownSize * focalLength / currentSizePixels


def calcDefOff(fov, res, pixelDiff):
    dperPixel = fov / res
    return pixelDiff * dperPixel
NORESULT = -9999
def processTarget(results):
    bearing = NORESULT
    noteRange = NORESULT

    if results != None and results[0] != None:
        boxes = results[0].boxes.xywh.cpu()
        confs = results[0].boxes.conf.cpu()
        confidences = []
        xs = []
        ys = []
        for box, conf in zip(boxes, confs):
            x,y,w,h = box
            

            diffX = x - MIDH
            bearing = calcDefOff(HFOVDEG, HRES, diffX)
            noteRange = calculateDistance(NOTEKNOWNSIZE,w,FX)
            targetX,targetY = turnBearingDistanceToXY(bearing,noteRange)
            confidences.append(conf)
            xs.append(targetX)
            ys.append(targetY)
        

    return confidences,xs,ys

def turnBearingDistanceToXY(bearing,range):
    return -10,-10







camera = PhotonCamera("testCam")
print(str(camera.getLatestResult().getTargets()))
# figure out how to get results
results = "blah blah blah"
confs,xs,ys = processTarget(results)
if(confs):
    confTargets.set(confs)
    xTargets.set(xs)
    yTargets.set(ys)

