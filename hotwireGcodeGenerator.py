#!/usr/bin/env python3
# requires svgpathtools, install it like this: pip3 install svgpathtools

# converts a list of path elements of a SVG file to simple line drawing commands
from svgpathtools import svg2paths
from svgpathtools import Path, Line, Arc, CubicBezier, QuadraticBezier
from xml.dom import minidom

#imports for using matplotlib with tkinter
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backend_bases import MouseEvent
from matplotlib.figure import Figure

import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from tkinter import ttk

from math import sqrt, floor, ceil
import numpy as np

import time

def slicePath(path, step): 
    anchorPoints = np.empty((0,2), float)
    pointCloud = np.empty((0,2), float)
    
    for e in path:
        x0 = e.start.real
        y0 = e.start.imag
        x1 = e.end.real
        y1 = e.end.imag
        #print("start&end: (%.2f, %.2f) - (%.2f, %.2f)" % (x0, y0, x1, y1))
        
        #create a finely granulated point cloud for each element (line, arc, etc)
        length = e.length()
        noPoints = length / step
        
        #print("noPoints: " + str(noPoints))
        #print("element length: " + str(e.length()))
        
        #extract anchor points
        startPoint = [e.start.real, e.start.imag]
        if not any(np.equal(anchorPoints, startPoint).all(1)) :
            anchorPoints = np.append(anchorPoints, [startPoint], axis = 0)
            
        endPoint = [e.end.real, e.end.imag]
        if not any(np.equal(anchorPoints, endPoint).all(1)) :
            anchorPoints = np.append(anchorPoints, [endPoint], axis = 0)
        
        #add points in steps of displayStep
        for i in range(int(noPoints)) :
            #add next point
            pointCloud=np.append(pointCloud, [[e.point(i * step / length).real, e.point(i * step / length).imag]], axis = 0)

        i = int(noPoints)
        if (i * step < length) : #if the pointClound isn't "full"
            #add the last point of the line
            pointCloud=np.append(pointCloud, [[e.point(i * step / length).real, e.point(i * step / length).imag]], axis = 0)
            
        #print(pointCloud)
    return (pointCloud, anchorPoints)

#gets the parameters needed for putting an arrow at a specific point of a pointcloud in matplotlib
def getArrowAtPoint(pointCloud, point, length):
    index = np.where(np.all(pointCloud==point,axis=1))[0][0]
    
    incIndex = index + 1
    if (incIndex > len(pointCloud) - 1): #wrap around
        incIndex = 0
    x = pointCloud[index][0]
    y = pointCloud[index][1]
    dx = pointCloud[incIndex][0]-pointCloud[index][0]
    dy = pointCloud[incIndex][1]-pointCloud[index][1]
    dlength = sqrt(dx**2 + dy**2)
    
    dx = dx * length/dlength
    dy = dy * length/dlength
    
    return (x, y, dx, dy)

#get the length of the curve between two anchor points
def getLengthBetweenAnchors(path, anchorPoint1, anchorPoint2) : 
    startIndex = -1
    stopIndex = -1

    for (i,e) in enumerate(path):
        if (e.start.real == anchorPoint1[0]) and (e.start.imag == anchorPoint1[1]) :
            startIndex = i
        if (e.end.real == anchorPoint2[0]) and (e.end.imag == anchorPoint2[1]) :
            stopIndex = i
    
    
    if (startIndex < 0) or (stopIndex <0):
        print("problem with finding Anchor points in the path")
        return
     
    #calculate length
    length = 0  
    
    if (startIndex > stopIndex) : #the two anchor points wrap around the list
        noElements = len(path)
        for j in range(startIndex, noElements):
            length = length + path[j].length()
        for j in range(0, stopIndex+1):
            length = length + path[j].length()
        
    else:
        for j in range(startIndex, stopIndex + 1):
            length = length + path[j].length()
           
    return length


#interpolates a point between a point in the point cloud
def interpolate(pointCloud, index) :
    cIndex = ceil(index)
    fIndex = floor(index)
    if (cIndex == len(pointCloud)) : # wraparound case
        x = (pointCloud[0, 0] + pointCloud[fIndex, 0]) / 2
        y = (pointCloud[0, 1] + pointCloud[fIndex, 1]) / 2
    else :
        x = (pointCloud[cIndex, 0] + pointCloud[fIndex, 0]) / 2
        y = (pointCloud[cIndex, 1] + pointCloud[fIndex, 1]) / 2
    
    return np.array([x,y])

#interpolates the point clouds along the longer axis for each segment between two anchor points
def interpolateBetweenAnchors(xyPath, xyPointCloud, xyAnchorPoint1, xyAnchorPoint2, uvPath, uvPointCloud, uvAnchorPoint1, uvAnchorPoint2, step):
    
    def getAnchorIndex(pointCloud, anchor):
        return np.where(np.all(pointCloud == anchor, axis = 1))[0][0]       

    def interpolateOnLongest(longPointCloud, longAnchorPoint1, longAnchorPoint2, longLength, shortPointCloud, shortAnchorPoint1, shortAnchorPoint2, shortLength, step):
        longAnchor1index = getAnchorIndex(longPointCloud, longAnchorPoint1)
        longAnchor2index = getAnchorIndex(longPointCloud, longAnchorPoint2)
        shortAnchor1index = getAnchorIndex(shortPointCloud, shortAnchorPoint1)
        shortAnchor2index = getAnchorIndex(shortPointCloud, shortAnchorPoint2)
        
        #rotate point clouds for more conveinient interpolation
        longPointCloud = np.roll(longPointCloud, -longAnchor1index, axis = 0)
        shortPointCloud = np.roll(shortPointCloud, -shortAnchor1index, axis = 0)
        #get the new top values
        longLast = longAnchor2index - longAnchor1index
        shortLast = shortAnchor2index - shortAnchor2index
        while (longLast < 0):
            longLast += len(longPointCloud)
        while (shortLast <0):
            shortLast += len(shortPointCloud)
        
        #steps on the index of the array
        longStep = step * longLast / longLength
        shortStep = longStep * shortLength / longLength
        
        #the new empty arrays
        longInterpolated = np.empty((0,2), float)
        shortInterpolated = np.empty((0,2), float)
        
        longIndex = 0
        shortIndex = 0

        while (longIndex < longLast):
            longInterpolated = np.append(longInterpolated, [interpolate(longPointCloud, longIndex)], axis = 0)
            shortInterpolated = np.append(shortInterpolated, [interpolate(shortPointCloud, shortIndex)], axis = 0)
            #increment indices
            longIndex += longStep
            shortIndex += shortStep

        return (longInterpolated, shortInterpolated)
        
    xyLength = getLengthBetweenAnchors(xyPath, xyAnchorPoint1, xyAnchorPoint2)
    uvLength = getLengthBetweenAnchors(uvPath, uvAnchorPoint1, uvAnchorPoint2)
    
    if (xyLength > uvLength):
        (xyInterpolated, uvInterpolated) = interpolateOnLongest(xyPointCloud, xyAnchorPoint1, xyAnchorPoint2, xyLength, uvPointCloud, uvAnchorPoint1, uvAnchorPoint2, uvLength, step)
    else:
        (uvInterpolated, xyInterpolated) = interpolateOnLongest(uvPointCloud, uvAnchorPoint1, uvAnchorPoint2, uvLength, xyPointCloud, xyAnchorPoint1, xyAnchorPoint2, xyLength, step)
        
    return (xyInterpolated, uvInterpolated)

#reslice the path between the anchor points
def slicePathAnchorPoints(xyPath, xyAnchorPoints, uvPath, uvAnchorPoints, step) :
    granularXY = slicePath(xyPath, 0.05) #slice the path in a very fine way to be interpolated later
    granularUV = slicePath(uvPath, 0.05)
    
    slicedXY = np.empty((0,2), float)
    slicedUV = np.empty((0,2), float)
    
    #go through the anchor points with slicing
    for i in range(len(xyAnchorPoints)-1):
        (appendXY, appendUV) = interpolateBetweenAnchors(xyPath, granularXY[0], xyAnchorPoints[i], xyAnchorPoints[i+1], uvPath, granularUV[0], uvAnchorPoints[i], uvAnchorPoints[i+1], step)
        slicedXY = np.append(slicedXY, appendXY, axis = 0)
        slicedUV = np.append(slicedUV, appendUV, axis = 0)
        
    #wrap around
    (appendXY, appendUV) = interpolateBetweenAnchors(xyPath, granularXY[0], xyAnchorPoints[-1], xyAnchorPoints[0], uvPath, granularUV[0], uvAnchorPoints[-1], uvAnchorPoints[0], step)
    slicedXY = np.append(slicedXY, appendXY, axis = 0)
    slicedUV = np.append(slicedUV, appendUV, axis = 0)
    
    return (slicedXY, slicedUV)

#convert the pointsclouds slicedXY and slicedUV into gcode with a feedrate
def translateToGcode(slicedXY, slicedUV, feedrate):
    gcode = ""
    gcode += "G28\n" #home
    gcode += "M3\n"   #turn on hotwire
    gcode += "G04 P2\n" #2 seconds pause for the hotwire to heat up
    gcode += "G90\n"  #absolute mode
    #all numeric values are rounded to 4 digits, not to overthrow the machine
    gcode += "G01" + " X00" + " Y" + str(round(slicedXY[0][1], 4)) + " U00" + " V" + str(round(slicedUV[0][1], 4)) + " F" + str(feedrate) + "\n" #move to start position vertically first
    
    for i in range(len(slicedXY)):
        gcode += "G01" + " X" + str(round(slicedXY[i][0], 4)) + " Y" + str(round(slicedXY[i][1], 4)) + " U" + str(round(slicedUV[i][0], 4)) + " V" + str(round(slicedUV[i][1], 4)) + " F" + str(feedrate) + "\n" #move to next point
        
    gcode += "G01" + " X" + str(round(slicedXY[0][0], 4)) + " Y" + str(round(slicedXY[0][1], 4)) + " U" + str(round(slicedUV[0][0], 4)) + " V" + str(round(slicedUV[0][1], 4)) + " F" + str(feedrate) + "\n" #close the path
    
    gcode += "G01" + " X00" + " Y" + str(round(slicedXY[0][1], 4)) + " U00" + " V" + str(round(slicedUV[0][1], 4)) + " F" + str(feedrate) + "\n" #pull wire back out     
    gcode += "G04 P5\n" #5 seconds pause to finish any lagging in the wire
    gcode += "M5\n"   #turn off hotwire
    
    return gcode

def extractSvg(svgToParse, displayStep = 0.5):

    # read the SVG file
    (path,dstring) = svg2paths(svgToParse)
    
    #slice the path. this is implemented for svg with only one path - svg2paths extracts the 1 layer svg into a whole path. elements can be unarranged
    if(path):
        path = mergePath(path)
        #path = parse_path(path_strings[0])
        (pointCloud, anchorPoints) = slicePath(path, displayStep)            
            
        return (pointCloud, anchorPoints, path)
    return "NO APPROPRIATE SVG PROVIDED"

#sort path in a way that connects all elements/subpaths in a closed figure
def mergePath(path):
    sortedPath = Path()
    sortedPath.append(path.pop(0)[0])
    
    while (len(path) != 0):
        noneFound = True
        for (i, element) in enumerate(path):
            if (sortedPath[-1].end == element[0].start):
                sortedPath.append(path.pop(i)[0])
                noneFound = False
                break
            
            if (sortedPath[-1].end == element[0].end):
                sortedPath.append(path.pop(i)[0].reversed())
                noneFound = False
                break
        
        if noneFound:
            print("error in svg: not a closed figure")
    
    return sortedPath
    
#find the closest point to startPoint in pointCloud
def findClosestPoint(startPoint, pointCloud):
    closestDistance = -1 # set closest distance negative to indicate it hasnt been set
    index = -1 # same with the index of the point
    for i, point in enumerate(pointCloud):
        distance = sqrt((point[0] - startPoint[0])**2 + (point[1] - startPoint[1])**2)
        if (distance < closestDistance) or (closestDistance < 0) :
            closestPoint = point
            index = i
            closestDistance = distance
        
    return closestPoint, index
    
#calculate actual points for the machine's axes considering a gantry length and placement of the foam in it
def calcToolPointClouds(xyPointCloud, uvPointCloud, gantryLength, foamWidth, distanceToXYaxis):
    xyToolPointCloud = np.empty((0,2), float)
    uvToolPointCloud = np.empty((0,2), float)
    for i in range(len(xyPointCloud)):
        #extract coordinates
        x = xyPointCloud[i][0]
        y = xyPointCloud[i][1]
        u = uvPointCloud[i][0]
        v = uvPointCloud[i][1]
        
        #calculate the coordinates on the tool axes (usage of similar triangles)
        xt = x - (u-x) * distanceToXYaxis / foamWidth
        ut = xt + (u-x) * gantryLength / foamWidth
        yt = y - (v-y) * distanceToXYaxis / foamWidth
        vt = yt + (v-y) * gantryLength / foamWidth
        
        #append to point cloud
        xyToolPointCloud = np.append(xyToolPointCloud, [[xt, yt]], axis = 0)
        uvToolPointCloud = np.append(uvToolPointCloud, [[ut, vt]], axis = 0)
    
    return (xyToolPointCloud, uvToolPointCloud)

class gcodeGeneratorApp(tk.Tk):

    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self, *args, **kwargs)

        #iconpath = str(os.path.dirname(os.path.realpath(__file__)))+os.path.sep+'stonetronicsLogo.png'
        #tk.Tk.iconphoto(True, tk.PhotoImage(file=iconpath))
        tk.Tk.wm_title(self, "XYUV profile gcode generator")

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand = True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        
        #svg file chooser
        svgChooser = tk.Frame(container)
        svgChooser.pack()
        fileChooserFrame = tk.Frame(svgChooser)
        fileChooserFrame.pack(side = tk.LEFT)
        xyFileChooser = FileChooser(fileChooserFrame, "XY svg File:")
        xyFileChooser.pack(side = tk.TOP)
        uvFileChooser = FileChooser(fileChooserFrame, "UV svg File:")
        uvFileChooser.pack(side = tk.BOTTOM)
        def loadFiles():
            (xyPc, self.xyAp, self.xyPath) = extractSvg(xyFileChooser.getFilePath())
            (uvPc, self.uvAp, self.uvPath) = extractSvg(uvFileChooser.getFilePath()) 
            self.anchorPointWidget.resetSelectedAnchorPoints()
            #print (xyPc)
            #print (uvPc)
            self.anchorPointWidget.updateData(xyPc, self.xyAp, uvPc, self.uvAp)
        loadButton = tk.Button(svgChooser, text = "load", command = loadFiles)
        loadButton.pack(side = tk.RIGHT)
        
        #anchor points definement
        self.anchorPointWidget = AnchorPointWidget(container)
        self.anchorPointWidget.pack(side="top",fill='both',expand=True)

        # "go" button
        def openGeneration():
            if (not hasattr(self, 'xyPath')) or (not hasattr(self, 'uvPath')) :
                messagebox.showerror("load files first!", "Please load files and select anchor points before moving on to the generation")
                return
            (self.xySelAp, self.uvSelAp) = self.anchorPointWidget.getSelectedAnchorPoints()
            noXySelAp = len(self.xySelAp)
            noUvSelAp = len(self.uvSelAp)
            if (noXySelAp != noUvSelAp) or (noXySelAp < 2) or (noUvSelAp < 2):
                messagebox.showerror("improper anchor points!", "-Please select an equal number of anchor points on both planes\n-There need to be at least 2 anchor points on each axis")
                return
            data = (self.xyPath, self.xyAp, self.xySelAp, self.uvPath, self.uvAp, self.uvSelAp)
            generationWindow = GenerationWindow(data)
        goButton = tk.Button(container, text = "generate!", command = openGeneration)
        goButton.pack()

class FileChooser(tk.Frame):
           
    def __init__(self, parent, labeltext = "File:", types = (("svg files","*.svg"), ("all files","*.*")) ):
        tk.Frame.__init__(self, parent)
        self.types = types
        
        self.filepathLabel = tk.Label(self, text = labeltext)
        self.filepathLabel.grid(row = 0, column = 0)
        
        self.filepathEntry = tk.Entry(self)
        self.filepathEntry.grid(row = 0, column = 1)
        
        self.browseButton = tk.Button(self, text = "browse...", command = self.browseFile)
        self.browseButton.grid(row = 0, column = 2)

    def browseFile(self):
        tmp = filedialog.askopenfilename(initialdir = "./", title = "select svg file", filetypes = self.types)
        self.filepathEntry.delete(0, tk.END) #delete and overwrite the set filename
        self.filepathEntry.insert(tk.INSERT, tmp)     
        self.filepathEntry.xview("end") #scroll to the back so the filename can be read better      
    
    def getFilePath(self):
        return self.filepathEntry.get()

        
        
class AnchorPointWidget(tk.Frame):

    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        
        self.clickMode = "none"
                
        self.selectedXYAnchorPoints = np.empty((0,2), float)
        self.selectedUVAnchorPoints = np.empty((0,2), float)
             
        self.header = tk.Frame(self)
        self.header.pack(side = tk.TOP)
        
        def setClickMode(mode):
            self.clickMode = mode

        self.clickModeXYButton = tk.Button(self.header, text = "select XY anchor points", command = lambda: setClickMode("XY"))
        self.clickModeXYButton.pack(side = tk.LEFT)
        self.clickModeUVButton = tk.Button(self.header, text = "select UV anchor points", command = lambda: setClickMode("UV"))
        self.clickModeUVButton.pack(side = tk.RIGHT)

        self.f = Figure(figsize=(8,8), dpi=100)
        self.a = self.f.add_subplot(111)
        self.a.plot([[0]], [[0]], 'r') #plot only origin point at first
        self.a.plot([[0]], [[0]], 'g')  
        self.a.axis("equal")    
        self.a.grid(color='grey', linestyle='-', linewidth=0.5)
        self.a.legend(["XY", "UV"])  

        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        
        def mouseClickCallback(event):
            if (not event.xdata) or (not event.ydata):
                #click outside of the diagram => reset the click mode 
                self.clickMode = "none"
                return
            clickPoint = np.array([event.xdata, event.ydata])
            
            if (self.clickMode == "XY"):
                (closestPoint,index) = findClosestPoint(clickPoint, self.xyAnchorPoints)
                if not any(np.equal(self.selectedXYAnchorPoints,[closestPoint]).all(1)) :
                    self.selectedXYAnchorPoints = np.append(self.selectedXYAnchorPoints, [closestPoint], axis = 0)
                else:
                    index = np.where(np.all(self.selectedXYAnchorPoints == closestPoint, axis = 1))[0][0]
                    self.selectedXYAnchorPoints = np.delete(self.selectedXYAnchorPoints, index, axis = 0)
                self.updateDisplay()
            elif (self.clickMode == "UV") :
                (closestPoint,index) = findClosestPoint(clickPoint, self.uvAnchorPoints)
                if not any(np.equal(self.selectedUVAnchorPoints,[closestPoint]).all(1)) :
                    self.selectedUVAnchorPoints = np.append(self.selectedUVAnchorPoints, [closestPoint], axis = 0)
                else:
                    index = np.where(np.all(self.selectedUVAnchorPoints == closestPoint, axis = 1))[0][0]
                    self.selectedUVAnchorPoints = np.delete(self.selectedUVAnchorPoints, index, axis = 0)
                self.updateDisplay()
            #else :
            
        self.canvas.mpl_connect ('button_press_event', mouseClickCallback)
        
        #toolbar = NavigationToolbar2TkAgg(self.canvas, self)
        #toolbar.update()
        self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    def updateData(self, xyPointCloud, xyAnchorPoints, uvPointCloud, uvAnchorPoints):
        self.xyPointCloud = xyPointCloud
        self.xyAnchorPoints = xyAnchorPoints
        self.uvPointCloud = uvPointCloud
        self.uvAnchorPoints = uvAnchorPoints
        self.updateDisplay()
        
    def updateDisplay(self):
        #clear
        self.a.clear()
        #plot point clouds
        self.a.plot(self.xyPointCloud[:,0], self.xyPointCloud[:,1], 'r')
        self.a.plot(self.uvPointCloud[:,0], self.uvPointCloud[:,1], 'g') 
        #plot anchor points
        self.a.scatter(self.selectedXYAnchorPoints[:,0], self.selectedXYAnchorPoints[:,1], c = 'r')
        for i, point in enumerate(self.selectedXYAnchorPoints):
            self.a.annotate(i+1, point)   
        self.a.scatter(self.selectedUVAnchorPoints[:,0], self.selectedUVAnchorPoints[:,1], c = 'g')
        for i, point in enumerate(self.selectedUVAnchorPoints):
            self.a.annotate(i+1, point)  
        #plot lines between anchorpoints (only if there has been at np.empty((0,2), float)least one tuple of anchor points selected)
        if ( (len(self.selectedUVAnchorPoints) > 0) and (len(self.selectedXYAnchorPoints) > 0) ):
            if ( len(self.selectedXYAnchorPoints) < len(self.selectedUVAnchorPoints) ):
                minNoSelectedAnchorPoints = len(self.selectedXYAnchorPoints)
            else:
                minNoSelectedAnchorPoints = len(self.selectedUVAnchorPoints)
            for i in range(minNoSelectedAnchorPoints):
                xCoordinates = [ self.selectedXYAnchorPoints[i][0], self.selectedUVAnchorPoints[i][0] ]
                yCoordinates = [ self.selectedXYAnchorPoints[i][1], self.selectedUVAnchorPoints[i][1] ]
                self.a.plot(xCoordinates, yCoordinates, 'grey')
                
        #visual settings
        self.a.axis("equal")    
        self.a.grid(color='grey', linestyle='-', linewidth=0.5)
        self.a.legend(["XY", "UV"])  
        self.canvas.draw()
        
    def getSelectedAnchorPoints(self):
        return (self.selectedXYAnchorPoints, self.selectedUVAnchorPoints)
        
    def resetSelectedAnchorPoints(self):
        self.selectedXYAnchorPoints = np.empty((0,2), float)
        self.selectedUVAnchorPoints = np.empty((0,2), float)
        
class MachineGeometryDisplayWindow(tk.Toplevel):
    def __init__(self):
        tk.Toplevel.__init__(self)
        self.image = tk.PhotoImage(file = "machineGeometry.png").subsample(3,3)
        self.disp = tk.Label(self, image = self.image)
        self.disp.pack()
        
class GenerationWindow(tk.Toplevel):
    def __init__(self, data):
        tk.Toplevel.__init__(self)
        
        self.generationWidget = GenerationWidget(self, data)
        self.generationWidget.pack()
        
        self.outputFileFrame = tk.Frame(self)
        self.outputFileFrame.pack()
        self.outputfileChooser = FileSaveChooser(self.outputFileFrame, labeltext = "output file:")
        self.outputfileChooser.pack(side = tk.LEFT)
        def saveGcode():
            f = open(self.outputfileChooser.getFilePath(), "w")
            feedrate = self.generationWidget.FeedrateSpinbox.get()
            f.write(translateToGcode(*self.generationWidget.getToolPoints(), feedrate))
            f.close()
        self.outputfileSaveButton = tk.Button(self.outputFileFrame, text = "save", command = saveGcode)
        self.outputfileSaveButton.pack(side = tk.RIGHT)
        
        self.closeButton = tk.Button(self, text = "close", command = self.destroy)
        self.closeButton.pack()
        
class FileSaveChooser(tk.Frame):
           
    def __init__(self, parent, labeltext = "File:", types = (("gcode files","*.gcode"), ("all files","*.*")) ):
        tk.Frame.__init__(self, parent)
        self.types = types
        
        self.filepathLabel = tk.Label(self, text = labeltext)
        self.filepathLabel.grid(row = 0, column = 0)
        
        self.filepathEntry = tk.Entry(self)
        self.filepathEntry.grid(row = 0, column = 1)
        
        self.browseButton = tk.Button(self, text = "browse...", command = self.browseFile)
        self.browseButton.grid(row = 0, column = 2)
               
    def browseFile(self):
        f = filedialog.asksaveasfile(parent = self.winfo_toplevel(), initialdir = "./", title = "select output file", filetypes = self.types)
        tmp = f.name 
        f.close()
        self.filepathEntry.delete(0, tk.END) #delete and overwrite the set filename
        self.filepathEntry.insert(tk.INSERT, tmp)     
        self.filepathEntry.xview("end") #scroll to the back so the filename can be read better      
    
    def getFilePath(self):
        return self.filepathEntry.get()
        
        
class GenerationWidget(tk.Frame):
    def __init__(self, parent, data):
        tk.Frame.__init__(self, parent)
        self.data = data
               
        #settings box
        self.settingsFrame = tk.Frame(self)
        self.settingsFrame.pack(side = tk.RIGHT)
        
        #gantry length spinbox
        self.gantryLengthSpinboxLabel = tk.Label(self.settingsFrame, text = "Gantry length:")
        self.gantryLengthSpinboxLabel.pack()
        self.gantryLengthSpinbox = tk.Spinbox(self.settingsFrame, from_ = 0, to = 10000, increment=1) # arbitrarily large from/to values for using spinbox as number input
        self.gantryLengthSpinbox.delete(0, "end")
        self.gantryLengthSpinbox.insert(0, '1000') #default val
        self.gantryLengthSpinbox.pack()
        
        #foam width spinbox
        self.foamWidthSpinboxLabel = tk.Label(self.settingsFrame, text = "Foam width:")
        self.foamWidthSpinboxLabel.pack()
        self.foamWidthSpinbox = tk.Spinbox(self.settingsFrame, from_ = 0, to = 10000, increment=1) # arbitrarily large from/to values for using spinbox as number input
        self.foamWidthSpinbox.delete(0, "end")
        self.foamWidthSpinbox.insert(0, '500') #default val
        self.foamWidthSpinbox.pack()
        
        #distance from foam to xy axes spinbox
        self.distanceToXYSpinboxLabel = tk.Label(self.settingsFrame, text = "distance Foam - XY axes:")
        self.distanceToXYSpinboxLabel.pack()
        self.distanceToXYSpinbox = tk.Spinbox(self.settingsFrame, from_ = 0, to = 10000, increment=1) # arbitrarily large from/to values for using spinbox as number input
        self.distanceToXYSpinbox.delete(0, "end")
        self.distanceToXYSpinbox.insert(0, '25') #default val
        self.distanceToXYSpinbox.pack()
        
        #button for showing the machine geometry
        self.recalcBtn = tk.Button(self.settingsFrame, text = "machine geometry", command = lambda: MachineGeometryDisplayWindow())
        self.recalcBtn.pack() 
        
        #granularity slider
        self.granularityLabel = tk.Label(self.settingsFrame, text = "granularity [mm] :")
        self.granularityLabel.pack()
        self.granularitySlider = tk.Scale(self.settingsFrame, from_ = 0.1, to=3, orient=tk.HORIZONTAL, resolution = 0.1)
        self.granularitySlider.set(0.5)
        self.granularitySlider.pack()
        
        #spinboxes for y and v offset
        self.YSpinboxLabel = tk.Label(self.settingsFrame, text = "Y Offset:")
        self.YSpinboxLabel.pack()
        self.YSpinbox = tk.Spinbox(self.settingsFrame, from_ = -10000, to = 10000, increment=1) # arbitrarily large from/to values for using spinbox as number input
        self.YSpinbox.delete(0, "end")
        self.YSpinbox.insert(0, '0') #default val
        self.YSpinbox.pack()
        self.VSpinboxLabel = tk.Label(self.settingsFrame, text = "V Offset:")
        self.VSpinboxLabel.pack()
        self.VSpinbox = tk.Spinbox(self.settingsFrame, from_ = -10000, to = 10000, increment=1)
        self.VSpinbox.delete(0, "end")
        self.VSpinbox.insert(0, '0') #default val
        self.VSpinbox.pack()  
        
        #spinboxes for x and u offset
        self.XSpinboxLabel = tk.Label(self.settingsFrame, text = "X Offset:")
        self.XSpinboxLabel.pack()
        self.XSpinbox = tk.Spinbox(self.settingsFrame, from_ = -10000, to = 10000, increment=1) # arbitrarily large from/to values for using spinbox as number input
        self.XSpinbox.delete(0, "end")
        self.XSpinbox.insert(0, '0') #default val
        self.XSpinbox.pack()
        self.USpinboxLabel = tk.Label(self.settingsFrame, text = "U Offset:")
        self.USpinboxLabel.pack()
        self.USpinbox = tk.Spinbox(self.settingsFrame, from_ = -10000, to = 10000, increment=1)
        self.USpinbox.delete(0, "end")
        self.USpinbox.insert(0, '0') #default val
        self.USpinbox.pack()
        
        #checkboxes for inverting the direction of the points
        self.reverseXY = ttk.Checkbutton(self.settingsFrame, text = "reverse XY direction")
        self.reverseXY.state(['!disabled','!alternate'])
        self.reverseXY.pack()
        self.reverseUV = ttk.Checkbutton(self.settingsFrame, text = "reverse UV direction")
        self.reverseUV.state(['!disabled','!alternate'])
        self.reverseUV.pack()
        
        #spinbox for the feedrate
        self.FeedrateSpinboxLabel = tk.Label(self.settingsFrame, text = "Feedrate:")
        self.FeedrateSpinboxLabel.pack()
        self.FeedrateSpinbox = tk.Spinbox(self.settingsFrame, from_ = 0, to = 200, increment=0.1)
        self.FeedrateSpinbox.delete(0, "end")
        self.FeedrateSpinbox.insert(0, '5') #default val
        self.FeedrateSpinbox.pack()
        
        #button for triggering recalculation
        self.recalcBtn = tk.Button(self.settingsFrame, text = "recalculate!", command = self.updateDisplay)
        self.recalcBtn.pack()      
        
        #the figure itself
        self.f = Figure(figsize=(8,8), dpi=100)
        self.a = self.f.add_subplot(111)
        self.a.plot([[0]], [[0]], 'r') #plot only origin point at first
        self.a.plot([[0]], [[0]], 'g')  
        self.a.axis("equal")    
        self.a.grid(color='grey', linestyle='-', linewidth=0.5)
        self.a.legend(["XY", "UV"])  

        self.canvas = FigureCanvasTkAgg(self.f, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side = tk.LEFT, expand=True)
        
    def calculate(self):
        #reverse the paths if necessary
        if (self.reverseXY.instate(['selected']) ):
            self.xyUsedPath = self.data[0].reversed()
        else:
            self.xyUsedPath = self.data[0]
        if (self.reverseUV.instate(['selected']) ):
            self.uvUsedPath = self.data[3].reversed()
        else:
            self.uvUsedPath = self.data[3]
        #get the step for slicing from the slider
        step = self.granularitySlider.get()   
        #slice the path
        (self.xyPoints, self.uvPoints) = slicePathAnchorPoints(self.xyUsedPath, self.data[2], self.uvUsedPath, self.data[5], step)
        #get anchor point indices
        self.anchorIndices = []
        for i in range(len(self.data[2])): #go through all anchor points
            (point,index) = findClosestPoint(self.data[2][i], self.xyPoints)
            self.anchorIndices.append(index)
        #add offsets
        self.xyPoints[:,0] += float(self.XSpinbox.get())
        self.xyPoints[:,1] += float(self.YSpinbox.get())
        self.uvPoints[:,0] += float(self.USpinbox.get())
        self.uvPoints[:,1] += float(self.VSpinbox.get())
        #calculate points for the tools
        
        (self.xyToolPoints, self.uvToolPoints) = calcToolPointClouds(self.xyPoints, self.uvPoints, float(self.gantryLengthSpinbox.get()) , float(self.foamWidthSpinbox.get()), float(self.distanceToXYSpinbox.get()) )
        
    def updateDisplay(self):
        #recalculate points
        self.calculate()
        
        self.a.clear()
        
        #do the plotting here!
        self.a.scatter(self.uvToolPoints[:,0], self.uvToolPoints[:,1], c = '#00b500', s = 1) #dark green
        self.a.scatter(self.uvPoints[:,0], self.uvPoints[:,1], c = '#00ff00', s = 1) #bright green
        self.a.scatter(self.xyPoints[:,0], self.xyPoints[:,1], c = '#ff0000', s = 1) #bright red, size = 1
        self.a.scatter(self.xyToolPoints[:,0], self.xyToolPoints[:,1], c = '#b50000', s = 1) #dark red
        self.a.legend(["UV tool", "UV on foam", "XY on foam", "XY tool"])  
        
        #draw arrow indicating direction
        ylim = self.a.get_ylim()
        xlim = self.a.get_xlim()
        arrowLength = ((ylim[1]-ylim[0]) + (xlim[1]-xlim[0]))/2
        arrowLength = arrowLength / 15
        self.a.arrow(*getArrowAtPoint(self.xyToolPoints, self.xyToolPoints[self.anchorIndices[0]], arrowLength), head_width = arrowLength / 2.5, color = "#750000")
        self.a.arrow(*getArrowAtPoint(self.uvToolPoints, self.uvToolPoints[self.anchorIndices[0]], arrowLength), head_width = arrowLength / 2.5, color = "#007500")
        
        #draw lines between corresponding anchors/points
        for anchorIndex in self.anchorIndices :
            xCoordinates = [ self.xyToolPoints[anchorIndex][0], self.uvToolPoints[anchorIndex][0] ]
            yCoordinates = [ self.xyToolPoints[anchorIndex][1], self.uvToolPoints[anchorIndex][1] ]
            self.a.plot(xCoordinates, yCoordinates, 'grey')
        
        #visual settings
        self.a.axis("equal")    
        self.a.grid(color='grey', linestyle='-', linewidth=0.5)
        self.canvas.draw()
    
    def getPoints(self):
        return (self.xyPoints, self.uvPoints) 
    
    def getToolPoints(self):
        return (self.xyToolPoints, self.uvToolPoints)
    
        
app = gcodeGeneratorApp()

app.mainloop()
