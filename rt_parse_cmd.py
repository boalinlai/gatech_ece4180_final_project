# ParseWeb version 1.0
# Author: Boa-Lin Lai
import math
import matplotlib.pyplot as plt
import time
import sys
from lxml import html
import urllib2
import select
serverUrl = "http://0.0.0.0"  # change the address for esp8266
"""
  This file will parse the data from ESP8266 and plot the points in real time
"""


def parseWeb():
    """
     This module is used to get the data points from ECE2031 server
    """
    response = urllib2.urlopen(serverUrl)
    html = response.read()
    xPoints = []  # list for robot x points
    yPoints = []  # list for robot y points
    xDesPoints = []  # list for destination x points
    yDesPoints = []  # list for destination y ponins
    # this line should be changed for starting trigger.
    strparse = "<p>"
    html = html.split(strparse)[-1]
    strparse = "</p>"
    html = html.split(strparse)[0]
    for line in html.split('<br>'):
        if line is not "":
            points = line.split(',')
            print points
            x = float(points[0])
            y = float(points[1])
            xPoints.append(x)
            yPoints.append(y)
    print "Robot Path"
    for i in range(1, len(xPoints)):
        print '(' + str(xPoints[i])[:5] + ',' + str(yPoints[i])[:5] + ')'
    return xPoints, yPoints


def printTerminal():
    plt.subplot(121)
    plt.plot([-16, 16], [-16, -16], 'k-')
    plt.plot([-16, 16], [0, 0], 'k-')
    plt.plot([-16, 16], [16, 16], 'k-')
    plt.plot([0, 0], [-16, 16], 'k-')
    plt.xlabel("X axis -- inch")
    plt.ylabel("Y axis -- inch")
    plt.title("Terminal Outline")
    axes = plt.gca()
    axes.set_xlim([-500, 500])
    axes.set_ylim([-500, 500])
    plt.show()


def printResult(dataX, dataY):
    """
      This module will initialize the point on the canvas
    """
    plt.clf()
    plt.plot([-16, 16], [-16, -16], 'k-')
    plt.plot([-16, 16], [0, 0], 'k-')
    plt.plot([-16, 16], [16, 16], 'k-')
    plt.plot([0, 0], [-16, 16], 'k-')
    plt.plot(dataX, dataY, 'ko', label="Robot Log")
    plt.plot(dataX, dataY, 'r--', label="Robot Path")
    plt.plot(0, 0, 'gs', label="Starting Point")
    plt.plot(dataX[-1], dataY[-1], 'bo', label="Current Location")
    plt.legend(loc='best')


if __name__ == "__main__":
    """
      main funciton for parseWeb
    """
    plt.ion()
    plt.grid()
    fName = "points.txt"
    ans = ""
    gl_dataX = [8]
    gl_dataY = [8]
    cmdlist = {}
    cmdlist['B'] = "b1b1d1b1b1"  # cmdlist, it can be extended
    cmdlist['b'] = ""
    while(ans != "q"):
        dataX, dataY = parseWeb()
        if (dataX and
                not gl_dataX[-1] == dataX[-1] and
                not gl_dataY[-1] == dataY[-1]):
            gl_dataX = [8]
            gl_dataY = [8]
            gl_dataX.extend(dataX)
            gl_dataY.extend(dataY)
        printResult(gl_dataX, gl_dataY)
        plt.xlabel("X axis -- Feet")
        plt.ylabel("Y axis -- Feet")
        plt.title("Simulation Result")
        axes = plt.gca()
        axes.set_xlim([-100, 100])
        axes.set_ylim([-100, 100])
        print "You have 2 seconds to answer!"
        i, o, e = select.select([sys.stdin], [], [], 2)
        # 2 second respond time
        if (i):
            ans = sys.stdin.readline().strip()
            print "You said " + ans
        else:
            print "You said nothing!"
        if ans == "s":
            plt.savefig("result.ps")
        if ans is not "":
            print "send  " + cmdlist[ans]
            urllib2.urlopen(serverUrl, "CMDSTART" + cmdlist[ans] + "CMDEND")
            # send the post data to wifi module
        ans = ""
        plt.pause(0.5)
