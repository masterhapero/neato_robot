"""
neato_driver.py is a generic driver for the Neato Botvac Vacuums.
ROS2 Bindings can be found in the neato_node package.
"""

#__author__ = "jnugen@gmail.com (James Nugen)"

import rclpy

import serial
import time
import threading

BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second

class Botvac():
    def __init__(self, port = "/dev/ttyUSB0"):
        self.logger = rclpy.logging.get_logger("neato_driver")

        try:
            self.port = serial.Serial(port, 115200, timeout = 0)
        except:
            self.logger.error("Could not access serial port: %s" % port)
            raise

        #if not self.port.isOpen():
        #    self.logger.error("Failed To Open Serial Port %s" % port)
        #    return

        self.logger.info("Opened Serial Port: %s" % port)

        # Storage for motor and sensor information
        self.state = {
            "LeftWheel_PositionInMM": 0.0,
            "RightWheel_PositionInMM": 0.0,
            "LSIDEBIT": 0,
            "RSIDEBIT": 0,
            "LFRONTBIT": 0,
            "RFRONTBIT": 0
        }

        self.stop_state = True
        # turn things on

        self.comsData = []
        self.responseData= []
        self.currentResponse=[]

        self.reading = False

        self.readLock = threading.RLock()
        self.readThread = threading.Thread(None, self.read)
        self.readThread.start()

        self.port.flushInput()
        self.sendCmd("\n\n\n")
        self.port.flushInput()

        self.setTestMode("on")
        self.setLDS("on")

        time.sleep(0.5)
        #self.setLed("ledgreen") #doesn't exist on Botvac Connected

        self.base_width = BASE_WIDTH
        self.max_speed = MAX_SPEED

        self.flush()

        self.logger.info("Init Done")


    def shutdown(self):
        self.setLDS("off")
        self.setLed("buttonoff")

        time.sleep(1)

        self.setTestMode("off")
        self.port.flush()

        self.reading=False
        self.readThread.join()

        self.port.close()


    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.sendCmd("testmode " + value)

    def setLDS(self, value):
        self.sendCmd("setldsrotation " + value )

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.sendCmd("getldsscan")

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        intensities = list()

        angle = 0

        if not self.readTo("AngleInDegrees"):
            self.flush()
            return []

        last = False
        while not last: #angle < 360:
            try:
                vals, last = self.getResponse()
            except Exception as ex:
                self.logger.error("Exception Reading Neato lidar: " + str(ex))
                last = True
                vals = []

            vals = vals.split(",")

            if ((not last) and ord(vals[0][0]) >= 48 and ord(vals[0][0]) <= 57 ):
                #print(angle, vals)
                try:
                    a = int(vals[0])
                    r = int(vals[1])
                    i = int(vals[2])
                    e = int(vals[3])

                    while (angle < a):
                        ranges.append(0.0)
                        intensities.append(0.0)
                        angle += 1

                    if (e == 0):
                        ranges.append(r / 1000.0)
                        intensities.append(float(i))
                    else:
                        ranges.append(0.0)
                        intensities.append(0.0)
                except:
                    ranges.append(0.0)
                    intensities.append(0.0)
                    
                angle += 1

        if len(ranges) != 360:
            self.logger.info( "Missing laser scans: got %d points" %len(ranges))

        return ranges, intensities

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then,
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False

        self.sendCmd("setmotor" + 
                     " lwheeldist " + str(int(l)) + 
                     " rwheeldist " + str(int(r)) + 
                     " speed " + str(int(s)))

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """

        self.sendCmd("getmotors")

        if not self.readTo("Parameter"):
            self.flush()
            return [0,0]

        last = False
        while not last:
            try:
                vals, last = self.getResponse()
                #print(vals, last)
                values = vals.split(",")
                self.state[values[0]] = float(values[1])
            except Exception as ex:
                self.logger.error("Exception Reading Neato motors: " + str(ex))

        return [self.state["LeftWheel_PositionInMM"], self.state["RightWheel_PositionInMM"]]

    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """

        self.sendCmd("getanalogsensors")

        if not self.readTo("SensorName"):
            self.flush()
            return

        last = False
        while not last:
            try:
                vals,last = self.getResponse()
                values = vals.split(",")
                self.state[values[0]] = int(values[1])
            except Exception as ex:
                self.logger.error("Exception Reading Neato Analog sensors: " + str(ex))

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """

        self.sendCmd("getdigitalsensors")

        if not self.readTo("Digital Sensor Name"):
             self.flush()
             return [0, 0, 0, 0]

        last = False
        while not last:
            try:
                vals, last = self.getResponse()
                values = vals.split(",")
                self.state[values[0]] = int(values[1])
                #print("Got Sensor: %s=%s" % (values[0], values[1]))
            except Exception as ex:
                self.logger.error("Exception Reading Neato Digital sensors: " + str(ex))
        return [self.state["LSIDEBIT"], self.state["RSIDEBIT"], self.state["LFRONTBIT"], self.state["RFRONTBIT"]]

    def getButtons(self):
        return [0, 0, 0, 0, 0]

    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """

        self.sendCmd("getcharger")

        if not self.readTo("Label"):
            self.flush()
            return

        last = False
        while not last:

            vals,last = self.getResponse()
            values = vals.split(",")
            try:
                self.state[values[0]] = int(values[1])

            except Exception as ex:
                self.logger.error("Exception Reading Neato charger info: " + str(ex))

    def setBacklight(self, value):
        if value > 0:
           self.sendCmd("setled backlighton")
        else:
           self.sendCmd("setled backlightoff")

    def setLed(self, cmd):
        self.sendCmd("setled %s" % cmd)
    
    def setLED(self, cmd):
        self.setLed(cmd)

    def sendCmd(self, cmd):
        #self.logger.info("Sent command: %s" % cmd)
        self.port.write(bytes("%s\n" % cmd, encoding = 'utf8'))

    def readTo(self, tag, timeout = 1):
        try:
            line,_ = self.getResponse(timeout)
        except:
            return False

        if line == "":
            return False

        while line.split(",")[0] != tag:
            try:
                line,_ = self.getResponse(timeout)
                if line == "":
                    return False
            except:
                return False

        return True

    # thread to read data from the serial port
    # when an end of response (^Z) is read, adds the complete list of response lines to self.responseData and resets internal buffer
    def read(self):
        self.reading = True
        datalst = list()
        datain = bytearray()

        while (self.reading and rclpy.ok()):
            try:
                valarr = self.port.read(size=8196)
            except Exception as ex:
                self.logger.error("Exception Reading Neato Serial: " + str(ex))
                valarr = b''

            if len(valarr) > 0:
                datain += valarr
                if datain[len(datain)-1] == 26:
                    # self.logger.info("Read "+str(len(valarr))+" "+datain)
                    datastr = datain[0:len(datain)-2].decode()
                    datalst = datastr.split('\n')
                    with self.readLock:
                        self.responseData.append(datalst)
                    datain = bytearray()
                        
    # thread to read data from the serial port
    # buffers each line in a list (self.comsData)
    # when an end of response (^Z) is read, adds the complete list of response lines to self.responseData and resets the comsData list for the next command response.
    def read_old(self):
        self.reading = True
        line = bytearray()

        while (self.reading and rclpy.ok()):
            try:
               val = self.port.read(1) # read from serial 1 char at a time so we can parse each character
            except Exception as ex:
                self.logger.error("Exception Reading Neato Serial: " + str(ex))
                val = b''

            #print(val)
            if len(val) > 0:
                if val == b'\r': # ignore the CRs
                    pass

                elif val == b'\x1a': # ^Z (end of response)
                    if len(line) > 0:
                        self.comsData.append(line.decode()) # add last line to response set if it is not empty
                        #print("Got Last Line: %s" % line)
                        line = b'' # clear the line buffer for the next line

                    #print("Got Last")
                    with self.readLock: # got the end of the command response so add the full set of response data as a new item in self.responseData
                        self.responseData.append(list(self.comsData))

                    self.comsData = [] # clear the bucket for the lines of the next command response

                elif val == b'\n': # NL, terminate the current line and add it to the response data list (comsData) (if it is not a blank line)
                    if len(line) > 0:
                        self.comsData.append(line.decode())
                        #print("Got Last Line: %s" % line)
                        line = b'' # clear the bufer for the next line

                else:
                    line += val # add the character to the current line buffer

    # read response data for a command
    # returns tuple (line,last)
    # line is one complete line of text from the command response
    # last = true if the line was the last line of the response data (indicated by a ^Z from the neato)
    # returns the next line of data from the buffer.
    # if the line was the last line last = true
    # if no data is avaialable and we timeout returns line=""
    def getResponse(self, timeout = 1):

        # if we don't have any data in currentResponse, wait for more data to come in (or timeout) 
        while (len(self.currentResponse) == 0) and rclpy.ok() and timeout > 0:

            with self.readLock: # pop a new response data list out of self.responseData (should contain all data lines returned for the last sent command)
               if len(self.responseData) > 0:
                  self.currentResponse = self.responseData.pop(0)
                  #print("New Response Set")
               else:
                  self.currentResponse = [] # no data to get

            if len(self.currentResponse) == 0: # nothing in the buffer so wait (or until timeout)
               time.sleep(0.010)
               timeout = timeout - 0.010

        # default to nothing to return
        line = ""
        last = False

        # if currentResponse has data pop the next line 
        if len(self.currentResponse) != 0:
            line = self.currentResponse.pop(0)
            #print(line, len(self.currentResponse))
            if len(self.currentResponse) == 0:
                last = True  # if this was the last line in the response set the last flag
        else:
            self.logger.error("Time Out") # no data so must have timedout

        #self.logger.debug("Got Response: %s, Last: %d" % (line,last))
        return (line,last)

    def flush(self):
        while(1):
            l,_ = self.getResponse(1)
            if l == "":
                return

