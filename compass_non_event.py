#!/usr/bin/env python

#Basic imports
from ctypes import *
import sys
import signal
#Phidget specific imports
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs
from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan
from Phidgets.Phidget import PhidgetLogLevel
import math

class Compass():
    
    def __init__(self):
        
        # Build the Phidget Object
        try:
            self.spatial = Spatial()
        except Exception as e:
            print("Could not build object %s" %e)
            exit(0)

        
        
        self.last_angles = [0,0,0]              # The last set of bearing angles
        self.compass_bearing_filter = []        # bearing store for filtering
        self.bearing_filter_size = 10           # Max size of bearing store
        self.compass_bearing = 0                # Init bearing value
        

 
        # Register signal capture to end programme
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def DeviceInfo(self):
        """
        Get and print all the information about this device
        """        
        
        self.is_attached = self.spatial.isAttached()
        self.device_name = self.spatial.getDeviceName()
        self.seriel_number = self.spatial.getSerialNum()
        self.device_version= self.spatial.getDeviceVersion()
        self.accel_axis_count = self.spatial.getAccelerationAxisCount()
        self.gyro_axis_count = self.spatial.getGyroAxisCount()
        self.comp_axis_count = self.spatial.getCompassAxisCount()
        
        print("Start of Device Info")
        print self.is_attached
        print self.device_name
        print self.seriel_number
        print self.device_version
        print self.accel_axis_count
        print self.gyro_axis_count
        print self.comp_axis_count
        print("End of Device Info")
        
            
    def ReadData(self, e):
        """
        Reads and prints data off of each sensor 
        """
        source = e.device
        print("Spatial %i: Amount of data %i" % (source.getSerialNum(), len(e.spatialData)))
        for index, spatialData in enumerate(e.spatialData):
            print("=== Data Set: %i ===" % (index))
            if len(spatialData.Acceleration) > 0:
                print("Acceleration> x: %6f  y: %6f  z: %6f" % (spatialData.Acceleration[0], spatialData.Acceleration[1], spatialData.Acceleration[2]))
            if len(spatialData.AngularRate) > 0:
                print("Angular Rate> x: %6f  y: %6f  z: %6f" % (spatialData.AngularRate[0], spatialData.AngularRate[1], spatialData.AngularRate[2]))
            if len(spatialData.MagneticField) > 0:
                print("Magnetic Field> x: %6f  y: %6f  z: %6f" % (spatialData.MagneticField[0], spatialData.MagneticField[1], spatialData.MagneticField[2]))
        print("Time Span> Seconds Elapsed: %i  microseconds since last packet: %i" % (spatialData.Timestamp.seconds, spatialData.Timestamp.microSeconds))
    
        print("------------------------------------------")
    
    
    
    def CalculateBearing(self):
        
        """
        Calculates a bearing value using the accelerometer and magnetometer
        """
        
        gravity = []
        mag_field = []
        angles = []
        
        # 0 = x axis, 1 = y axis, 2 = z axis
        gravity.append(self.spatial.getAcceleration(0)) 
        gravity.append(self.spatial.getAcceleration(1))
        gravity.append(self.spatial.getAcceleration(2))
        
        mag_field.append(self.spatial.getMagneticField(0))
        mag_field.append(self.spatial.getMagneticField(1))
        mag_field.append(self.spatial.getMagneticField(2))
        
        # Roll angle about axis 0
        # tan(roll angle) = gy/gz
        # Atan2 gives us output as (-180 - 180) deg
        roll_angle = math.atan2(gravity[1], gravity[2])
        
        # Pitch Angle - about axis 1
        # tan(pitch angle) = -gx / ((gy * sin(roll angle)) + (gz * cos(roll angle)))
        # Pitch angle range is (-90 - 90) degrees
        pitch_angle = math.atan( -gravity[0] / (gravity[1] * math.sin(roll_angle) + gravity[2] * math.cos(roll_angle)))
        
        # Yaw Angle - about axis 2
        # tan(yaw angle) = (mz * sin(roll) \96 my * cos(roll)) / 
        #                   (mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll))
        #      Use Atan2 to get our range in (-180 - 180)
           
        #  Yaw angle == 0 degrees when axis 0 is pointing at magnetic north
        yaw_angle = math.atan2( mag_field[2] * math.sin(roll_angle) - mag_field[1] * math.cos(roll_angle),
                mag_field[0] * math.cos(pitch_angle) + mag_field[1] * math.sin(pitch_angle) * math.sin(roll_angle) + mag_field[2] * math.sin(pitch_angle) * math.cos(roll_angle))                                         
        
        # Add angles to our list
        angles.append(roll_angle)
        angles.append(pitch_angle)
        angles.append(yaw_angle)
        
        # This is our low pass filter to make the values look nicer on screen
        try:
            # Make sure the filter bugger doesn't have values passing the -180<->180 mark
            # This only applies to the Roll and Yaw - Pitch will never have a sudden switch like that
            for i in xrange(0,3,2):
                
                if( math.fabs( angles[i] - self.last_angles[i] > 3 )):
                    
                    for stuff in self.compass_bearing_filter:
                        
                        if(angles[i] > self.last_angles[i]):
                            stuff[i] += 360 * math.pi / 180.0
                        else:
                            stuff[i] -= 360 * math.pi / 180.0
                            
            self.last_angles = angles
            
            self.compass_bearing_filter.append(angles)
            
            if(len(self.compass_bearing_filter) > self.bearing_filter_size):
                self.compass_bearing_filter.pop(0)
                
            yaw_angle = pitch_angle = roll_angle = 0
            
            for stuff in self.compass_bearing_filter:
                roll_angle += stuff[0]
                pitch_angle += stuff[1]
                yaw_angle += stuff[2]
                
            yaw_angle = yaw_angle / len(self.compass_bearing_filter)
            pitch_angle = pitch_angle / len(self.compass_bearing_filter)
            roll_angle = roll_angle / len(self.compass_bearing_filter)
            
            # Convert radians to degrees for display
            self.compass_bearing = yaw_angle * (180.0 / math.pi)
            
            # Set a directional string (one of 8 compass directions
            # based on the angle i.e. North, North-East, South-West etc.
            # Split by checking main 4 compass point angles +/- 22.5 deg
            
            if(self.compass_bearing >=0 and self.compass_bearing < 22.5 ):
                string_bearing = "North: "
            elif(self.compass_bearing >=22.5 and self.compass_bearing < 67.5 ):
                string_bearing = "North East: "
            elif(self.compass_bearing >=67.5 and self.compass_bearing < 112.5 ):
                string_bearing = "East:  "
            elif(self.compass_bearing >=112.5 and self.compass_bearing < 157.5 ):
                string_bearing = "South East: "
            elif(self.compass_bearing >=157.5 and self.compass_bearing <= 180 ):
                string_bearing = "South: "
            elif(self.compass_bearing <= -157.5 and self.compass_bearing > -180 ):
                string_bearing = "South: "
            elif(self.compass_bearing <= -112.5 and self.compass_bearing > -157.5 ):
                string_bearing = "South West: "
            elif(self.compass_bearing <= -67.5 and self.compass_bearing > -112.5 ):
                string_bearing = "West: "
            elif(self.compass_bearing <= -22.5 and self.compass_bearing > -67.5 ):
                string_bearing = "North West: "
            elif(self.compass_bearing <= 0.0 and self.compass_bearing > -22.5 ):
                string_bearing = "North: "
            
            # Add the actual bearing value to the string then print
            string_bearing = string_bearing + " %.1f" % self.compass_bearing
            print string_bearing
                            
        except Exception as e:
            print e
                        
        
    
  
        
    def Main(self):
                
        try:
            self.spatial.openPhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(0)
        
        print ("Looking for device...")
        
        try:
            self.spatial.waitForAttach(10000)
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            
        self.spatial.setDataRate(200)
        
        print("Press Enter to start....")
        
        chr = sys.stdin.read(1)
        
        while True:
            self.CalculateBearing()
            pass
        
        
        
        
    def signal_handler(self, signal, frame):
        """
        Nice clean way to catch user terminating program
        
        Closes Phidget Object - IMPORTANT, this object needs to be closed
        otherwise it will assume it is still being used and therefore will not
        respond to new instances of this program
        
        """
        print("Ctrl+C pressed")
        
        try:
            self.spatial.closePhidget()
            print("Compass Closed")
            print("Exiting....")
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(0)
            
        exit(1)
        


if __name__ == "__main__":
    
    """
    This program needs to be run with sudo unless you have usb dev rules added
    """
    
    compass = Compass()
    
    compass.Main()
    
 
        
        
        
        
        
            