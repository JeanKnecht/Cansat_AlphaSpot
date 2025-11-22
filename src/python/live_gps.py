"""
CanSat AlphaSpot - Live GPS Tracker

High School Project: Real-time GPS tracking and visualization system.

This script reads GPS coordinates from the serial port (Arduino receiver)
and creates a KML file that can be opened in Google Earth to visualize
the CanSat's real-time location and flight path.

Features:
    - Real-time GPS coordinate reception via serial port
    - Automatic KML file generation for Google Earth
    - 3D flight path visualization
    - Live coordinate display

Usage:
    python live_gps.py

Requirements:
    - Serial connection to Arduino receiver
    - pyserial library (pip install pyserial)

Author: Jean Knecht (High School Project)
Year: [Your graduation year]
"""

import serial
from pathlib import Path
from typing import List, Optional


class CanSatGPSTracker:
    """Tracks CanSat GPS coordinates and generates KML file for Google Earth."""
    
    def __init__(self, port: str = 'COM8', baud_rate: int = 115200):
        """
        Initialize GPS tracker.
        
        Args:
            port: Serial port name (e.g., 'COM8' on Windows, '/dev/ttyUSB0' on Linux)
            baud_rate: Serial communication baud rate
        """
        self.port = port
        self.baud_rate = baud_rate
        self.serial_connection: Optional[serial.Serial] = None
        self.path_data: str = ""
        self.kml_file = Path("position.kml")
        
    def connect(self) -> bool:
        """
        Connect to serial port.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False
    
    def read_gps_data(self) -> Optional[List[str]]:
        """
        Read GPS coordinates from serial port.
        
        Returns:
            List with [longitude, latitude, altitude] or None if error
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            return None
            
        try:
            # Read line from serial port
            raw_data = self.serial_connection.readline().decode('utf-8').strip('\r\n')
            gps_data = raw_data.split(",")
            
            if len(gps_data) >= 3:
                # Convert from integer format to decimal degrees
                longitude = str((float(gps_data[1])) / 10000000.0)
                latitude = str((float(gps_data[0])) / 10000000.0)
                altitude = str((float(gps_data[2])) / 100.0)
                
                return [longitude, latitude, altitude]
        except (ValueError, IndexError, UnicodeDecodeError) as e:
            print(f"Error parsing GPS data: {e}")
            return None
        
        return None
    
    def update_kml_file(self) -> None:
        """
        Update KML file with current path data.
        """
        kml_template = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>CanSat Flight Path</name>
    <description>Real-time GPS tracking of CanSat AlphaSpot flight path</description>
    <Style id="yellowLineGreenPoly">
      <LineStyle>
        <color>7f00ffff</color>
        <width>4</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>Absolute Extruded</name>
      <description>CanSat flight path in 3D</description>
      <styleUrl>#yellowLineGreenPoly</styleUrl>
      <LineString>
        <coordinates>{}</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>"""
        
        try:
            with open(self.kml_file, "w", encoding='utf-8') as kml_file:
                kml_file.write(kml_template.format(self.path_data.strip()))
            print(f"KML file updated: {self.kml_file}")
        except IOError as e:
            print(f"Error writing KML file: {e}")
    
    def run(self) -> None:
        """
        Main tracking loop - reads GPS data and updates KML file.
        """
        if not self.connect():
            return
        
        print("Starting GPS tracking... Press Ctrl+C to stop")
        
        try:
            while True:
                gps_coords = self.read_gps_data()
                
                if gps_coords:
                    # Format: longitude,latitude,altitude
                    coord_string = ','.join(gps_coords)
                    self.path_data += coord_string + "\n"
                    
                    print(f"GPS: Lat={gps_coords[1]}, Lon={gps_coords[0]}, Alt={gps_coords[2]}m")
                    
                    # Update KML file
                    self.update_kml_file()
        except KeyboardInterrupt:
            print('\nGPS tracking stopped!')
        finally:
            if self.serial_connection:
                self.serial_connection.close()
                print("Serial connection closed")


def main():
    """
    Main entry point for GPS tracker.
    
    Note: Update the COM port to match your system:
    - Windows: 'COM8', 'COM3', etc.
    - Linux/Mac: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    """
    # Configure serial port - UPDATE THIS FOR YOUR SYSTEM
    tracker = CanSatGPSTracker(port='COM8', baud_rate=115200)
    tracker.run()


if __name__ == "__main__":
    main()

