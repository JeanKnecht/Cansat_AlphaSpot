import serial
ser = serial.Serial('com8', 115200) #verander "com3" naar de juiste naam van seriele poort.
path = ""

data = []

def getSerialText():
    ser_bytes = ser.readline().decode('utf-8').strip('\r\n')     # ser_bytes = rauwe tekst van 1 lijntje op de seriële monitor
    meting = ser_bytes.split(",")                                  # vb: als ser_bytes = "5890 4.98" dan is meting = ['5890', '4.98']
    return meting

try:
    while True:
        data = [] 
        gpsData = getSerialText()
        data.append(str((float(gpsData[1]))/10000000.000))
        data.append(str((float(gpsData[0]))/10000000.000))
        data.append(str((float(gpsData[2]))/100.000))
        result = ','.join(data)
        path += result + "\n"
        print(result)

        with open("position.kml", "w") as pos:
             pos.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Paths</name>
    <description>Examples of paths. Note that the tessellate tag is by default
      set to 0. If you want to create tessellated lines, they must be authored
      (or edited) directly in KML.</description>
    <Style id="yellowLineGreenPoly">
      <LineStyle>
        <color>7f00ffff</color>
        <width>4</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>Absolute Extruded</name>
      <description>Transparent green wall with yellow outlines</description>
      <styleUrl>#yellowLineGreenPoly</styleUrl>
      <LineString>
        <coordinates>%s</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>""" % (path))
except KeyboardInterrupt:
    print('interrupted!')