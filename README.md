# RotatorPositioner

Firmware for a device that gets the antenna orientation from a BNO055 sensor and distributes it to local LAN via MQTT

## Emitted Messages

### MQTT

MQTT payloads are composed of JSON objects. There are three topics (to date):

- antenna/orientation
  This topic carries the antenna orientation data (example):

  ```json
  {
    "dateTime" : "Thursday, 19-Feb-2026 09:32:40 UTC",
    "ts" : 1771493560427,
    "azimuth" : 47.33,
    "elevation" : 19.13,
    "revolutions": -1,
    "units" : {
      "ts" : "milliseconds",
      "orientation" : "decimal degrees"
    }
  }
  ```

- antenna/status
  Transmits system information (will be expanded on):

  ```json
  {
    "dateTime" : "Thursday, 19-Feb-2026 09:35:11 UTC",
    "ts" : "1771493711925",
    "bssid" : "B8:69:F4:81:8D:E6",
    "temperatureC": 27
  }
  ```

- antenna/calibration
  Calibration data for the evaluation of the quality of orientation data:

  ```json
  {
    "dateTime" : "Thursday, 19-Feb-2026 09:37:37 UTC",
    "ts" : "1771493857399",
    "system" : 0,
    "gyro" : 3,
    "accelerometer" : 1,
    "magnetometer" : 0
  }
  ```

  Calibration is better when values are higher (0..3).


  ### Serial Output (NMEA)

  Serial output data is formatted as an NMEA-sentence with a checksum value calculated over the data of the sentence (as per NMEA-specification).

  ```nmea
  $PANT,TRK,1771493857399,ms,47.33,dd,19.13,dd,-1,revs,29,c,0,3,1,0*<HH>\r\n   
  ```
  - "$" : Start of Sentence identifier
  - "PANT,TRK": The Talker and Payload identifier
    - "P": proprietory message, 
    - "ANT": antenna, 
    - "TRK": tracker
  - "1771493857399": Timestamp
  - "ms": Timestamp is in milliseconds
  - "47.33": Azimuth angle (0..360)
  - "dd": Decimal degrees
  - "19.13": Elevation angle (0..90)
  - "dd": Decimal degrees
  - "-1": Revolutions (if < 0 or > 360)
  - "revs": Revolutions
  - "29": Temperature
  - "c": Celsius
  - "0,3,1,0": Calibration quality indicators
  - "*": End of Sentence identifier
  - <HH>: Checksum
  - ",": Field separator

Checksum field is the 8-bit exclusive OR (no start or stop characters) of all characters in the sentence. Checksum consists of 2 characters and is represented as a hex number.  

