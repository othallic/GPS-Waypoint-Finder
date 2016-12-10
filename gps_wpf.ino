/*****************************************************************************/
/*   This program is free software: you can redistribute it and/or modify    */
/*   it under the terms of the GNU General Public License as published by    */
/*   the Free Software Foundation, either version 3 of the License, or       */
/*   (at your option) any later version.                                     */
/*                                                                           */
/*    This program is distributed in the hope that it will be useful,        */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of         */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          */
/*    GNU General Public License for more details.                           */
/*                                                                           */
/*    You should have received a copy of the GNU General Public License      */
/*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */
/*****************************************************************************/

// Libraries
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
// Pin definitions
#define SSRX_PIN 8
#define SSTX_PIN 7
#define NEO_PIN 6
#define NEXT_PIN 4
#define PREV_PIN 5
#define LED 13
// Constants
#define NUMOFWAYPOINTS 5
#define PIXELCOUNT 16
#define WAYPOINTRADIUS 5
#define DISTANCEINDICATORADDRESS 0x70
#define MPERDEGLONGITUDE 75830
#define MPERDEGLATITUDE 111190
// Global variables
float currentWaypointLongitude = 0;
float currentWaypointLatitude = 0;
int currentWaypointAngle = 0;
byte currentWaypoint = 0;
int correctionAngle = 0;
int distance = 0;
boolean prev_button_pressed, next_button_pressed, dispwp = false;
byte inc1, inc2, printcount = 0;
float Waypoint[NUMOFWAYPOINTS][2] = {{0,0},{0,0},{0,0},{0,0},{0,0}};
// NeoPixel interface on pin 6
Adafruit_NeoPixel directionIndicator = Adafruit_NeoPixel(PIXELCOUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
// 4 x 7-Seg display interface
Adafruit_7segment distanceIndicator = Adafruit_7segment();
// Software Serial on pins 8 and 7 to comunicate wih GPS module
SoftwareSerial gpsSerial(8, 7);
// Create instance of GPS with comunication link to mySerial
Adafruit_GPS GPS(&gpsSerial);
// ISR for 1 ms Timer0 Interrupt
SIGNAL(TIMER0_COMPA_vect)
   {
   GPS.read();
   }
// This function displays debug and gps data on the serial port
void printGPSinfo(void)
   {
   Serial.print("\nTime: ");
   Serial.print(GPS.hour, DEC);
   Serial.print(':');
   Serial.print(GPS.minute, DEC);
   Serial.print(':');
   Serial.print(GPS.seconds, DEC);
   Serial.print('.');
   Serial.println(GPS.milliseconds);
   Serial.print("Date: ");
   Serial.print(GPS.day, DEC);
   Serial.print('/');
   Serial.print(GPS.month, DEC);
   Serial.print("/20");
   Serial.println(GPS.year, DEC);
   Serial.print("Fix: ");
   Serial.print((int)GPS.fix);
   Serial.print(" quality: ");
   Serial.println((int)GPS.fixquality);
   if (GPS.fix)
      {
      Serial.print("Location (in degrees): ");
      Serial.print(GPS.latitudeDegrees, 10);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 10);
      Serial.print("Speed (m/s): ");
      Serial.println(GPS.speed*0.514);
      Serial.print("Angle: ");
      Serial.println(GPS.angle);
      Serial.print("HDOP: ");
      Serial.println(GPS.HDOP);
      Serial.print("Distance to next Waypoint (m): ");
      Serial.println(distance, DEC);
      Serial.print("Current Waypoints heading (in degrees): ");
      Serial.println(currentWaypointAngle);
      Serial.print("Correction heading to next Waypoint (in degrees): ");
      Serial.println(correctionAngle);
      Serial.print("Current Waypoint Latitude: ");
      Serial.println(currentWaypointLatitude, 7);
      Serial.print("Current Waypoint Longitude: ");
      Serial.println(currentWaypointLongitude, 7);
      Serial.print("Current Waypoint: ");
      Serial.println(currentWaypoint);
      }
   }
// This function calculates the distance between current position and the current waypoint with pythagoras
int calculateDistance(float longitude1, float latitude1, float longitude2, float latitude2)
   {
   return (int) sqrt(pow((longitude2 - longitude1)*MPERDEGLONGITUDE, 2) + pow((latitude2 - latitude1)*MPERDEGLATITUDE, 2));
   }
// This function determines if the current waypoint is south, north, east or west of our position and then calculates the angle
// to it from geographic north
int calculateAngle(float longitude1, float latitude1, float longitude2, float latitude2)
   {
   float arcos = 0;
   int dist, deg = 0;
   dist = calculateDistance(longitude1, latitude1, longitude2, latitude2);
   arcos = acos((float)((longitude2 - longitude1)*MPERDEGLONGITUDE / dist));
   if ((latitude2 - latitude1) > 0)
      {
      // Current position below waypoint
      if ((longitude2 - longitude1) > 0)
         {
         deg = (int)(90 - arcos*57.3);
         // Current position left from waypoint
         }
      else
         {
         deg = (int)((180 - arcos*57.3) + 270);
         // Current position right from waypoint
         }
      }
   else
      {
      // Current position above waypoint
      if ((longitude2 - longitude1) > 0)
         {
         deg = (int)(arcos*57.3 + 90);
         // Current position left from waypoint
         }
      else
         {
         deg = (int)(arcos*57.3 + 90);
         // Current position right from waypoint
         }
      }
   return deg;
   }
// This function properly displays the direction to the next waypoint on the direction indicator
void setDirection(int angle)
   {
   int dir = 0;
   int i = 0;
   for (int i = 0;i < PIXELCOUNT;i++)
      {
      directionIndicator.setPixelColor(i, directionIndicator.Color(0,0,100));
      }
   if (angle < 0)
      {
      angle = 360 + angle;
      }
   if (angle > 22)
      {
      if (angle > 45)
         {
         if (angle > 67)
            {
            if (angle > 90)
               {
               if (angle > 112)
                  {
                  if (angle > 135)
                     {
                     if (angle > 157)
                        {
                        if (angle > 180)
                           {
                           if (angle > 202)
                              {
                              if (angle > 225)
                                 {
                                 if (angle > 247)
                                    {
                                    if (angle > 270)
                                       {
                                       if (angle > 292)
                                          {
                                          if (angle > 315)
                                             {
                                             if (angle > 337)
                                                {
                                                dir = 9;
                                                }
                                             else
                                                {
                                                dir = 8;
                                                }
                                             }
                                          else
                                             {
                                             dir = 7;
                                             }
                                          }
                                       else
                                          {
                                          dir = 6;
                                          }
                                       }
                                    else
                                       {
                                       dir = 5;
                                       }
                                    }
                                 else
                                    {
                                    dir = 4;
                                    }
                                 }
                              else
                                 {
                                 dir = 3;
                                 }
                              }
                           else
                              {
                              dir = 2;
                              }
                           }
                        else
                           {
                           dir = 1;
                           }
                        }
                     else
                        {
                        dir = 0;
                        }
                     }
                  else
                     {
                     dir = 15;
                     }
                  }
               else
                  {
                  dir = 14;
                  }
               }
            else
               {
               dir = 13;
               }
            }
         else
            {
            dir = 12;
            }
         }
      else
         {
         dir = 11;
         }
      }
   else
      {
      dir = 10;
      }
   directionIndicator.setPixelColor(dir, directionIndicator.Color(0,100,0));
   directionIndicator.show();
   }
// This function is called every ms and checks if there is a programming command comeing
void lookforWaypoint(void)
   {
   float lat, lon;
   // Only read if serial data is available in the buffer
   if (Serial.available() > 0)
      {
      // Read waypoint number
      currentWaypoint = (byte) (Serial.read() - 48);
      // Read separator sign
      Serial.read();
      // Read and parse latitude and lingitude in float
      lat = Serial.parseFloat();
      lon = Serial.parseFloat();
      // Acknowledge with the read parameters latitude and longitude to verify the received data
      Serial.print("WP");
      Serial.println(currentWaypoint);
      Serial.print("Lat: ");
      Serial.println(lat, 7);
      Serial.print("Lon: ");
      Serial.println(lon, 7);
      // Flush the buffer to be ready for the next command
      while (Serial.available() > 0)
         {
         Serial.read();
         }
      // Programmed waypoint is current waypoint
      Waypoint[currentWaypoint][0] = lat;
      Waypoint[currentWaypoint][1] = lon;
      // Write Waypoint data to EEPROM
      EEPROM.put(8*currentWaypoint, lat);
      EEPROM.put(4 + 8*currentWaypoint, lon);
      }
   }
// This function shows an animation on the direction indicator if the current waypoint is reached
void waypointReached(void)
   {
   int i, k = 0;
   for (i = 0;i < 26;i++)
      {
      for (k = 0;k < PIXELCOUNT;k++)
         {
         directionIndicator.setPixelColor(k, directionIndicator.Color(0,i*10,255));
         directionIndicator.show();
         delay(1);
         }
      }
   // Clear all pixels
   for (k = 0;k < PIXELCOUNT;k++)
      {
      directionIndicator.setPixelColor(k, directionIndicator.Color(0,0,0));
      directionIndicator.show();
      delay(1);
      }
   }
// This function reads the Waypoints from the EEPROM and stores them in the Waypoint array
void getEEPROMData(void)
   {
   int i = 0;
   for (i = 0;i < NUMOFWAYPOINTS;i++)
      {
      EEPROM.get(8*i, Waypoint[i][0]);
      EEPROM.get(4 + 8*i, Waypoint[i][1]);
      }
   }
// Init function called once the processor has reset
void setup()
   {
   // Setup Serial comunication with host
   Serial.begin(115200);
   //Setup comunication with GPS module
   GPS.begin(9600);
   // Set GPS to output RMC only
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   // Set output rate to 1 Hz
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
   //Setup Timer0 interrupt to read the GPS serial comunication once a millisecond
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
   // Direction indicator setup
   directionIndicator.begin();
   // Distance indicator setup
   distanceIndicator.begin(DISTANCEINDICATORADDRESS);
   distanceIndicator.print(0000, DEC);
   distanceIndicator.writeDisplay();
   distanceIndicator.setBrightness(5);
   distanceIndicator.writeDisplay();
   // Read Waypoint data from EEPROM
   getEEPROMData();
   // Setup LED pin
   pinMode(LED, OUTPUT);
   // Setup NEXT PIN
   pinMode(NEXT_PIN, INPUT_PULLUP);
   // Setup PREV PIN
   pinMode(PREV_PIN, INPUT_PULLUP);
   Serial.println("Init done");
   // Wait until GPS has a valid fix
   while (GPS.fix == false)
      {
      // Animation
      delay(500);
      digitalWrite(LED, 1);
      delay(500);
      digitalWrite(LED, 0);
      // Check serial communication
      lookforWaypoint();
      // We need to parse the NMEA sentence to be able to check the Fix status at the beginning of the while loop
      if (GPS.newNMEAreceived())
         {
         if (GPS.parse(GPS.lastNMEA()))
            {
            }
         }
      }
   Serial.println("Fixed position");
   }
// Endless loop
void loop()
   {
   // If a new NMEA sentence is received
   if (GPS.newNMEAreceived())
      {
      // Parse new NMEA sentence
      if (GPS.parse(GPS.lastNMEA()))
         {
         if(GPS.fix == true)
            {
            // Calculate distance between current waypoint and local position
            distance = calculateDistance(GPS.longitudeDegrees, GPS.latitudeDegrees, currentWaypointLongitude, currentWaypointLatitude);
            // Calculate heading and heading to current waypoint
            currentWaypointAngle = calculateAngle(GPS.longitudeDegrees, GPS.latitudeDegrees, currentWaypointLongitude, currentWaypointLatitude);
            // Calculate correction angle
            correctionAngle = currentWaypointAngle - GPS.angle;
            if (distance < WAYPOINTRADIUS)
               {
               // Display waypoint reached animation
               waypointReached();
               }
            else
               {
               // Display correction heading on NeoPixel ring indicator
               setDirection(correctionAngle);
               }
            // Print debug data every 3 seconds
            if (printcount > 1)
               {
               printcount = 0;
               // Print GPS Data to host
               printGPSinfo();
               }
            else
               {
               printcount++;
               }
            if (dispwp)
               {
               // Display current waypoint number
               distanceIndicator.print(currentWaypoint, DEC);
               distanceIndicator.writeDisplay();
               dispwp = false;
               }
            else
               {
               // Display distance in m on 4 x 7-Seg display
               distanceIndicator.print(distance, DEC);
               distanceIndicator.writeDisplay();
               }
            }
         }
      }
   // Read Previous Waypoint Button with debouncing   
   if (digitalRead(PREV_PIN) == LOW)
      {
      inc1++;
      if (inc1 > 105)
         {
         inc1 = 102;
         }
      if (inc1 == 100)
         {
         if (currentWaypoint > 0)
            {
            currentWaypoint--;
            }
         // Prev button has been pressed for more than 0.1 seconds
         prev_button_pressed = false;
         // Display waypoint number
         distanceIndicator.print(currentWaypoint);
         dispwp = true;
         }
      }
   else
      {
      inc1 = 0;
      }
   // Read Next Waypoint Button with debouncing
   if (digitalRead(NEXT_PIN) == LOW)
      {
      inc2++;
      if (inc2 > 105)
         {
         inc2 = 102;
         }
      if (inc2 == 100)
         {
         if (currentWaypoint < NUMOFWAYPOINTS - 1)
            {
            currentWaypoint++;
            }
         // Next button has been pressed for more than 0.1 seconds
         next_button_pressed = false;
         // Display waypoint number
         distanceIndicator.print(currentWaypoint);
         dispwp = true;
         }
      }
   else
      {
      inc2 = 0;
      }
   // Chek serial for incoming waypoint data
   lookforWaypoint();
   // Set current waypoint position
   currentWaypointLatitude = Waypoint[currentWaypoint][0];
   currentWaypointLongitude = Waypoint[currentWaypoint][1];
   delay(1);
   }
