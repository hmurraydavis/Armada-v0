/* 
Writen by Halie Murray-Davis, hmurraydavis@gmail.com.
*/

//#include <nmea.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Stepper.h>
#include "TinyGPS.h"
TinyGPS gps;

//Tuning constants:
#define KRUDDER 3
#define STEPSAT90 90 //The number of steps the stepper has traveled when the sail gets to 90 degrees

//DIGITAL PINS FOR RC OVERRIDE CONTROL
#define RC_TOGGLE 11
#define RUDDER_RC 12
#define SAIL_RC 13
#define RUDDER_PIN 3
#define GPS_RX 10
#define GPS_TX 11

//Physical parameters:
#define LBOOM .3 //length of the boom in cm



// Define globals for first pass at GPS working:
long lat, lon;
unsigned long fix_age, time, date, speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;

// create a GPS data connection to GPRMC sentence type
//NMEA gps(GPRMC);

//Initialize software objects:
SoftwareSerial gpsSerial(10, 11); //RX, TX //TODO: replace with defined variables. 

Servo rudder;

const int stepsPerRevolution = 48;
Stepper sail(stepsPerRevolution, 4,5,6,7); //Connect stepper on pins 4,5,6,& 7

//Create variable to keep track of sail stepper's position:
int currentSailPosition = 0;



void setup() {
  // Start serial coms for debuging via the serial monitor and reading back GPS data:
  Serial.begin(9600);
  gpsSerial.begin(9600);

  // Set the pins for RC control of sail and rudder as inputs
  pinMode(RUDDER_RC, INPUT);
  pinMode(SAIL_RC, INPUT);

  // Initialize the rudder servo objects:
  rudder.attach(RUDDER_PIN);
  sail.setSpeed(60);
}


void takeRCControl(){
  int rudderSetPoint = pulseIn(RUDDER_RC, HIGH, 25000); 
  setRudderPosition(rudderSetPoint) ;
  int sailSetPoint = pulseIn(SAIL_RC, HIGH, 25000);
  setSailPosition(sailSetPoint);
}


int windDirection(){
  return int(analogRead(A0)/72);
}


void setRudderPosition(int desiredRudderAngle){
  rudder.write(desiredRudderAngle);
}

void setSailPosition(int stepsToMoveSail){
  sail.step(stepsToMoveSail);
}


int sailStepsToMove(int desiredSailAngle){
   /*Calculate number of steps the sail stepper motor should move
   * to get to a desired angle. Doesn't pay attention to wind
   * direction; only does the math to figure out what the stepper 
   * should do to allow a given sail angle. 
   * 
   * NOTES:
   * *This function uses a band pass filter to avoid excessive 
   * sail actuation. If less than 5 degrees of sail motion are needed,
   * the number of steps to move is zero. 
   * *This function does not account for slip of the stepper motor and 
   * does not employ any closed loop control. 
   * 
   * INPUT:
   *    int desiredSailAngle: The angle in degrees that the sail should be set at
   * OUTPUT:
   *    int stepsToMoveSail: The number of steps the stepper motor should turn to 
   *      allow the sail to move to the desired position.
   */
  int sailSteps = LBOOM * desiredSailAngle /STEPSAT90;
  int stepsToMoveSail = currentSailPosition - sailSteps;

  //Filter sail movement so only larger changes are made.
  if (abs(stepsToMoveSail) < 0){
    return 0;  
  }
  else{
    return stepsToMoveSail;
  }
}


int rudderSetPoint(float desLatitude, float desLongitude){
  /* Calculate current set point for rudder using most recent GPS position and 
  loaction of next GPS waypoint. Only does straight line to waypoints. 

  Currently uses very simple porportional control to set rudder position.
  
  INPUT: 
    float desLatitude: the latitude of the waypoint
    float desLongitude: the longitude of the waypoint
  OUTPUT: 
    int rudderSetPoint: servo position (in degrees) that the rudder should be set at to reach the waypoint
  */
  int rudderSetPoint = 0;
  //TODO: replace with actual heading: 
  float headingToPoint = 2.0; //gps.gprmc_course_to(desLatitude, desLongitude) - gps.gprmc_course();

  //Bandpass filter to prevent excessive rudder action: 
  if (headingToPoint < 5){
    rudderSetPoint = 0;
  }
  else{ //If the boat is too far off track, do corrective rudder action
    //Porportional controller for rudder course correction:
    //Rudder needs to turn in opposite direction to error to stear boat back on track
    rudderSetPoint = (-1) * headingToPoint/KRUDDER; 
  }
  return int(rudderSetPoint);
}

void loop() {
  Serial.println("Hi!!");
  if (gpsSerial.available() > 0 ) {
    //Serial.println("trying to read!");
    char c = gpsSerial.read();
    //Serial.print("c is: "); 
    Serial.println(c);
    

      // Process GPS data!
    if (gps.encode(c)) {
      
      // retrieves +/- lat/long in 100000ths of a degree
      gps.get_position(&lat, &lon, &fix_age);
       
      // time in hhmmsscc, date in ddmmyy
      gps.get_datetime(&date, &time, &fix_age);
       
      // returns speed in 100ths of a knot
      speed = gps.speed();
       
      // course in 100ths of a degree
      course = gps.course();
      
      Serial.println(date);

    }
    

  }
   
  
  //Old library version: 
  /*
  if (gpsSerial.available() > 0 ) {
    //Serial.println("trying to read!");
    char c = gpsSerial.read();
    //Serial.print("c is: "); 
    Serial.println(c);
    

      // check if the character completes a valid GPS sentence
    if (gps.decode(c)) {
      // check if GPS positioning was active
      Serial.print("Status is: "); Serial.println(gps.gprmc_status() );
      if (gps.gprmc_status() == 'A') {
        Serial.println("In A loop!");
      Serial.println(gps.gprmc_latitude() );
        // check if you are in Colorado, USA

    }
    

  }
  } */
  setSailPosition(40);
  delay(100);

}
