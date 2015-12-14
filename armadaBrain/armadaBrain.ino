#include <nmea.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Stepper.h>

//Tuning constants:
#define KRUDDER 3
#define STEPSAT90 90 //The number of steps the stepper has traveled when the sail gets to 90 degrees

//Physical parameters:
#define LBOOM .3 //length of the boom in cm

// create a GPS data connection to GPRMC sentence type
NMEA gps(GPRMC);

//Initialize software objects:
SoftwareSerial gpsSerial(10, 11); //RX, TX
Servo rudder;
const int stepsPerRevolution = 200;
Stepper sail(stepsPerRevolution, 4,5,6,7); //Connect stepper on pins 4,5,6,& 7

int currentSailPosition = 0;



void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}


int windDirection(){
  return analogRead(A0)/72;
}

void setSailPosition(int stepsToMoveSail){
  int cat = 8;
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
  float headingToPoint = gps.gprmc_course_to(desLatitude, desLongitude) - gps.gprmc_course();

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
  }

}
