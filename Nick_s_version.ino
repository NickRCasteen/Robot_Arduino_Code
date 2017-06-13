//3 sensor algorithm
//****PACKAGES****
#include <Servo.h>
#include <NewPing.h>
//****DEFINITIONS****
#define sonar_num 3 //Number of sensors now 3
#define FrontSenseSend 7 //Front Sensor Send
#define FrontSenseRecieve 8 //Front Sensor Recieve
#define LeftSenseSend 6 //Left Sensor Send
#define LeftSenseRecieve 5 //Left Sensor Recieve
#define sigPinright 4 //Right Sensor Send
//Conveinience Vars
#define UPPERROTATE 1950 //For anytime 2000-esque variables are needed
#define LOWERROTATE 1050 //For anytime 1000-esque variables are needed
#define SenseDistanceSIDE 37 //Sensor distance for the side sensors
#define SenseDistanceFRONT 20 //Sensor distance for the front sensors
#define NINTYDEGREE 150 //Delay timer for 90 degrees
#define ONE_EIGHTY 380 //Delay timer for 180 degrees
#define SHIMMY 25 //Delay timer for a shimmy

//Servo Objects
Servo LEFTservo;
Servo RIGHTservo;

//GLOBAL VARIABLES ARE FOR SHIT PROGRAMMERS SO HERE'S MINE
long RIGHTTURN; //Reset after each cycle
long LEFTTURN; //Also Reset after each cycle
long LASTMOVE; //3 values, 0, 1 and 2. 0 is NULL, 1 is LEFT, 2 is RIGHT.
              //Used for if there's no wall either left or right.

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  MAIN FUNCTIONS  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup()
{
  delay(5000); //Start with a delay, so the Arduino can warm up.
  Serial.begin(9600);
  //OUTPUT SENSORS
  pinMode(FrontSenseSend, OUTPUT);
  pinMode(LeftSenseSend, OUTPUT);
  //INPOOT SENSORS
  pinMode(FrontSenseRecieve, INPUT);
  pinMode(LeftSenseRecieve, INPUT);
  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
  //SET UP SERVOS
  LEFTservo.attach(10);
  RIGHTservo.attach(9);
  
}

/*PLAN OF ACTIONS:
    Need global variable detailing whether we turned left or
      right. Resets when forward is fixed.
    Algorithm 1
    *FORWARD UNTO DAWN*  
        >>>Go forward until wall
        >>>Pause
    *SENSOR SIDE CHECKER*
        >>>Check left.
          >>If Wall nearby, set LEFTURN to 1
        >>>Check right
          >>If Wall nearby, set RIGHTTURN to 1
    *TURNING POINT*
        >>>Check variables
          >>If BOTH LEFTURN and RIGHTTURN are 1, THEN 180
          >>If BOTH LEFTURN and RIGHTTURN are 0:
            >>If LASTMOVE is LEFT, turn right, ENTER CORRECTION RIGHT
            >>If LASTMOVE is RIGHT, turn left, ENTER CORRECTION LEFT
            >>If LASTMOVE is NULL, turn right
          >>If LEFTURN is 1
            >>Turn Left. ENTER CORRECTION MODE LEFT.
            >>LASTMOVE set to LEFT
          >>If RIGHTTURN is 1
            >>Turn Right. ENTER CORRECTION MODE RIGHT.
            >>LASTMOVE set to RIGHT        
        >>>***CORRECTION MODE***
          >>If LEFTURN is 1
            >>If wall front and wall right, slight correction left
            >>When front > 40, return to forward mode.
          >>If RIGHTTURN is 1
            >>If wall front and wall left, slight correction right
            >>When front > 40, return to forward mode          
*/

void loop()
{
  RIGHTTURN = 0; //Reset this value
  LEFTTURN = 0; //Also a big fat reset.
  Forward_Unto_Dawn(); //Begin the loop until we hit a wall.
  Side_Sensor_Checker(); //This will set out Left and Right.
  TURNING_POINT(); //Use SSC's value changes to choose what to do, then do it.
  //From there, we just go back...forever. We will never stop.
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  TOOL FUNCTIONS  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void CORRECTION_R() //Make little jittery movements to get it right
{
  //Here, we've turned right. Check the left sensor for a wall and front for a wall.
  long distance,duration;
  digitalWrite(FrontSenseSend, HIGH);
  delay(200);
  digitalWrite(FrontSenseSend, LOW);
  duration=pulseIn(FrontSenseRecieve, HIGH);
  distance=(duration/2)/29.1;
  Serial.println(" ");
  Serial.println(distance);
  //BEGIN LOOP
  while(distance < 40) //Try to make front as far as possible
  {
    LEFTservo.writeMicroseconds(1950+5); //Shimmy to the right
    RIGHTservo.writeMicroseconds(1950); //Shimmy to the right
    delay(SHIMMY);
    ITS_TIME_TO_STOP();
    //Take New Pulse
    digitalWrite(FrontSenseSend, HIGH);
    delay(200);
    digitalWrite(FrontSenseSend, LOW);
    duration=pulseIn(FrontSenseRecieve, HIGH);
    distance=(duration/2)/29.1;
    Serial.println(" ");
    Serial.println(distance);
  }
}


void CORRECTION_L() //Same as above, but in the other direction.
{
  //Here, we've turned right. Check the left sensor for a wall and front for a wall.
  long distance,duration;
  digitalWrite(FrontSenseSend, HIGH);
  delay(200);
  digitalWrite(FrontSenseSend, LOW);
  duration=pulseIn(FrontSenseRecieve, HIGH);
  distance=(duration/2)/29.1;
  Serial.println(" ");
  Serial.println(distance);
  //BEGIN LOOP
  while(distance < 40)
  {
    LEFTservo.writeMicroseconds(1050-5);
    RIGHTservo.writeMicroseconds(1050);
    delay(SHIMMY);
    ITS_TIME_TO_STOP();
    //Take New Pulse
    digitalWrite(FrontSenseSend, HIGH);
    delay(200);
    digitalWrite(FrontSenseSend, LOW);
    duration=pulseIn(FrontSenseRecieve, HIGH);
    distance=(duration/2)/29.1;
    Serial.println(" ");
    Serial.println(distance);
  }
}


void go_FORWARD() //One step forward, then a one second pause to catch our breath.
{
  LEFTservo.writeMicroseconds(UPPERROTATE+5); //Left is 1500 to 2000
  RIGHTservo.writeMicroseconds(LOWERROTATE); //Right is 1500 to 1000
  delay(200);
}


void ITS_TIME_TO_STOP() //Stop that robutt
{
  LEFTservo.writeMicroseconds(1505); 
  RIGHTservo.writeMicroseconds(1500);//Right servo's stop value is between 91 and 93
}


void Turn_RIGHT() //RIght there on the label
{
  LEFTservo.writeMicroseconds(UPPERROTATE+5); //Turn left a bit.
  RIGHTservo.writeMicroseconds(UPPERROTATE); //Turn right bit
  LASTMOVE = 2; //LAST MOVE SET TO 2, RIGHT
  delay(NINTYDEGREE); //take a breather
  CORRECTION_R();
}

void Turn_LEFT() //What the fuck else is it, Harry?
{
  LEFTservo.writeMicroseconds(LOWERROTATE-5); //Turn left a bit.
  RIGHTservo.writeMicroseconds(LOWERROTATE); //Let's a go.
  LASTMOVE = 1; //LAST MOVE SET TO 1, LEFT
  delay(NINTYDEGREE); //take a breather
  CORRECTION_L();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//$$$$$$$$$$$$$$$$$$$$$$$  OH BOY THE THREE STEP PROSESS  $$$$$$$$$$$$$$$$$$$$$$
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//<<<< STEP 1 >>>>
void Forward_Unto_Dawn() //Forward until wall is found. A loop. Broken by a wall.
{
  while(1)
  {
    //Pulse the front sensor. If distance is > 15, go forward. If not, break.
    //OLD SENSOR PING CODE
    long distance,duration;
    digitalWrite(FrontSenseSend, HIGH);
    delay(200);
    digitalWrite(FrontSenseSend, LOW);
    duration=pulseIn(FrontSenseRecieve, HIGH);
    distance=(duration/2)/29.1;
    Serial.println(" ");
    Serial.println(distance);
    //OLD SENSOR PING CODE
    if (distance > SenseDistanceFRONT) //ALL IS WELL, KEEP GOING
    {
      go_FORWARD();
    }
    else //OBSTACLE DETECTED HLEP
    {
        ITS_TIME_TO_STOP();
        delay(2000); //Rest
        break; //End loop
    }
    
  }
}


//<<<< STEP 2 >>>>
void Side_Sensor_Checker() //When FUD hits a wall, check the left and right sensors.
{
 //CHECK LEFT
    long distance,duration;
    digitalWrite(LeftSenseSend, HIGH);
    delay(200);
    digitalWrite(LeftSenseSend, LOW);
    duration=pulseIn(LeftSenseRecieve, HIGH);
    distance=(duration/2)/29.1;
    Serial.println(" ");
    Serial.println(distance);
    //is there a wall?
    if(distance < SenseDistanceSIDE) //THere's a wall to our left!
    {
      RIGHTTURN = 1; //So we turn to the right...so set the value as such
    }
 //CHECK RIGHT
    pinMode(sigPinright,OUTPUT);
    digitalWrite(sigPinright,LOW);
    delay(200);
    digitalWrite(sigPinright,HIGH);
    delay(200);
    digitalWrite(sigPinright,LOW);
    pinMode(sigPinright, INPUT);
    duration = pulseIn(sigPinright, HIGH);
    distance = microsecondstoCentimeters(duration);
    //is there a wall?
    if(distance < SenseDistanceSIDE) //THere's a wall to our right!
    {
      LEFTTURN = 1; //So we turn to the left...so set the value as such
    }
 //Now either LEFTTURN or RIGHTTURN are 1...or both! OR NEITHER.  
}

//<<<< STEP 3 >>>>
void TURNING_POINT() //This is where we decide what to do, given our variables.
{
    if(LEFTTURN == 0 && RIGHTTURN == 0) //Check LASTMOVE, do opposite. Correct according
    {
      if(LASTMOVE == 0) //NULL
      {
        Turn_RIGHT(); //Correction handled in turning function. Not my problem.
      }
      else if(LASTMOVE == 1) //WENT LEFT
      {
        Turn_RIGHT(); //Opposite of left is right, come on, keep up!
      }
      else if(LASTMOVE == 2) //WENT RIGHT
      {
        Turn_LEFT(); //NOW we go left!
      }
    }
    else if(LEFTTURN == 0 && RIGHTTURN == 1) //Turn Right, correct right.
    {
      Turn_RIGHT();
    }
    else if(LEFTTURN == 1 && RIGHTTURN == 0) //Turn left, correct left.
    {
      Turn_LEFT();
    }
    else if(LEFTTURN == 1 && RIGHTTURN == 1) //Do a 180! WOAAAAAH *edge*
    {
      LEFTservo.writeMicroseconds(UPPERROTATE-5); //Turn left a bit.
      RIGHTservo.writeMicroseconds(UPPERROTATE); //Turn right bit
      delay(ONE_EIGHTY);
      //a 90 degree turn and another 90 degree turn is...
    }
}


long microsecondstoCentimeters(long microseconds) 
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
