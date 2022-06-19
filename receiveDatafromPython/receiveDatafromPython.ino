 /* Tommy Le
  * Northern Illinois University, Mechanical Engineering Dept
  * 6/19/2022
  * Program for receiving a string from Python and using it to 
  * turn on and off an LED on the board. This code is to be ran
  * with passData.py.
  */
 

String myCmd;

void setup() 
{
  Serial.begin(9600);                     // init port
  pinMode(LED_BUILTIN, OUTPUT);           // init LED
}

void loop() 
{
  while(Serial.available() == 0)          // wait for buffer to fill
  {
    
  }
  myCmd = Serial.readStringUntil('\r');   // read until \r
  if(myCmd == "ON")
  {
    digitalWrite(LED_BUILTIN, HIGH);      // if 'ON' turn on LED
  }
  if(myCmd == "OFF")
  {
    digitalWrite(LED_BUILTIN, LOW);       // if 'OFF' turn it off
  }
}
