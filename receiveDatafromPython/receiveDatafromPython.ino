// Program for receiving a string from Python and using it to turn on and off an LED on the board

String myCmd;

void setup() 
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  while(Serial.available() == 0)
  {
    
  }
  myCmd = Serial.readStringUntil('\r');
  if(myCmd == "ON")
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if(myCmd == "OFF")
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
