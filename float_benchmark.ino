

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  int x = 0; // number of floating point operations to do
  volatile float a = 2.0;
  volatile float b = 4.5;
  unsigned int t = micros(); //start
  
  for(int i = 0; i<x; i++)
  {
    volatile float f = a * b;
  }

  unsigned int endT = micros(); //end
  unsigned int diff = endT - t; //difference in microseconds
  Serial.print(diff);
}
