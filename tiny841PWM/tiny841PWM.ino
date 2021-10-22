/*
 * Make sure CKDIV8 is set so board runs at 8MHz!
 */

const bool freq = 1; //0=10kHz, 1=20kHz

void setup() {
  // put your setup code here, to run once:

  DDRB = (1 << PB2);//PA2 pin as an output
  TOCPMSA1 = (1 << TOCC7S0);//TOCC1 linkage
  TOCPMCOE = (1 << TOCC7OE);//Enable PWM
  TCCR1A = (1 << COM1A1) | (1 << WGM11);//Fast PWM 1110
  TCCR1B = (1 << CS00) | (1 << WGM12) | (1 << WGM13);//Fast PWM 1110

  if(freq == 0)
  {
    ICR1 = 820; // 10 KHz
 
    OCR1A = 7; //1% Duty Cycle
  }
  else
  {
    ICR1 = 410; //20 kHz
    OCR1A = 3; // 1% Duty Cycle
  }


}

void loop() {
  // put your main code here, to run repeatedly:

}
