/*
Robot version 1
ATtiny85, 2 motors, LED, two sensors
Sensor stuff: 
	both white	:	no prior	:	random walk
				:	prior		:	turn in direction of last black
	
	one white	:	ideal situation, move straight
	
	both black	:	no prior	:	random walk
				:	prior		:	turn in direction of last white
This code was written by Eric Heisler (shlonkin) Updated Jeff Chap (Mr. Spriggs)
It is in the public domain, so do what you will with it.
*/
#include "Arduino.h"

#include <avr/sleep.h> 

// Here are some parameters that you need to adjust for your setup
// Base speeds are the motor pwm values. Adjust them for desired motor speeds.
#define baseLeftSpeed 28
#define baseRightSpeed 50

// This determines sensitivity in detecting black and white
// measurment is considered white if it is > whiteValue*n/4
// where n is the value below. n should satisfy 0<n<4
// A reasonable value is 3. Fractions are acceptable.
#define leftSensitivity 3
#define rightSensitivity 3

// the pin definitions
#define lmotorpin 0 // PB0 pin 5 
#define rmotorpin 1 // PB1 pin 6
#define lsensepin 1 //ADC1 pin 7
#define rsensepin 3 //ADC3 pin 2
#define ledpin 4 //PB4 pin 3

// some behavioral numbers
// these are milisecond values
#define steplength 2500
#define smallturn 150
#define bigturn 500
#define memtime 1000

// variables
uint8_t lspd, rspd; // motor speeds
uint16_t lsenseval, rsenseval, lwhiteval, rwhiteval; // sensor values

void move(uint8_t lspeed, uint8_t rspeed);
void stop();
void senseInit();
void flashLED(uint8_t flashes);

// just for convenience and simplicity (HIGH is off)
#define ledoff PORTB  &= ~(1 << 4)
#define ledon PORTB |= (1 << 4)

void setup(){
        PRR |= (1 << PRUSI); //Set byte; disables the USI
        WDTCR &= ~(1 << WDE); //Set byte; disables the Watchdog Timer
	// setup pins
	pinMode(lmotorpin, OUTPUT);
	pinMode(rmotorpin, OUTPUT);
	pinMode(2, INPUT); // these are the sensor pins, but the analog
	pinMode(3, INPUT); // and digital functions use different numbers
	ledoff;
	pinMode(ledpin, OUTPUT);
	analogWrite(lmotorpin, 0);
	analogWrite(rmotorpin, 0);
	
	lspd = baseLeftSpeed;
	rspd = baseRightSpeed;
        
	flashLED(4);
	//delay(250);
        setupADCSleepmode();
	senseInit();
}

void loop(){
	
	// now look for an edge
	uint8_t lastMove = 1; //0=straight, 1=left, 2=right
	unsigned long moveEndTime = 0; // the millis at which to stop
	unsigned long randomBits = micros(); // for a random walk
	
	unsigned long prior = 0; // after edge encounter set to millis + memtime
	uint8_t priorDir = 0; //0=left, 1=right, 2=both
	uint8_t lastSense = 1; //0=edge, 1=both white, 2=both black
	uint8_t i = 0; // iterator
	

	while(true){
		// only read sensors about once every 20ms
		// it can be done faster, but this makes motion smoother
		
                
		// read the value 4 times and average
		
		lsenseval = 0;
		rsenseval = 0;
                ledon;
                PRR &= ~(1 <<PRADC); //clear ADC shutdown
                ADCSRA |= (1 << ADEN); //enable ADC
                ACSR |= (1<< ACD);  //disables the Analog Comparator
                
		for(i=0; i<4; i++){
                        /*ADMUX |= (0b00001111 & PB1);
                        sleep_mode();
		        lsenseval += ADC;
                        ADMUX |= (0b00001111 & PB3);
                        sleep_mode();
		        rsenseval += ADC;*/
                        //sleep_enable();
                        //sleep_cpu();
			lsenseval += analogRead(lsensepin);
			rsenseval += analogRead(rsensepin);
                        //sleep_disable();
		}
 
		ledoff;
		// don't divide by 4 because it is used below
                ADCSRA &= ~(1 << ADEN); //disable ADC
                PRR |= (1 << PRADC); //set ADC shutdown
               
		// refill the random bits if needed
		if(randomBits == 0){ randomBits = micros(); }
		
		// Here is the important part
		// There are four possible states: both white, both black, one of each
		// The behavior will depend on current and previous states
		if((lsenseval > lwhiteval) && (rsenseval > rwhiteval)){
			// both white - if prior turn toward black, else random walk
                        
                        //senseInit();
			if(lastSense == 2 || millis() < prior){
				// turn toward last black or left
				if(priorDir == 0){
					moveEndTime = millis()+smallturn;
					move(0, rspd); // turn left
					lastMove = 1;
				}else if(priorDir == 1){
					moveEndTime = millis()+smallturn;
					move(lspd, 0); // turn right
					lastMove = 2;
				}else{
					moveEndTime = millis()+bigturn;
					move(0, rspd); // turn left a lot
					lastMove = 1;
				}
			}else{
				// random walk
				if(millis() < moveEndTime){
					// just continue moving
				}else{
					if(lastMove){
						moveEndTime = millis()+steplength;
						move(lspd, rspd); // go straight
						lastMove = 0;
					}else{
						if(randomBits & 1){
							moveEndTime = millis()+smallturn;
							move(0, rspd); // turn left
							lastMove = 1;
						}else{
							moveEndTime = millis()+smallturn;
							move(lspd, 0); // turn right
							lastMove = 2;
						}
						randomBits >>= 1;
					}
				}
			}
			lastSense = 1;
			
		}else if((lsenseval > lwhiteval) || (rsenseval > rwhiteval)){
			// one white, one black - this is the edge
			// just go straight
			moveEndTime = millis()+steplength;
			move(lspd, rspd); // go straight
			lastMove = 0;
			lastSense = 0;
			prior = millis()+memtime;
			if(lsenseval > lwhiteval){
				// the right one is black
				priorDir = 1;
			}else{
				// the left one is black
				priorDir = 0;
			}
			
		}else{
			// both black - if prior turn toward white, else random walk
			if(lastSense == 1 || millis() < prior){
				// turn toward last white or left
				if(priorDir == 0){
					moveEndTime = millis()+smallturn;
					move(lspd, 0); // turn right
					lastMove = 2;
				}else if(priorDir == 1){
					moveEndTime = millis()+smallturn;
					move(0, rspd); // turn left
					lastMove = 1;
				}else{
					moveEndTime = millis()+bigturn;
					move(lspd, 0); // turn right a lot
					lastMove = 2;
				}
			}else{
				// random walk
				if(millis() < moveEndTime){
					// just continue moving
				}else{
					if(lastMove){
						moveEndTime = millis()+steplength;
						move(lspd, rspd); // go straight
						lastMove = 0;
					}else{
						if(randomBits & 1){
							moveEndTime = millis()+smallturn;
							move(0, rspd); // turn left
							lastMove = 1;
						}else{
							moveEndTime = millis()+smallturn;
							move(lspd, 0); // turn right
							lastMove = 2;
						}
						randomBits >>= 1;
					}
				}
			}
			lastSense = 2;
		}
	}
}

void move(uint8_t lspeed, uint8_t rspeed){
	//analogWrite(lmotorpin, lspeed);
	//analogWrite(rmotorpin, rspeed);
}

void stop(){
	analogWrite(lmotorpin, 0);
	analogWrite(rmotorpin, 0);
}

// stores the average of 16 readings as a white value
void senseInit(){
	lwhiteval = 0;
	rwhiteval = 0;
	ledon;
	delay(1);

	for(uint8_t i=0; i<16; i++){
                ADMUX |= (0b00001111 & PB1);
                sleep_mode();
		lwhiteval += ADC;
                ADMUX |= (0b00001111 & PB3);
                sleep_mode();
		rwhiteval += ADC;
		delay(1);
	}

	lwhiteval >>= 4;
	rwhiteval >>= 4;
        lwhiteval = lwhiteval*leftSensitivity;
        rwhiteval = rwhiteval*rightSensitivity;
        
        ledoff;
}

void flashLED(uint8_t flashes){
	while(flashes){
		flashes--;
		ledon;
		delay(200);
		ledoff;
		if(flashes){ delay(500); }
	}
}

void setupADCSleepmode(void) {
  set_sleep_mode(SLEEP_MODE_ADC);            /* defined in avr/sleep.h */
  ADCSRA |= (1 << ADIE);                       /* enable ADC interrupt */
  sei();                                   /* enable global interrupts */
}

EMPTY_INTERRUPT(ADC_vect);
