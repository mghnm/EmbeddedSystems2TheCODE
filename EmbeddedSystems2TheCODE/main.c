
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "lcd.h"
#include "i2c.h"
#include "font.h"
#include <stdio.h>
#include <avr/common.h>
#include <stdbool.h>
#include "uart.h"


//Left motor operated internally by 3 inputs. Two control inputs and one PWM signal defined below as follows.
#define L_CTRL_1 PIND2
#define L_CTRL_2 PIND4
#define PWM_L PIND5 //OC0B

//Same as the left motor
#define R_CTRL_1 PIND7
#define R_CTRL_2 PINB0
#define PWM_R PIND6 //OC0A


//Definitions that allow us to more easily select which sensor we are targeting.
#define IR_LEFT 2
#define IR_RIGHT 3
#define OPTICAL_LEFT 4
#define OPTICAL_RIGHT 5
#define OPTICAL_FRONT 1

//Offset of Accel Data
#define MPU_AXOFFSET 0.02
#define MPU_AYOFFSET 0.05
#define MPU_AZOFFSET -0.2

#define HIT_THRESHHOLD 0.18

//Function prototypes for assisting subroutines.
void initializeMotors();
void initializeUltrasonic();
void initializePWM();
void initializePWM();
void initializeADC();
void startConversion();
void selectChannel(uint8_t analogPin);
uint16_t readSensor(uint8_t analogPin);
void sendUltraSonicSignal();

void analogWrite(uint8_t pin, uint8_t dutyCycle);


//Motor subroutines supporting CW and CCW rotation, coasting, braking and pivoting.
void rightFwd(uint8_t spd);
void rightRev(uint8_t spd);
void leftFwd(uint8_t spd);
void leftRev(uint8_t spd);
void rightStop();
void leftStop();
void rightMotor(int speed);
void leftMotor(int speed);
void drive(int speed);
void stop();


//Array of unsigned integers used to store up to 6 different sensor values located on different analog channels.
volatile uint16_t sensorValues[6];

//Initial channel set to Infra red sensor right.

volatile uint8_t currentAnalogChannel = OPTICAL_FRONT;

//volatile uint8_t currentAnalogChannel = OPTICAL_FRONT;

//Interrupt service routine to update the sensor value array continuously using interrupts
ISR(ADC_vect){
	
	
	//There is a simple logic change that would allow up to 6 sensors 0-5 analog pins
	
	if(currentAnalogChannel > OPTICAL_RIGHT){
		currentAnalogChannel = OPTICAL_FRONT;
		} else {}
		
		
		
		selectChannel(currentAnalogChannel);
		sensorValues[currentAnalogChannel] = ADCW;
		currentAnalogChannel++;
		startConversion();
		
		
	}
	
	volatile char run = '1';
	ISR(USART_RX_vect){
		
		switch(UDR0){
			case 'A':
			run = '1';
			break;
			case 'B':
			leftMotor(0);
			rightMotor(0);
			_delay_ms(5000);
			break;
			case 'C':
			leftMotor(0);
			rightMotor(0);
			run = '0';
			break;
			default:
			break;
		}
		
		
	}
	
	double raw;
	uint16_t numuS;
	uint8_t hit = 0;
	
	
	ISR(PCINT1_vect) {
		if (bit_is_set(PINC,PORTC3)) {					// Checks if echo is high
			TCNT1 = 0;								// Reset Timer
			} else {
			numuS = TCNT1;					// Save Timer value
			uint8_t oldSREG = SREG;
			cli();									// Disable Global interrupts
			raw = numuS * 0.00107388316;

			SREG = oldSREG;							// Enable interrupts
			// Toggle debugging LED
		}
		
	}


	/* ****************************** Start of main *****************************/

	int main(void){
		
		initUART0();
		initializeMotors();
		initializeADC();
		initializeUltrasonic();
		DDRB =	0xFF;
		
		char distance[100];
		char axg_arr[10];
		char ayg_arr[10];
		char azg_arr[10];
		
		lcd_init(0xAF);    // init lcd and turn on
		//init mpu6050
		mpu6050_init();
		_delay_ms(50);
		
			#if MPU6050_GETATTITUDE == 0
			int16_t ax = 0;
			int16_t ay = 0;
			int16_t az = 0;
			int16_t gx = 0;
			int16_t gy = 0;
			int16_t gz = 0;
			double axg = 0;
			double ayg = 0;
			double azg = 0;
			double gxds = 0;
			double gyds = 0;
			double gzds = 0;
			#endif

		while(1){
			_delay_ms(60);
			
			
			#if MPU6050_GETATTITUDE == 0
			mpu6050_getConvAccelData(&axg, &ayg, &azg);
			axg = axg + MPU_AXOFFSET;
			ayg = ayg + MPU_AYOFFSET;
			azg = azg + MPU_AZOFFSET;
			
			#endif
			
			#if MPU6050_GETATTITUDE == 0
			
			if((axg > HIT_THRESHHOLD || axg < -HIT_THRESHHOLD) || (ayg > HIT_THRESHHOLD || ayg < -HIT_THRESHHOLD)){
				hit = 1;
			}
			#endif
			
			sendUltraSonicSignal();
			dtostrf(raw, 3, 1, distance);
			
			lcd_gotoxy(0,0);
			lcd_puts("Distance: ");
			lcd_puts(distance);
			lcd_puts("   ");
			lcd_display();
					
			
		    lcd_gotoxy(0,5);
			lcd_puts("Status: ");
			lcd_puts(hit);
			lcd_puts("   ");
			lcd_display();
			
			
			
			
			if(raw > 10 && !hit){
				drive(126);
			} else {
				drive(0);
			}
			
		}
		
		return 0;
	}


	/* ****************************** End of Main *****************************/



	//Changes the contents of the relevant registers to allow for PWM on 2 separate pins that serve as input for the motor-driver module.
	void initializePWM(){
		
		DDRD |= (1 << PWM_L) | (1 << PWM_R);
		
		OCR0A = 0;
		OCR0B = 0;

		// set none-inverting mode
		TCCR0A |= (1 << COM0A1);
		TCCR0A |= (1 << COM0B1);

		// set fast PWM Mode
		TCCR0A |= (1 << WGM01) | (1 << WGM00);
		
		
		// set prescaler to 8 and starts PWM
		//Temp disabled prescaler
		TCCR0B |= (1 << CS02) | (1 << CS00);
	}

	//initializeMotors is a convenient subroutine which can be called in order set the required pins to output and initializes PWM with the expectation
	//that the motors should not work if PWM has not been initialized.
	void initializeMotors(){
		initializePWM();
		
		DDRD |= (1 << L_CTRL_1) | (1 << L_CTRL_2) | (1 << R_CTRL_1);
		DDRB |= (1 << R_CTRL_2);
		
	}
	
	//Initialization of pin mask interrupt for the echo to get accurate readings
	void initializeUltrasonic(){
		
		DDRC = 0xFF;							// Port C all output. PC0: RW		PC1: RS		PC2: E
		DDRC &= ~(1<<DDC3);						// Set Pin C5 as input to read Echo
		PORTC |= (1<<PORTC3);					// Enable pull up on C5
		PORTC &= ~(1<<PORTC2);						// Init C4 as low (trigger)

		PRR &= ~(1<<PRTIM1);					// To activate timer1 module
		TCNT1 = 0;								// Initial timer value
		TCCR1B |= (1<<CS10);					// Timer without prescaller. Since default clock for atmega328p is 1Mhz period is 1uS
		TCCR1B |= (1<<ICES1);					// First capture on rising edge

		PCICR = (1<<PCIE1);						// Enable PCINT[14:8] we use pin C5 which is PCINT13
		PCMSK1 = (1<<PCINT11);					// Enable C5 interrupt
	}

	//Rotate the leftMotor CW
	void leftFwd(uint8_t spd){
		PORTD |= (1 << L_CTRL_1);
		PORTD &= ~(1 << L_CTRL_2);
		analogWrite(PWM_L, spd);
		
	}

	//Rotate the leftMotor CCW
	void leftRev(uint8_t spd){
		PORTD &= ~(1 << L_CTRL_1);
		PORTD |= (1 << L_CTRL_2);
		analogWrite(PWM_L, spd);
	}

	//Rotate right motor CW
	void rightFwd(uint8_t spd){
		PORTD |= (1 << R_CTRL_1);
		PORTB &= ~(1 << R_CTRL_2);
		analogWrite(PWM_R, spd);
	}

	//Rotate right motor CCW
	void rightRev(uint8_t spd){
		PORTD &= ~(1 << R_CTRL_1);
		PORTB |= (1 << R_CTRL_2);
		analogWrite(PWM_R, spd);
	}

	//Stop left motor
	void leftStop(){
		PORTD &= ~(1 << L_CTRL_1);
		PORTD &= ~(1 << L_CTRL_2);
		analogWrite(PWM_L, 0);
	}

	//Stop right motor
	void rightStop(){
		PORTD &= ~(1 << R_CTRL_1);
		PORTB &= ~(1 << R_CTRL_2);
		analogWrite(PWM_R, 0);
	}

	//Stop both motors
	void stop(){
		leftStop();
		rightStop();
	}

	//Takes in a value either positive or negative that will drive both motors at the same speed. If the value is negative the car will move CCW or in reverse
	void drive(int speed){
		if(speed > 0){
			leftFwd((uint8_t) abs(speed));
			rightRev((uint8_t) abs(speed));
			} else {
			leftRev((uint8_t) abs(speed));
			rightFwd((uint8_t) abs(speed));
		}
	}

	void rightMotor(int speed){
		if(speed > 0){
			rightFwd((uint8_t) abs(speed));
			} else {
			rightRev((uint8_t) abs(speed));
		}
	}

	void leftMotor(int speed){
		if(speed > 0){
			//leftFwd((uint8_t) abs(speed));
			leftRev((uint8_t) abs(speed));
			} else {
			//leftRev((uint8_t) abs(speed));
			leftFwd((uint8_t) abs(speed));
		}
	}
	
	void sendUltraSonicSignal(){
		PORTC |= (1<<PORTC2);						// Set trigger high
		_delay_us(10);							// for 10uS
		PORTC &= ~(1<<PORTC2);						// to trigger the ultrasonic module
	}

	
	//analogWrite works in conjunction with initializePWM where we enable timer0, OCR0B and OCR0A. YOU MUST INITIALIZE PWM BEFORE ANALOGWRITE WOULD WORK.
	//This sub-routine defines the dutyCycle of the motors by writing to the respective timer0 registers which will drive the motors.
	void analogWrite(uint8_t pin, uint8_t dutyCycle){
		
		//TODO: Do some checking so that value is between 0-255
		
		if(pin == PWM_L){
			OCR0B = dutyCycle;
			}else if(pin == PWM_R){
			OCR0A = dutyCycle;
			}else{
			//Do something weird
		}
	}

	//Initializes the ADC registers to support the IR and optical sensors.
	void initializeADC(){
		//Configure the ADMUX for AVCC as input (internal 5v reference) / not left adjusted results (10-bit resolution) / ADC0 as input / 0b0100:0000
		ADMUX = 0x40;
		//Configure ADCSRA for enable ADC / don't start conversion / prescaler 128 / disable auto-trigger / enable ADC interrupt 0b10001:111
		ADCSRA = 0x8F;
		
		selectChannel(OPTICAL_FRONT);
		sei();
		startConversion();
	}

	//Takes an analog pin input as previously defined e.g. IR_LEFT IR_RIGHT to initialize the ADMUX with the proper channel
	void selectChannel(uint8_t analogPin){
		ADMUX = (ADMUX & 0xE0) | (analogPin & 0x1F);   //select channel (MUX0-4 bits)
	}

	//Read sensor takes in the latest value stored in the sensors array at the specified index and returns it. This value is updated via the ADC_vect interrupt.
	uint16_t readSensor(uint8_t analogPin){
		return sensorValues[analogPin + 1];
	}

	//For convenience startConversion is used instead of the bit maths.
	void startConversion(){
		ADCSRA |= (1 << ADSC);
	}

	//Returns 1 if there is an obstacle detected on that sensor (call only on OPTICAL sensors aka OPTICAL_FRONT OPTICAL_LEFT OPTICAL_RIGHT).
	uint8_t checkObstacle(uint8_t analogPin){
		uint16_t analogSignal = readSensor(analogPin);
		if(analogSignal > 360){
			return 1;
			} else{
			return 0;
		}
	
	}
