#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


#define MOTOR_1 8
#define MOTOR_2 9

#define LED_1 1
#define LED_2 16
#define LED_3 15

void hardwareSetup(){
	if(wiringPiSetup() == -1){
		printf("There is an error at gpio pin setup");
		exit(-1);
	} 

	// Motor pins preallocated for vibratiom motors
	pinMode(MOTOR_1,OUTPUT);
	pinMode(MOTOR_2,OUTPUT);

	// This pins preallocated for leds
	pinMode(1,OUTPUT);
	pinMode(15,OUTPUT);
	pinMode(16,OUTPUT);
}

int vibrationStateChange(int number){
	if(number == 0){
		digitalWrite(MOTOR_1,0);
		digitalWrite(MOTOR_2,0);		
	}
	else if(number == 1)
		digitalWrite(MOTOR_1,1);
	else if(number == 2){
		digitalWrite(MOTOR_1,1);
		digitalWrite(MOTOR_2,1);
	}
	else{
		fprintf(stderr,"Number of vibrationStateChange parameter is it should be in between [0,2]\n Current parameter value is %d\n",number);
		return -1;	
	}

	return 0;
}

int ledStateChange(int number){
	if(number == 0){
		digitalWrite(LED_1,0);
		digitalWrite(LED_2,0);
		digitalWrite(LED_3,0);		
	}
	else if(number == 1)
		digitalWrite(LED_1,1);
	else if(number == 2){
		digitalWrite(LED_1,1);
		digitalWrite(LED_2,1);
	}
	else if(number == 3){
		digitalWrite(LED_1,1);
		digitalWrite(LED_2,1);
		digitalWrite(LED_3,1);		

	}
	else{
		fprintf(stderr,"Number of ledStateChange parameter is it should be in between [0,3]\n Current parameter value is %d\n",number);
		return -1;	
	}

	return 0;
}

void reset(){
	ledStateChange(0);
	vibrationStateChange(0);
}

void delayMS(int ms){
	delay(ms);
}

