/// @dir bandgap
/// Try reading the bandgap reference voltage to measure current VCC voltage.
/// @see http://jeelabs.org/2012/05/12/improved-vcc-measurement/
// 2012-04-22 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/sleep.h>

#define DEBUG	 0	 // set to 1 to display each loop() run and PIR trigger

#define SEND_MSG_EVERY	22		// Watchdog is a timerTick on a avg 8,2 sec timebase
															// SEND_MSG_EVERY=7	-> +- 1min
															// SEND_MSG_EVERY=14 -> +- 2min
															// SEND_MSG_EVERY=22 -> +- 3min
															// SEND_MSG_EVERY=26 -> +- 4min
															// SEND_MSG_EVERY=33 -> +- 5min

#define SEND_MSG_BATT_EVERY	90	 // Measure battery voltage every N messages
																	// MEASURE_EVERY=90 -> +- 4 hour

#define NODE_ID				 2				 // NodeId of this JeeNode
#define GROUP_ID				5				 // GroupId of this JeeNode

volatile unsigned char sendMsgTimer = SEND_MSG_EVERY;
volatile unsigned char sendMsgBatteryLevelTimer = SEND_MSG_BATT_EVERY;

//Message max 8 bytes
struct payload {
	 unsigned int nrOfPulses;
	 byte batteryLevel;				 //getVcc 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
	 bool doorOpenChanged;
	 bool doorOpen;
} payload;

//Number of pulses, used to measure energy.
volatile unsigned int	nrOfPulses = 0; //Unsigned int=32bit=max 4.294.967.295
volatile unsigned char pulseSendCounter = 0;
volatile unsigned char oldPulseSendCounter = 0;
volatile unsigned char deltaPulse = 0;
volatile bool sendMsg = false;
volatile bool sendMsgPulsesNow = false;
volatile bool timerTick = false;
volatile bool doorOpenChanged = false;
volatile bool adcDone = false;
volatile bool powerTest = false;

//Pins JeeNode port 1
#define PULSE_INPUT 3 //Port1.IRQ //Arduino pin volgorde
#define DOOR_INPUT	4 //Port1.DIO
#define POWER_TEST_INPUT 5 //Port2.DIO

ISR(WDT_vect) { 
	 Sleepy::watchdogEvent();
	 if(!timerTick) { timerTick = true; }
}

ISR(ADC_vect) { adcDone = true; }

//Pin change interrupt routine for PortD0..7, only PD4 is used
ISR(PCINT2_vect) { doorOpenChanged = true; }

//External interrupt 1
void pulseCount () { nrOfPulses++; pulseSendCounter++; }

static byte batteryLevelRead (byte count =4) {
	 set_sleep_mode(SLEEP_MODE_ADC);
	 ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
	 bitSet(ADCSRA, ADIE);
	 while (count-- > 0) {
			adcDone = false;
			while (!adcDone) sleep_mode();
	 }
	 bitClear(ADCSRA, ADIE);	
	 // convert ADC readings to fit in one byte, i.e. 20 mV steps:
	 //	1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	 return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
	 rf12_initialize(NODE_ID, RF12_868MHZ, GROUP_ID);
	
	 //Init the door input and pin change IRQ routine on this input
	 //Bron: http://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
	 pinMode(DOOR_INPUT,INPUT_PULLUP);
	 bitSet(PCICR,	PCIE2);				//Enable pin change interrupt for Port D (D0..D7)
	 bitSet(PCMSK2, DOOR_INPUT);	 //Choose which pins has to interrupt: D4
	
	 //Init the pulse input and IRQ routine on this input
	 pinMode(PULSE_INPUT, INPUT);
	 //pinMode(PULSE_INPUT, INPUT_PULLUP);
	 attachInterrupt(1, pulseCount, FALLING);
	 //EICRA = (EICRA & 0x0C) | (1<<ISC11) | (0<<ISC10);	//interrupt on int1's falling edge
	 bitSet(EICRA, ISC11);
	 bitClear(EICRA, ISC10);

	 //Read the powerTest pin to speed up power reading (less accurate)
	 pinMode(POWER_TEST_INPUT, INPUT_PULLUP); //On JeeNode:P2-DIO2
	 powerTest = !digitalRead(POWER_TEST_INPUT);
	 
	 Sleepy::watchdogInterrupts(6); //Start the watchdog timer for time base
}

void loop() {
	 if (doorOpenChanged) {
			doorOpenChanged = false;
			payload.doorOpenChanged = true;
			sendMsg = true;
	 }
	 
	 if (timerTick) 
	 { // There has ben a Watchdog interrupt for time measurement
			timerTick = false;

			if (!powerTest) {
				 //Normal situation, power test not active
				 deltaPulse = oldPulseSendCounter >> 2; //Divide pulses (power) by 4, which is a 25% change
				 if (deltaPulse == 0) deltaPulse = 1; //Limit deltaPulse by 1, this is a minimum of aprox. 44W
				 
				 if ((pulseSendCounter < oldPulseSendCounter - deltaPulse) || (pulseSendCounter > oldPulseSendCounter + deltaPulse)) {
						//Power is changed more than -25% or +25%
						sendMsgTimer = SEND_MSG_EVERY;
				 } else {
						sendMsgTimer++;
				 }
				 oldPulseSendCounter = pulseSendCounter;
				 pulseSendCounter = 0;
			} else {
				 //Power Test active, speed up power reading (but less accurate)
				 if ((pulseSendCounter >= 5) || (sendMsgTimer >= 7)) {
						//Send if a little pulses are counted, or more time than 7 timerticks (about 63sec) exceeded
						sendMsgTimer = SEND_MSG_EVERY;
						pulseSendCounter = 0;
				 } else {
						sendMsgTimer++;
				 }
			}
			
			if (sendMsgTimer >= SEND_MSG_EVERY) {
				 sendMsgTimer = 0;
				
				 sendMsgBatteryLevelTimer++;
				 if (sendMsgBatteryLevelTimer >= SEND_MSG_BATT_EVERY) {
					 sendMsgBatteryLevelTimer = 0;
					 payload.batteryLevel = batteryLevelRead();
				 }
				 sendMsg = true;
			}
	 }
	 
	 if (sendMsg) 
	 {
			sendMsg = false;
			
			payload.nrOfPulses = nrOfPulses;
			payload.doorOpen = !digitalRead(DOOR_INPUT);
			
			rf12_sleep(RF12_WAKEUP);
			while (!rf12_canSend())
			rf12_recvDone();
			rf12_sendStart(0, &payload, sizeof payload);
			rf12_sendWait(1);
			rf12_sleep(RF12_SLEEP);
			
			payload.doorOpenChanged = false;
	 }
	 Sleepy::watchdogInterrupts(9);		// Start the watchdog timer for timerTick 6=5,2sec 9=8,31sec
	 Sleepy::powerDown();
}

