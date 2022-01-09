/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

//Sensor's libraries
#include "MBed_Adafruit_GPS.h"
#include "SI7021.h"
#include "TCS3472_I2C.h"
#include "MMA8451Q.h"

//#define MMA8451_I2C_ADDRESS (0X1d<<1)



//Sensors variables

//GPS
BufferedSerial * gps_Serial = new BufferedSerial(PA_9, PA_10,9600);
Adafruit_GPS myGPS(gps_Serial);
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
const int refresh_Time = 2000;

//GPS Default
float latitude = 43.47;
char lat_ = 'N';
int lat_num = 0;
float longitude =	3.79;
char lon_ = 'W';
int lon_num = 1;

//TempHum Sensor
Si7021 temphum (PB_9, PB_8);
float temp;
float hum;

//Light sensor
AnalogIn light_signal(PA_4);
unsigned short lightIn;
int16_t lightInPCT;

//Soil moisture
AnalogIn soilMoisture(PA_0);
unsigned short moistureIn;
int16_t moistureInPCT;

//RGB Sensor
DigitalOut Red(PH_0);
DigitalOut Blue(PH_1);
DigitalOut Green(PB_13);
DigitalOut sLed(PB_7);

TCS3472_I2C rgb_sensor (PB_9, PB_8);

//MMA8451Q acc(PB_9,PB_8,0x1c<<1);
		
//RGB sensor
int rgb_readings[4]; //Declare a 4 element array to store RGB sensor readings


//Accel
float x,y,z;

//Threads
Thread thread(osPriorityNormal, 2048);

//Timeouts and tickers
Ticker sensor_tick_1;
Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?

//Flags
bool sensor_tick_event = true;

void sensor_tick_isr(void){
	sensor_tick_event = true;
}

//Functions of the threads
void sensors_thread(void);

using namespace std::chrono;
using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

/**
 * Dummy sensor class object
 */
DS1820  ds1820(PC_9);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;
static uint8_t DEV_EUI[] = { 0x7b, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0xda };
static uint8_t APP_KEY[] = { 0xf3,0x1c,0x2e,0x8b,0xc6,0x71,0x28,0x1d,0x51,0x16,0xf0,0x8f,0xf0,0xb7,0x92,0x8f };

/**
 * Entry point for application
 */
int main(void)
{
	
		//Setting up the GPS
		
		myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
													//a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf

    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
		
		thread.start(sensors_thread);
		sensor_tick_1.attach_us(sensor_tick_isr,5000000);
	
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
    lorawan_connect_t connect_params;
		connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;
		
    retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");
		
		
		


    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

void sensors_thread(void){
	
		while(1){
			
				refresh_Timer.start();  //starts the clock on the timer
						
				//GPS Initialization
				c = myGPS.read();   //queries the GPS
										
				//check if we recieved a new message from GPS, if so, attempt to parse it,
				if ( myGPS.newNMEAreceived() ) {
					if ( !myGPS.parse(myGPS.lastNMEA()) ) {
						continue;
					}
				}
				
				lightIn = (uint16_t)light_signal.read_u16();
				lightInPCT = lightIn*100/65535;
				
				moistureIn = (uint16_t)soilMoisture.read_u16();
				moistureInPCT = moistureIn*100/65535;
						
				temphum.get_data();	
				temphum.check();
				temp = ((float)(temphum.get_temperature() / 1000) + ((float)(temphum.get_temperature() % 1000)/1000));
				hum = ((float)(temphum.get_humidity() / 1000) + ((float)(temphum.get_humidity() % 1000)/1000));
				
				//rgb_sensor.getAllColors( rgb_readings ); // read the sensor to get red, green, and blue color data along with overall brightness
						
				//ACCELEROMETER
				//x=acc.getAccX();
				//y=acc.getAccY();
				//z=acc.getAccZ();
				
				
				if(sensor_tick_event){
					
						sensor_tick_event = false;
					
						//GPS								
						//check if enough time has passed to warrant printing GPS info to screen
						//note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
						if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {

								refresh_Timer.reset();
								//printf("GPS:\nTime: %d:%d:%d.%u\r\n", myGPS.hour+1, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
								//printf("Date: %d/%d/20%d\r\n", myGPS.day, myGPS.month, myGPS.year);
								//printf("Quality: %d\r\n", (int) myGPS.fixquality);
							
								if ((int)myGPS.fixquality > 0) {
										
//										latitude = myGPS.latitude/100.0;
//										lat_ = myGPS.lat;
//										longitude = myGPS.longitude/100.0;
//										lon_ = myGPS.lon;

//										if(lat_ == 'N'){
//												lat_num = 0;
//										}
//										else if(lat_ == 'S')
//										{
//												lat_num = 1;
//										}
//										
//										if(lon_ == 'E'){
//												lon_num = 0;
//										}
//										else if(lon_ == 'W')
//										{
//												lon_num = 1;
//										}
//										
//										printf(" Location: %5.2f %c, %5.2f %c\r\n", latitude, lat_, longitude, lon_);
										//printf("Speed: %5.2f knots\r\n", myGPS.speed);
										//printf("Angle: %5.2f\r\n", myGPS.angle);
										//printf("Altitude: %5.2fm\r\n", myGPS.altitude);
										//printf("Satellites: %d\r\n\n", myGPS.satellites);
								}
								else{
									latitude = 43.47;
									lat_ = 'N';
									lat_num = 0;
									longitude =	3.79;
									lon_ = 'W';
									lon_num = 1;
								//printf(" Lat: %2.2f %c, Lon: %2.2f %c\n", latitude, lat_, longitude, lon_);
								}
						}
						
						printf("LIGHT: %03u%%\n\n", lightInPCT);
						printf("SOIL MOISTURE: %03u%%\n\n", moistureInPCT);
						
						printf("TEMPERATURE: %02.1f C,\r\t\t\t", temp);
						printf("HUMIDITY: %02.1f%%\r\n\n", hum);
			
						printf(" Loc: %02.2f %c, %03.2f %c\r\n", latitude, lat_, longitude, lon_);
						printf("COLOR SENSOR: Clear: %d, Red: %d, Green: %d, Blue: %d\t",rgb_readings[0],rgb_readings[1],rgb_readings[2],rgb_readings[3]);
						//printf("ACCELEROMETER: X_Axis=%f,\tY_Axis=%f,\tZ_Axis=%f\n\n",x,y,z);
				
				
				}
				
				ThisThread::sleep_for(100ms);
		}
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{		
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;
	
	//DummySensorValue
    if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        //printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
        ds1820.startConversion();
			
    } else {
        printf("\r\n No sensor found \r\n");
        return;
    }

    packet_len = sprintf((char *) tx_buffer, "%03u%02.1f%02.1f%02.2f%d%03.2f%d%03u",
                         lightInPCT,temp,hum,latitude,lat_num,longitude,lon_num,moistureInPCT);
		
		//Location: 40.23 N,  3.38 W

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);
		//retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
    //                       MSG_CONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
		//uint8_t rx_buffer[30];

    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
		for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);			
		}
		if (rx_buffer[0]==0x4f && rx_buffer[1]==0x46 && rx_buffer[2]==0x46){ //means OFF
					Red = 0;
					Green = 0;
					Blue = 0;
					printf("Led OFF");
		}
		if (rx_buffer[0]==0x47 && rx_buffer[1]==0x72 && rx_buffer[2]==0x65 && rx_buffer[3]==0x65 && rx_buffer[4]==0x6e){ //means Green
					Red = 0;
					Green = 1;
					Blue = 0;
					printf("Led green");
		}
		if (rx_buffer[0]==0x52 && rx_buffer[1]==0x65 && rx_buffer[2]==0x64){ //means Red
					Red = 1;
					Green = 0;
					Blue = 0;
					printf("Led red");
		} 
		
    printf("\r\n");	
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF