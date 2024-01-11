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
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "mbed_version.h"
#include "mbed.h"








#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;
using namespace std::chrono_literals;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10s

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


#define TCS34725_CDATAL 0x14
#define TCS34725_RDATAL 0x16
#define TCS34725_GDATAL 0x18
#define TCS34725_BDATAL 0x1A
#define COMMAND_ADDRESS 0X80
#define TCS34725_ADDRESS 0x29<<1    // RGB sensor address
#define Si7021_ADDRESS 0x40<<1      // Temperature humidity address
#define MMA8451_I2C_ADDRESS 0x1C<<1



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

/**
 * Default and configured device EUI, application EUI and application key
 */
static const uint8_t DEFAULT_DEV_EUI[] = {0x40, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t DEV_EUI[] = {0x7b, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = {0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0x4d};
static uint8_t APP_KEY[] = {0xf3, 0x1c, 0x2e, 0x8b, 0xc6, 0x71, 0x28, 0x1d,
                            0x51, 0x16, 0xf0, 0x8f, 0xf0, 0xb7, 0x92, 0x8f};

I2C i2c(PB_9, PB_8); 

//GPS
float timeGPS, latitude, longitude, altitude;
char* NS_ind, *EW_ind;
int counter_GPS=0;
BufferedSerial GPS(PA_9, PA_10, 9600);

//brightness
float v_brightness;
AnalogIn Brightness(PA_4);

//temp and humd
float v_temp;
float v_humidity;

//soil
float v_soilMoisture;
DigitalOut switch_soil(PA_8);
AnalogIn soil(PA_0);

//RGB sensor
DigitalOut led(PB_7); //led to light the place to measure
int16_t red;
int16_t green;
int16_t blue;
int16_t clear;

//Accelerometer
float x_acc = 0.0;
float y_acc = 0.0;
float z_acc = 0.0;


//RGB LED
DigitalOut redLED(PB_14);   
DigitalOut greenLED(PB_15);
DigitalOut blueLED(PB_13);



void location();
void parseSentenceGPS(const char* sentence);
float brightness();
void tempAndHum();
void soilMoisture();
void rgb();
void read_colour();
void accelerometer();
static size_t setbuffer();



/**
 * Entry point for application
 */
int main(void)
{
    printf("\r\n*** Sensor Networks @ ETSIST, UPM ***\r\n"
           "   Mbed (v%d.%d.%d) LoRaWAN example\r\n",
           MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    printf("\r\n DEV_EUI: ");
    for (int i = 0; i < sizeof(DEV_EUI); ++i) printf("%02x", DEV_EUI[i]);
    printf("\r\n APP_EUI: ");
    for (int i = 0; i < sizeof(APP_EUI); ++i) printf("%02x", APP_EUI[i]);
    printf("\r\n APP_KEY: ");
    for (int i = 0; i < sizeof(APP_KEY); ++i) printf("%02x", APP_KEY[i]);
    printf("\r\n");

    if (!memcmp(DEV_EUI, DEFAULT_DEV_EUI, sizeof(DEV_EUI))) {
        printf("\r\n *** You are using the default device EUI value!!! *** \r\n");
        printf("Please, change it to ensure that the device EUI is unique \r\n");
        return -1;
    }

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

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;

    location();
    brightness();
    tempAndHum();
    soilMoisture();
    rgb();
    accelerometer();
    
    if(latitude == 0 or longitude == 0){
    
        latitude = 40.389725;
        longitude = -3.628375;




        /*
        latitude = 4023.68;
        longitude = 337.34;

        int latitudeDegrees = (int) latitude / 100;
        double latitudeMinutes = latitude - latitudeDegrees*100;
        latitude = latitudeDegrees + (latitudeMinutes/60);
        

        int longitudeDegrees = (int) longitude / 100;
        double longitudeMinutes = longitude - longitudeDegrees*100;
        longitude = -(longitudeDegrees + (longitudeMinutes/60));
        

        printf("\r\n Latitude %f %s Longitude %f %s  \r\n", latitude, NS_ind, longitude, NS_ind);
        //latitude = 43.348273;
        //longitude = -8.349613;
        */

        /*
        float lat = latitude / 100;
        float latDec = int((lat - int(lat))*100)/60;
        latitude = int(lat) + latDec;

        float lon = longitude / 100;
        float lonDec = int((lon - int(lon))*100)/60;
        longitude = float(int(lon)) + (lonDec);
        */


    }

    if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        //printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
        printf("\r\n Latitude Value = %.2f Longitude Value = %.2f", latitude, longitude);
        printf("\r\n Temperature Value = %.1f Humidity Value = %.1f",v_temp, v_humidity);
        printf("\r\n Soil moisture Value = %.0f", v_soilMoisture);
        printf("\r\n Brightness Value = %.2f", v_brightness);
        printf("\r\n Color: Red = %d Green = %d Blue = %d ", red, green, blue);
        printf("\r\n Acelerometro: X = %.1f Y = %.1f Z = %.1f \r\n", x_acc, y_acc, z_acc);
        ds1820.startConversion();
    } else {
        printf("\r\n No sensor found \r\n");
        return;
    }

   size_t length = setbuffer();
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, length,
                           MSG_UNCONFIRMED_FLAG);


    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3s, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}


static size_t setbuffer(){
    uint8_t vv_brightness = (int) v_brightness;
    uint8_t vv_soil = (int) v_soilMoisture;
    uint8_t vv_humidity = (int) v_humidity;
    int32_t vv_temp = *(uint32_t *) &v_temp; 
    uint16_t vv_red = red; 
    uint16_t vv_green = green;
    uint16_t vv_blue = blue;
    //uint16_t vv_clear = clear; 
    int8_t xx_acc =  (int) (x_acc * 10);
    int8_t yy_acc = (int) (y_acc * 10);
    int8_t zz_acc = (int) (z_acc* 10);
    int32_t vv_latitude = *(uint32_t *) &latitude;
    int32_t vv_longitude = *(uint32_t *) &longitude;

    size_t pos = 0;
    tx_buffer[pos++] = vv_brightness & 0xff;
    tx_buffer[pos++] = vv_soil & 0xff;
    tx_buffer[pos++] = vv_humidity & 0xff;

    tx_buffer[pos++] = vv_temp & 0xff;
    tx_buffer[pos++] = (vv_temp >> 8) & 0xff;
    tx_buffer[pos++] = (vv_temp >> 16) & 0xff;
    tx_buffer[pos++] = (vv_temp >> 24) & 0xff;

    tx_buffer[pos++] = vv_red & 0xff;
    tx_buffer[pos++] = (vv_red >> 8) & 0xff;
    tx_buffer[pos++] = vv_green & 0xff;
    tx_buffer[pos++] = (vv_green >> 8) & 0xff;
    tx_buffer[pos++] = vv_blue & 0xff;
    tx_buffer[pos++] = (vv_blue >> 8) & 0xff;
    //tx_buffer[pos++] = vv_clear & 0xff;
    //tx_buffer[pos++] = (vv_clear >> 8) & 0xff;

    tx_buffer[pos++] = xx_acc & 0xff;
    tx_buffer[pos++] = yy_acc & 0xff;
    tx_buffer[pos++] = zz_acc & 0xff;

    tx_buffer[pos++] = vv_latitude & 0xff;
    tx_buffer[pos++] = (vv_latitude >> 8) & 0xff;
    tx_buffer[pos++] = (vv_latitude >> 16) & 0xff;
    tx_buffer[pos++] = (vv_latitude >> 24) & 0xff;
    tx_buffer[pos++] = vv_longitude & 0xff;
    tx_buffer[pos++] = (vv_longitude >> 8) & 0xff;
    tx_buffer[pos++] = (vv_longitude >> 16) & 0xff;
    tx_buffer[pos++] = (vv_longitude >> 24) & 0xff;

    return pos;

}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        //printf("%02x ", rx_buffer[i]);
        printf("%d - %d \n", rx_buffer[i], i);
    }
    printf("\r\n");

    if((rx_buffer[0] ==  82) && (rx_buffer[1] == 101) && (rx_buffer[2] == 100)){
        //82 = 01010010 = R
        //101 = 01100101 = e
        //100 = 01100100 = d
        redLED=1;
        greenLED=0;
        blueLED=0;
        printf("RED TURNED RED");
    }else if((rx_buffer[0] == 71) && (rx_buffer[1] == 114) && (rx_buffer[2] == 101) && (rx_buffer[3] == 101) && (rx_buffer[4] == 110)){
        //Green
    
        redLED=0;
        greenLED=1;
        blueLED=0;
        printf("RED TURNED GREEN");
    } else if((rx_buffer[0] == 79) && (rx_buffer[1] == 70) && (rx_buffer[2] == 70)){
        //OFF

        redLED=0;
        greenLED=0;
        blueLED=0;
        printf("RED TURNED OFF");
    }

    
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



void parseSentenceGPS(const char* sentence) {
    if (strncmp(sentence, "$GPGGA,", 7) == 0) {
        //Solo si empieza por "$GPGGA,"
        counter_GPS++; 
        if (counter_GPS < 2){ //solo obtenemos la primera que llega
            
            char* divisions[15];
            int divisionCount = 0;
            char *wordStart = (char*)sentence; // puntero a donde esta la frase
            
            int i =0;
            while(true){ // para dividir la frase en cada uno de los elementos
                i++;
                char currentChar = sentence[i]; //primer caracter

                //hasta que se llega a la coma o fin de string
                if (currentChar == ',' || currentChar == '\0'){
                    int length = &sentence[i] - wordStart;          // tamaño de la palabra restando direcciones de memoria
                    
                    // If the segment is non-empty, store it in divisions array
                    if(length > 0){
                        divisions[divisionCount] = (char*)malloc(length + 1);   
                        strncpy(divisions[divisionCount], wordStart, length);   // Se copia la palabra
                        divisions[divisionCount][length] = '\0';                
                        divisionCount++;

                        wordStart = (char*)sentence + i + 1; // se mueve al siguiente caracter

                    }else{
                        //si entre comas no hay caracter se almacena nulo
                        divisions[divisionCount] = NULL;
                        wordStart = (char*)sentence + i + 1;
                        divisionCount ++;
                    }
                    

                }

                if(currentChar == '\0'){
                    break;
                }
            }


            
            
            latitude = strtof(divisions[2], nullptr);
            
            longitude = strtof(divisions[4], nullptr);
            NS_ind = divisions[3];
            EW_ind = divisions[5];

            if((latitude != 0) || (longitude != 0)){
                 int latitudeDegrees = (int) latitude / 100;
                double latitudeMinutes = latitude - latitudeDegrees*100;
                latitude = latitudeDegrees + (latitudeMinutes/60);
                

                int longitudeDegrees = (int) longitude / 100;
                double longitudeMinutes = longitude - longitudeDegrees*100;
                longitude = -(longitudeDegrees + (longitudeMinutes/60));
            }
            
            
            for (int j = 0; j < divisionCount; j++) {     
                free(divisions[j]); // liberar la memoria
            }
        }

        if (counter_GPS == 2){
            counter_GPS = 0;
        }
    }
}

void location() {
    char sentence[200];
    int bytesRead = 0;  // To track the number of bytes stored in the sentence
    
    while (GPS.readable()) {
        char c;
        if (GPS.read(&c, 1)) {                  // Se lee un byte (un caracter)
            if (c == '\n') {                        
                sentence[bytesRead] = '\0';     
                parseSentenceGPS(sentence);  
                bytesRead = 0;                 
            } else {
                sentence[bytesRead] = c;        // se almacena en el array
                bytesRead++;                    
                if (bytesRead >= sizeof(sentence) - 1) {
                    // Buffer overflow --> Discard the sentence
                    bytesRead = 0;
                }
            }
        }
    }
}


float brightness(){
    // Analog sensor
    v_brightness = Brightness.read() * 100 * 1.20;
    if(v_brightness>100){
        v_brightness = 100;
    }
    
    return v_brightness;
}


void tempAndHum(){
    // I2C sensor
    char command[1];
    char data[2];
    // HUMIDITY
    command[0] = {0xE5};                        //for read humidity
    i2c.write(Si7021_ADDRESS, command, 1);
    i2c.read(Si7021_ADDRESS, data, 2);               // we save the result in two bytes (L and H)

    uint16_t h = (data[0] << 8) | data[1];      // MSB at [0]
    v_humidity = h * 125.0 / 65536 - 6.0;       // formula in datasheet
    // TEMPERATURE
    command[0] = {0xE3};
    i2c.write(Si7021_ADDRESS, command, 1);           // Send command for reading
    ThisThread::sleep_for(100ms);               // to perform the measurement                
    i2c.read(Si7021_ADDRESS, data, 2);               // we save the result in two bytes (L and H)
                
    uint16_t temperature = (data[0]) << 8 | data[1];    // MSB at [0]
    v_temp = ((175.72 * temperature) / 65536) - 46.85;  // formula in datasheet


}

void soilMoisture(){
    // Analog sensor
    switch_soil = 1;
    v_soilMoisture = soil.read() * 100;

    switch_soil = 0;
}


uint16_t read16(uint8_t reg) {
    char color[2];
    char cmd[1];
    cmd[0] = reg;
    i2c.write(TCS34725_ADDRESS, cmd, 1, true);
    i2c.read(TCS34725_ADDRESS, color, 2);
    return (color[1] << 8) | color[0];
}

void rgb(){

    led=1;
    
    char data[2];
    data[0] = COMMAND_ADDRESS;
    data[1] = 0x03;
    i2c.write(TCS34725_ADDRESS, data, 2);

    wait_us(3000);
    
    clear = read16(TCS34725_CDATAL|COMMAND_ADDRESS);
    red = read16(TCS34725_RDATAL|COMMAND_ADDRESS);
    green = read16(TCS34725_GDATAL|COMMAND_ADDRESS);
    blue = read16(TCS34725_BDATAL|COMMAND_ADDRESS);

}


void accelerometer() {
    char reg[2];
    reg[0] = 0x2A; // Address of the control register 1
    reg[1] = 0x03; // Set active mode and fast mode
    i2c.write(MMA8451_I2C_ADDRESS, reg, 2);

    reg[0] = 0x09; // Address of FIFO setup register
    reg[1] = 0x40; // set FIFO as  circular buffer
    i2c.write(MMA8451_I2C_ADDRESS, reg, 2);

    // Read the X, Y, Z data
    char data[4]; // Changed the size to 6 as there are 6 bytes of data (2 bytes for each axis)
    reg[0] = 0x01; // Address of the X_MSB register
    i2c.write(MMA8451_I2C_ADDRESS, reg, 1);
    i2c.read(MMA8451_I2C_ADDRESS, data, 4); // Read 6 bytes of data (2 bytes for each axis)

    // Combine the high and low bytes for each axis
    int8_t x = (data[1]);
    int8_t y = (data[2]);
    int8_t z = (data[3]);

    int8_t x_flip=0;
    int8_t y_flip=0;
    int8_t z_flip=0;

    
    //get number from c'2 to sigend
    if (x > 127){
        x_flip = ~x + 0x01;
        }
    else {
        x_flip = x;
    }

    if (y > 127){
        y_flip = ~y + 0x01;
        }
    else {
        y_flip = y;
    }

    if (z > 127){
        z_flip = ~z + 0x01;
        }
    else {
        z_flip = z;
    }

    // al ser 8 bits se divide entre 4096/64 para obetner valores en 1g
    x_acc = (float)x_flip / 64; 
    y_acc = (float)y_flip / 64;
    z_acc = (float)z_flip / 64;

    //Obtener  en m/s
    x_acc = 9.81 * x_acc;
    y_acc = 9.81 * y_acc;
    z_acc = 9.81 * z_acc;


}



