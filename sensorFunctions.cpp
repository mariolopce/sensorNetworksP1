#include "mbed.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include <string>

#define MAX_LIGHT 70
#define MIN_LIGHT 0

#define MAX_SOIL 100
#define MIN_SOIL 0

#define MAX_TEMPERATURE 50
#define MIN_TEMPERATURE -10

#define MAX_REL_HUMIDITY 85
#define MIN_REL_HUMIDITY 25

#define MAX_RED_COLOR 1
#define MAX_BLUE_COLOR 1

#define MAX_X_VALUE 2
#define MAX_Y_VALUE 2
#define MAX_Z_VALUE 12
#define MIN_X_VALUE -2
#define MIN_Y_VALUE -2
#define MIN_Z_VALUE 7

#define TCS34725_CDATAL 0x14
#define TCS34725_RDATAL 0x16
#define TCS34725_GDATAL 0x18
#define TCS34725_BDATAL 0x1A
#define COMMAND_ADDRESS 0X80
#define TCS34725_ADDRESS 0x29<<1    // RGB sensor address
#define Si7021_ADDRESS 0x40<<1      // Temperature humidity address
#define MMA8451_I2C_ADDRESS 0x1C<<1

extern DigitalOut redLED;   
extern DigitalOut greenLED;
extern DigitalOut blueLED;


extern uint16_t clear; 
extern uint16_t red; 
extern uint16_t green; 
extern uint16_t blue; 
int counter_red=0;
int counter_green=0;
int counter_blue=0;
 
float v_brightness;
float mean_brightness = 0;
float maximum_brightness = 0;
float minimum_brightness = 1000;
int counter_brightness = 0; 

float v_temp;
float mean_temperature = 0.0;
float maximum_temperature = 0.0;
float minimum_temperature = 1000.0;
int counter_temperature = 0;

float v_humidity;
float mean_humidity = 0.0;
float maximum_humidity = 0.0;
float minimum_humidity = 1000.0;

float v_soilMoisture;
float mean_soil = 0.0;
float maximum_soil = 0.0;
float minimum_soil = 1000.0;
int counter_soil = 0;

float x_acc = 0.0;
float y_acc = 0.0;
float z_acc = 0.0;
float maximum_x = -1000.0;
float minimum_x = 1000.0;
float maximum_y = -1000.0;
float minimum_y = 1000.0;
float maximum_z = -1000.0;
float minimum_z = 1000.0;


float timeGPS, latitude,  longitude, altitude;
char* NS_ind, *EW_ind, *altitude_unit;
int satellites, totalSeconds, hours, minutes, seconds;

Timer echoTimer;
DigitalOut trig(PA_13);
DigitalIn echo(PA_14);
float duration_sound;
extern float distance_sound;
extern Mutex distanceMutex;


extern int mode; 
extern int mode_ant;

I2C i2c(PB_9, PB_8); 
DigitalOut led(PB_7);  
DigitalOut switch_soil(PA_8);
BufferedSerial GPS(PA_9, PA_10, 9600);

AnalogIn soil(PA_0);
AnalogIn Brightness(PA_4);


void brightness();
void soilMoisture();
void tempAndHum();
void rgb();
void read_colour();
void location();
void parseSentenceGPS();
void accelerometer();
void reset_variables();
void print_result();
void print_stats();
void sound();
void print_sound();
void print_mode();

int counter_GPS=0;
int counter_hour=0;

bool flag_30 = false;
bool repeat_30 = false;
Timeout to;

void count_30(){
    flag_30 = true;
    repeat_30 = false;
}


void measure(){
    trig = 0;
    //GPS.set_baud(9600);
    while(1){
        led=1;
        
        if (mode == 0){
            // If TEST MODE --> Measures are taken each 2 seconds
            print_mode();
            soilMoisture();
            brightness();
            tempAndHum();
            rgb();
            location();
            accelerometer();
            print_result();
            //led = 0;
            ThisThread::sleep_for(2000);
        }

        /*
        else if (mode == 1) {
        // If NORMAL MODE --> We check the limits, we show the maximum, minimum and average
            //                    and measures are taken each 30 seconds (now each 5s)
            if (repeat_30 == false){ //set again the function 
                to.attach_us(count_30, 30000000); 
                repeat_30 = true;
            }
            if (flag_30 == true){
                print_mode();
                soilMoisture();
                brightness();
                tempAndHum();
                rgb();
                location();
                accelerometer();
                print_result();
                
                if (v_brightness >= MAX_LIGHT || v_brightness <= MIN_LIGHT){
                    // YELLOW COLOUR
                    redLED = 1;
                    greenLED = 1;
                    blueLED = 0;
                    printf("ALARM: Brightness exceeding limits: %f %%\n", v_brightness);
                }
                else if (v_soilMoisture >= MAX_SOIL || v_soilMoisture <= MIN_SOIL){
                    // CYAN COLOUR
                    redLED = 0;
                    greenLED = 1;
                    blueLED = 1;
                    printf("ALARM: Soil Moisture exceeding limits: %f %%\n", v_soilMoisture);
                }
                else if (v_temp >= MAX_TEMPERATURE || v_temp <= MIN_TEMPERATURE) {
                    // PURPLE COLOUR
                    redLED = 1;
                    greenLED = 0;
                    blueLED = 1;
                    printf("ALARM: Temperature exceeding limits: %f ºC\n", v_temp);
                }
                else if (v_humidity >= MAX_REL_HUMIDITY || v_humidity <= MIN_REL_HUMIDITY) {
                    // WHITE COLOUR
                    redLED = 1;
                    greenLED = 1;
                    blueLED = 1;
                    printf("ALARM: Relative humidity exceeding limits: %f %%\n", v_humidity);
                }
                else if ((float)(green/red) < 1.0) {
                    // RED
                    redLED = 1;
                    greenLED = 0;
                    blueLED = 0;
                    printf("ALARM: Red colour exceeding limits\n");
                
                }
                else if ((float)(green/blue) < 1.0){
                    // BLUE
                    redLED = 0;
                    greenLED = 0;
                    blueLED = 1;
                    printf("ALARM: Blue colour exceeding limits\n");

                }
                else if (x_acc > MAX_X_VALUE || x_acc < MIN_X_VALUE || y_acc > MAX_Y_VALUE || y_acc < MIN_Y_VALUE || z_acc > MAX_Z_VALUE || z_acc < MIN_Z_VALUE){
                    // BLUE
                    redLED = 0;
                    greenLED = 1;
                    blueLED = 0;
                    printf("ALARM: Accelerator exceeding limits: x: %f m/s2, y: %f m/s2, z: %f m/s2\n", x_acc, y_acc, z_acc);

                }
                else {
                    // RGB LED switched off
                    redLED = 0;
                    greenLED = 0;
                    blueLED = 0; 
                }
                
                counter_hour++;
                if (counter_hour == 120){ 
                    // Stats shown each 30 seconds * 120 --> 1 hour
                    mean_soil = mean_soil/counter_soil;
                    mean_brightness = mean_brightness/counter_brightness;
                    mean_temperature = mean_temperature/counter_temperature;
                    mean_humidity = mean_humidity/counter_temperature;

                    print_stats();
                    
                    counter_hour = 0;
                    reset_variables();
                    

                    
                }
                flag_30 = false;
            }
            
        }
        else if (mode == 2) {
        // If ADVANCED MODE --> We check the limits, we show the maximum, minimum and average
            //                    and measures are taken each 30 seconds + telemtric
            if (repeat_30 == false){
                to.attach_us(count_30, 30000000);
                repeat_30 = true;
            }
            if (flag_30 == true){
                soilMoisture();
                brightness();
                tempAndHum();
                rgb();
                location();
                accelerometer();
                sound();
                print_mode();
                print_result();
                print_sound();
                
                if (v_brightness >= MAX_LIGHT || v_brightness <= MIN_LIGHT){
                    // YELLOW COLOUR
                    redLED = 1;
                    greenLED = 1;
                    blueLED = 0;
                    printf("ALARM: Brightness exceeding limits: %.1f %%\n", v_brightness);
                }
                else if (v_soilMoisture >= MAX_SOIL || v_soilMoisture <= MIN_SOIL){
                    // CYAN COLOUR
                    redLED = 0;
                    greenLED = 1;
                    blueLED = 1;
                    printf("ALARM: Soil Moisture exceeding limits: %.1f %%\n", v_soilMoisture);

                }
                else if (v_temp >= MAX_TEMPERATURE || v_temp <= MIN_TEMPERATURE) {
                    // PURPLE COLOUR
                    redLED = 1;
                    greenLED = 0;
                    blueLED = 1;
                    printf("ALARM: Temperature exceeding limits: %.1f ºC\n", v_temp);
                }
                else if (v_humidity >= MAX_REL_HUMIDITY || v_humidity <= MIN_REL_HUMIDITY) {
                    // WHITE COLOUR
                    redLED = 1;
                    greenLED = 1;
                    blueLED = 1;
                    printf("ALARM: Relative humidity exceeding limits: %.1f %%\n", v_humidity);
                }
                else if ((float)(green/red) < 1.0) {
                    // RED
                    redLED = 1;
                    greenLED = 0;
                    blueLED = 0;
                    printf("ALARM: Red colour exceeding limits\n");
                
                }
                else if ((float)(green/blue) < 1.0){
                    // BLUE
                    redLED = 0;
                    greenLED = 0;
                    blueLED = 1;
                    printf("ALARM: Blue colour exceeding limits\n");

                }
                else if (x_acc > MAX_X_VALUE || x_acc < MIN_X_VALUE || y_acc > MAX_Y_VALUE || y_acc < MIN_Y_VALUE || z_acc > MAX_Z_VALUE || z_acc < MIN_Z_VALUE){
                    // BLUE
                    redLED = 0;
                    greenLED = 1;
                    blueLED = 0;
                    printf("ALARM: Accelerator exceeding limits: x: %f m/s2, y: %f m/s2, z: %f m/s2\n", x_acc, y_acc, z_acc);
                    

                }
                else {
                    // RGB LED switched off
                    redLED = 0;
                    greenLED = 0;
                    blueLED = 0; 
                }
                
                counter_hour++;
                if (counter_hour == 120){
                    // Stats shown each 30 seconds * 120 --> 1 hour
                    mean_soil = mean_soil/counter_soil;
                    mean_brightness = mean_brightness/counter_brightness;
                    mean_temperature = mean_temperature/counter_temperature;
                    mean_humidity = mean_humidity/counter_temperature;

                    print_stats();
                    
                    counter_hour = 0;
                    reset_variables();
                    

                    
                }
                flag_30 = false;
            }
            
        }
        */
    }
}

void print_result()
{
    printf("SOIL MOISTURE: %.1f %%\n", v_soilMoisture);
    printf("LIGHT: %.1f %%\n", v_brightness);
    printf("TEMP/HUM: Temperature: %.1f ºC,     Relative humidity: %.1f %%\n", v_temp, v_humidity);
    printf("COLOR SENSOR: Clear: %d  Red: %d  Green: %d  Blue: %d\n", clear, red, green, blue);
    printf("GPS: #sats: %d  Lat(UTC): %f %s  Long(UTC): %f %s  Altitude: %f %s  %02d:%02d:%02d\n",satellites, latitude, NS_ind, longitude, EW_ind, altitude, altitude_unit, hours, minutes, seconds);
    printf("ACCELOREMETERS: X_axis: %f m/s2, Y_axis: %f m/s2, Z_axis: %f m/s2\n", x_acc, y_acc, z_acc);
}

void print_stats()
{
    printf("\n");
    printf("RESULTS \n");
    printf("SOIL MOISTURE: Maximum: %f  Minimum: %f  Mean: %f\n", maximum_soil, minimum_soil, mean_soil);
    printf("LIGHT: Maximum: %f  Minimum: %f  Mean: %f\n", maximum_brightness, minimum_brightness, mean_brightness);
    printf("TEMPERATURE: Maximum: %f  Minimum: %f  Mean: %f\n", maximum_temperature, minimum_temperature, mean_temperature);
    printf("HUMIDITY: Maximum: %f  Minimum: %f  Mean: %f\n", maximum_humidity, minimum_humidity, mean_humidity);
    printf("ACCELEROMETERS: Maximum_x: %f m/s2  Minimum_x: %f m/s2\n", maximum_x, minimum_x);
    printf("                Maximum_y: %f m/s2  Minimum_y: %f m/s2\n", maximum_y, minimum_y);
    printf("                Maximum_z: %f m/s2  Minimum_z: %f m/s2\n", maximum_z, minimum_z);

    if ((max(counter_red,max(counter_green,counter_blue))==counter_red)){
        printf("RGB SENSOR: Dominant: RED, appeared %d times\n", counter_red);
    }
    else if ((max(counter_red,max(counter_green,counter_blue))==counter_green)) {
        printf("RGB SENSOR: Dominant: GREEN, appearred %d times\n", counter_green);
    }
    else if ((max(counter_red,max(counter_green,counter_blue))==counter_blue)) {
        printf("RGB SENSOR: Dominant: BLUE, appeared %d times\n", counter_blue);
    }
    printf("\n");
}

void print_sound(){
    printf("Distance: %.2f cm\n", distance_sound); // Print the distance in centimeters
}

void print_mode(){
    printf("\n");
    switch (mode) {
    case 0:
        printf("TEST MODE\n");
        break;
    case 1:
        printf("NORMAL MODE\n");
        break;
    case 2:
        printf("ADVANCED MODE \n");
        break;
    default:
        break;
    }
}

void reset_variables(){
    // Function used to reset the stats variables for new computation each hour
    mean_brightness = 0;
    maximum_brightness = 0;
    minimum_brightness = 1000;
    mean_temperature = 0.0;
    maximum_temperature = 0.0;
    minimum_temperature = 1000.0;
    mean_humidity = 0;
    maximum_humidity = 0;
    minimum_humidity = 1000.0;
    mean_soil = 0;
    maximum_soil = 0;
    minimum_soil = 1000.0;
    counter_red = 0;
    counter_green = 0;
    counter_blue = 0;
    counter_brightness = 0;
    counter_temperature = 0;
    counter_soil = 0;
    maximum_x = -1000.0;
    maximum_y = -1000.0;
    maximum_z = -1000.0;
    minimum_x = 1000.0;
    minimum_y = 1000.0;
    minimum_z = 1000.0;



}

void soilMoisture(){
    // Analog sensor
    switch_soil = 1;
    v_soilMoisture = soil.read() * 100;

    if (mode > 0) {
        // Only if mode = NORMAL we update maximum, minimum and average values
        if (mode != mode_ant){
            mean_soil = 0;
            maximum_soil = 0;
            minimum_soil = 1000;
            mode_ant = mode;
         }
        counter_soil++;
        maximum_soil = max(maximum_soil,v_soilMoisture);
        minimum_soil = min(minimum_soil,v_soilMoisture);
        mean_soil = (mean_soil + v_soilMoisture);
        
    }
    switch_soil = 0;
}

void brightness(){
    // Analog sensor
    v_brightness = Brightness.read() * 100 * 1.20;
    if(v_brightness>100){
        v_brightness = 100;
    }
    
    if (mode > 0) {
        // Only if mode = NORMAL we update maximum, minimum and average values
        if (mode != mode_ant){
            mean_brightness = 0;
            maximum_brightness = 0;
            minimum_brightness = 1000;
            mode_ant = mode;
         }
        counter_brightness++;
        maximum_brightness = max(maximum_brightness,v_brightness);
        minimum_brightness = min(minimum_brightness,v_brightness);
        mean_brightness = (mean_brightness + v_brightness);
    }
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
    


    if (mode > 0) {
        // Only if mode = NORMAL we update maximum, minimum and average values
        if (mode != mode_ant){
            mean_temperature = 0.0;
            maximum_temperature = 0.0;
            minimum_temperature = 1000.0;
            maximum_humidity = 0;
            minimum_humidity = 1000.0;
            mean_humidity = 0;
            mode_ant = mode;
         }
        counter_temperature++;
        maximum_temperature = max(maximum_temperature,v_temp);
        minimum_temperature = min(minimum_temperature,v_temp);
        mean_temperature = (mean_temperature + v_temp);

        maximum_humidity = max(maximum_humidity,v_humidity);
        minimum_humidity = min(minimum_humidity,v_humidity);
        mean_humidity = (mean_humidity + v_humidity);

    }

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

    //led=0;

     if (mode > 0) {
        if (mode != mode_ant){
            counter_red = 0;
            counter_green = 0;
            counter_blue = 0;
            mode_ant = mode;
         }
        
        if (max(red,max(blue,green))==red){
            counter_red++;
        }
        else if (max(red,max(blue,green))==green) {
            counter_green++;
        }
        else if (max(red,max(blue,green))==blue) {
            counter_blue++;
        }
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


            
            timeGPS = strtof(divisions[1], nullptr);
            latitude = strtof(divisions[2], nullptr);
            NS_ind = divisions[3];
            longitude = strtof(divisions[4], nullptr);
            EW_ind = divisions[5];
            satellites = strtof(divisions[7], nullptr);
            altitude = strtof(divisions[9], nullptr);
            altitude_unit = divisions[10];
            totalSeconds = static_cast<int>(timeGPS);      // Convert to integer (truncate decimal)
            hours = (totalSeconds / 10000)+1;           // Get the hours part (hhmm.ss -> hh)
            minutes = (totalSeconds % 10000) / 100;     // Get the minutes part (hhmm.ss -> mm)
            seconds = totalSeconds % 100;               // Get the remaining seconds (hhmm.ss -> ss)
                
           
            
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


    if (mode > 0) {
        // Only if mode = NORMAL or ADVANCE  we update maximum, minimum and average values
        if (mode != mode_ant){
            maximum_x = -1000.0;
            minimum_x = 1000.0;
            maximum_y = -1000.0;
            minimum_y = 1000.0;
            maximum_z = -1000.0;
            minimum_z = 1000.0;
            mode_ant = mode;
         }
        maximum_x = max(maximum_x,x_acc);
        minimum_x = min(minimum_x,x_acc);

        maximum_y = max(maximum_y,y_acc);
        minimum_y = min(minimum_y,y_acc);

        maximum_z = max(maximum_z,z_acc);
        minimum_z = min(minimum_z,z_acc);

    }


}

void sound(){
    echoTimer.reset();
    trig = 1;
    wait_us(10); //pulso de 10us
    trig = 0;
    while (!echo){}
    echoTimer.start();
    while (echo){}
    echoTimer.stop();

    distanceMutex.lock();
    duration_sound = static_cast<float>(echoTimer.read());
    distance_sound = duration_sound * 17000.0;
    distanceMutex.unlock(); 

    //v=d/t -> 340m/s -> velocidad del sonido -> *100 to get cm -> entre dos por que la distancia es ida y vuelta
}
