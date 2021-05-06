#include "mbed.h"
#include "uop_msb_2_0_0.h"
#include <iostream>
#include "BMP280_SPI.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

using namespace uop_msb_200;
using namespace std;

// C Function Prototypes
extern int write_sdcard();
extern int read_sdcard(); 
extern void matrix_init(void);
extern void matrix_scan(void);
extern void clearMatrix(void);

//Digital Inputs (you could use DigitalIn or InterruptIn instead)
Buttons btns;

//Analogue Inputs
AnalogIn ldr(AN_LDR_PIN);

//LED Outputs
LatchedLED ledStrip(LatchedLED::STRIP);
LatchedLED ledDigit(LatchedLED::SEVEN_SEG);

//Buzzer
Buzzer buzz;

//LCD Display
LCD_16X2_DISPLAY lcd_disp;

// Functions //
void debugStuff();
void sampleEnvironment();
int write_sd();
int read_sd();

// Threads //
// You should use at least 4 separate and dedicated threads for:
//  - (i) sampling the data, 
//  - (ii) writing to the SD card; 
//  - (iii) communicating with the serial interface 
//  - (iv) communicating with the network. 
//  - Event Queues are recommended but not a requirement. In addition, you may also use interrupts if appropriate and where justified, again with suitable synchronization.
Thread tSample, tSDWrite, tSerialComm, tNetComm;
Thread tDebug;

// Structs //
struct SensorData
{
    // TODO: Data types probably incorrect
    float temperature;
    float pressure;
    float light_level;
};
SensorData sensorData;

BMP280_SPI bmp280(PB_5, PB_4, PB_3, PB_2);

// SD Card
static SDBlockDevice sd_mine(PB_5, PB_4, PB_3, PF_3); // SD Card Block Device
static InterruptIn sd_inserted(PF_4); // Interrupt for card insertion events

/* Requirements *
 - Active: 1, 2, 3, 4, 8, 9, 12, 13
 - Passive: 5, 6, 7, 10, 11
*/

// Remember that main() runs in its own thread in the OS
int main()
{
   
    /* START REQUIREMENT 1 - Environmental Sensor */
    // This device shall periodically measure sensor data at a fixed and deterministic rate. 
    // This shall include temperature (deg C), pressure (mbar) and light levels (from the LDR). 
    // The default update rate shall be once every second and you should write your code to sampling minimize jitter. 
    // The data shall be encapsulated in a single C++ structure or class

    //Environmental sensor
    bmp280.initialize();

    //tDebug.start(debugStuff);
    tSample.start(sampleEnvironment);

    /* END Requirement 1 */

    /* START Requirement 2 - SD Card Writing */
    
    write_sd();
    read_sd();

    /* END Requirement 2 - SD Card Writing */
}

int write_sd() {
  printf("Initialise and write to a file\n");

  // call the SDBlockDevice instance initialisation method.
  if (0 != sd_mine.init()) {
    printf("Init failed \n");
    return -1;
  }

  FATFileSystem fs("sd", &sd_mine);
  FILE *fp = fopen("/sd/test.txt", "w+");
  if (fp == NULL) {
    error("Could not open file for write\n");
    sd_mine.deinit();
    return -1;
  } else {
    // Put some text in the file...
    fprintf(fp, "Jordan says, \"Hi!\"\n");
    // Tidy up here
    fclose(fp);
    printf("SD Write done...\n");
    sd_mine.deinit();
    return 0;
  }  
}

int read_sd()
{
    printf("Initialise and read from a file\n");

  // call the SDBlockDevice instance initialisation method.
  if (0 != sd_mine.init()) {
    printf("Init failed \n");
    return -1;
  }

  FATFileSystem fs("sd", &sd_mine);
  FILE *fp = fopen("/sd/test.txt", "r");
  if (fp == NULL) {
    error("Could not open or find file for read\n");
    sd_mine.deinit();
    return -1;
  } else {
    // Put some text in the file...
    char buff[64];
    buff[63] = 0;
    while (!feof(fp)) {
      fgets(buff, 63, fp);
      printf("%s\n", buff);
    }
    // Tidy up here
    fclose(fp);
    printf("SD Write done...\n");
    sd_mine.deinit();
    return 0;
  }
}

void debugStuff()
{
    // Sensor data
    while(true)
        printf("Temperature: %f || Pressure: %f || Light: %f\n", sensorData.temperature, sensorData.pressure, sensorData.light_level);
}

void sampleEnvironment()
{
    while(true)
    {
        // Collect sample data
        sensorData.light_level = ldr;
        sensorData.temperature = bmp280.getTemperature();
        sensorData.pressure = bmp280.getPressure();

        // Wait a second (minus however long it took? Timer would probably cause overhead, though)
        ThisThread::sleep_for(1000ms);
    }
}


