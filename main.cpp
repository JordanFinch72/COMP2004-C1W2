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

<<<<<<< HEAD
// Switches // TODO: This seems to be an alternative to btns; delete after if not used
DigitalIn btnA(PG_0);

=======
>>>>>>> parent of d302256 (Experimental set datetime code)
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

<<<<<<< HEAD
void change_part()
{
    dt_part = (dt_part == 6) ? 0 : dt_part + 1; // Increment until 6, then wrap to 0
}
void display_datetime()
{
    ++second;
}

=======
>>>>>>> parent of d302256 (Experimental set datetime code)
// Remember that main() runs in its own thread in the OS
int main()
{
   
    /* START REQUIREMENT 1 - Environmental Sensor */
    // This device shall periodically measure sensor data at a fixed and deterministic rate. 
    // This shall include temperature (deg C), pressure (mbar) and light levels (from the LDR). 
    // The default update rate shall be once every second and you should write your code to sampling minimize jitter. 
    // The data shall be encapsulated in a single C++ structure or class
    printf("Heleeeeeeeelo?");
    //Environmental sensor
    bmp280.initialize();

    //tDebug.start(debugStuff);
    //tSample.start(sampleEnvironment);

    /* END Requirement 1 */

    printf("Hellofeffefef?");
    /* START Requirement 2 - SD Card Writing */
    
    write_sd();
    read_sd();

    /* END Requirement 2 - SD Card Writing */
<<<<<<< HEAD

    /* START Requirement 4 - Set Date/Time */
    
    // I think that the main thread should always be looking out for changes to the switches/potentiometer
    // If not, can always run it on another thread

    printf("Hello?");

    ticker.attach(&display_datetime, 1000ms);
    while(true)
    {
        sleep();

        lcd_disp.printf("%02d/%02d/%04d @ %02d:%02d:%02d", day, month, year, hour, minute, second);

        if(btnA != 0) // User is required to hold the button for up to a second
            change_part();

        while(dt_part == 1) // DAY
        {
            // Handle potentiometer turns. Function would cause a lot of overhead
            unsigned short pot_val = pot.read_u16();            
            unsigned short delta = pot_old - pot_val;            
            bool stable = !(pot_val - POT_TOLERANCE > pot_old  || pot_val + POT_TOLERANCE < pot_old); // Tolerance to prevent noise from causing change
            int up = (!stable && pot_val > POT_HALFWAY);  // If turned over halfway, it's going up. Otherwise, it's going down

            // Store current values
            pot_old = pot_val;

            // Convert to operand values
            int pot_mod = (stable) ? 0 : ((up) ? 1 : -1);

            // TODO: It's times like this that I used some sort of class or struct to handle this crap - Nicholas would probably appreciate it and it would look good, right?
            //  - How important is space/time efficiency vs. code neatness and good practice, here? I need to know. Don't delete this TODO until I've found out.
            if(pot_mod == 1)
            {
                if(day == 31 || (day == 30 && (month == 4 || month == 6 || month == 9 || month == 11)) || (day == 28 && month == 2))
                    day = 1;
                else
                    ++day;
            }
            else if(pot_mod == -1)
            {
                if(day == 1)
                {
                    if(month == 2)
                        day = 28;
                    else if(month == 4 || month == 6 || month == 9 || month == 11)
                        day = 30;
                    else
                        day = 31;
                }
            }
        }        

        while(dt_part == 2) // MONTH
        {
            // Handle potentiometer turns
            unsigned short pot_val = pot.read_u16();            
            unsigned short delta = pot_old - pot_val;            
            bool stable = !(pot_val - POT_TOLERANCE > pot_old  || pot_val + POT_TOLERANCE < pot_old); // Tolerance to prevent noise from causing change
            int up = (!stable && pot_val > POT_HALFWAY);  // If turned over halfway, it's going up. Otherwise, it's going down

            // Store current values
            pot_old = pot_val;

            // Convert to operand values
            int pot_mod = (stable) ? 0 : ((up) ? 1 : -1);

            if(pot_mod == 1) month = (month == 12) ? 1 : month + 1;
            if(pot_mod == -1) month = (month == 1) ? 12 : month - 1;
        }

        while(dt_part == 3) // YEAR
        {
            // Handle potentiometer turns
            unsigned short pot_val = pot.read_u16();            
            unsigned short delta = pot_old - pot_val;            
            bool stable = !(pot_val - POT_TOLERANCE > pot_old  || pot_val + POT_TOLERANCE < pot_old); // Tolerance to prevent noise from causing change
            int up = (!stable && pot_val > POT_HALFWAY);  // If turned over halfway, it's going up. Otherwise, it's going down

            // Store current values
            pot_old = pot_val;

            // Convert to operand values
            int pot_mod = (stable) ? 0 : ((up) ? 1 : -1);

            year += pot_mod;
        }

        while(dt_part == 4) // HOUR
        {
            // Handle potentiometer turns
            unsigned short pot_val = pot.read_u16();            
            unsigned short delta = pot_old - pot_val;            
            bool stable = !(pot_val - POT_TOLERANCE > pot_old  || pot_val + POT_TOLERANCE < pot_old); // Tolerance to prevent noise from causing change
            int up = (!stable && pot_val > POT_HALFWAY);  // If turned over halfway, it's going up. Otherwise, it's going down

            // Store current values
            pot_old = pot_val;

            // Convert to operand values
            int pot_mod = (stable) ? 0 : ((up) ? 1 : -1);

            if(pot_mod == 1) hour = (hour == 23) ? 0 : hour + 1;
            if(pot_mod == -1) hour = (hour == 0) ? 23 : hour - 1;
        }

        while(dt_part == 5) // MINUTE
        {
            // Handle potentiometer turns
            unsigned short pot_val = pot.read_u16();            
            unsigned short delta = pot_old - pot_val;            
            bool stable = !(pot_val - POT_TOLERANCE > pot_old  || pot_val + POT_TOLERANCE < pot_old); // Tolerance to prevent noise from causing change
            int up = (!stable && pot_val > POT_HALFWAY);  // If turned over halfway, it's going up. Otherwise, it's going down

            // Store current values
            pot_old = pot_val;

            // Convert to operand values
            int pot_mod = (stable) ? 0 : ((up) ? 1 : -1);

            if(pot_mod == 1) minute = (minute == 59) ? 0 : minute + 1;
            if(pot_mod == -1) minute = (minute == 0) ? 59 : minute - 1;
        }

        while(dt_part == 6) // SECOND
        {
            // Handle potentiometer turns
            unsigned short pot_val = pot.read_u16();            
            unsigned short delta = pot_old - pot_val;            
            bool stable = !(pot_val - POT_TOLERANCE > pot_old  || pot_val + POT_TOLERANCE < pot_old); // Tolerance to prevent noise from causing change
            int up = (!stable && pot_val > POT_HALFWAY);  // If turned over halfway, it's going up. Otherwise, it's going down

            // Store current values
            pot_old = pot_val;

            // Convert to operand values
            int pot_mod = (stable) ? 0 : ((up) ? 1 : -1);

            if(pot_mod == 1) second = (second == 59) ? 0 : second + 1;
            if(pot_mod == -1) second = (second == 0) ? 59 : second - 1;
        }
        
        pot_old = 0; // TODO: This must happen after each button press to rest it. Test if it does (I'm not sure if a full loop is done with each button press or not. I think maybe not)

        // This entire thing would better be written with interrupts:
        // Constantly display date/time
        // When interrupt fires, modify variable so date/time flickers the currently-selected part (e.g. refresh = 0 and part = "", then refresh = 500 & part = "hour")
        // Loop a thread that handles potentiometer turning left and right, until another button press interrupts and selects a new part. The loop ends after seconds have been set
               

    }

    /* END Requirement 4 - Set Date/Time */
=======
>>>>>>> parent of d302256 (Experimental set datetime code)
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


