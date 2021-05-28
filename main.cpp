#include "mbed.h"
#include "uop_msb_2_0_0.h"
#include <iostream>
#include "BMP280_SPI.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

#define BUFFER_SIZE       1024  // Sufficient to hold 16 samples (sample size max. 64 bytes)
#define CONSUME_THRESHOLD 768   // Consume once 75% full (12 samples) to mitigate/prevent buffer full error

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
InterruptIn btnA(BTN1_PIN);

//Analogue Inputs
AnalogIn ldr(AN_LDR_PIN);

// Potentiometer
AnalogIn pot(PA_0);

//LED Outputs
LatchedLED ledStrip(LatchedLED::STRIP);
LatchedLED ledDigit(LatchedLED::SEVEN_SEG);

//Buzzer
Buzzer buzz;

//LCD Display
LCD_16X2_DISPLAY lcd_disp;

// Ticker
Ticker ticker;

// Functions //
// TODO: Make sure they're all here
void debugStuff();
void sampleEnvironment();
void handleDatetimeChange();
int write_sd();
int read_sd();

BMP280_SPI bmp280(PB_5, PB_4, PB_3, PB_2);

// SD Card
static SDBlockDevice sd_mine(PB_5, PB_4, PB_3, PF_3); // SD Card Block Device
static InterruptIn sd_inserted(PF_4); // Interrupt for card insertion events // TODO: If SD card hasn't been inserted, this should switch a boolean to let SD writes happen

// Globals
int dt_part = 0; // Date/Time Set Part [0: wait, 1: day, 2: month, 3: year, 4: hour, 5: minute, 6: second]. Not enum because can't ++dt_part if enum
int day = 1, month = 1, year = 1970, hour = 0, minute = 0, second = 0; // Datetime data (no need to use a heavy class/struct here unless spec requires it)
bool date_changing = false;
bool write_block = true;


// Threads //
// You should use at least 4 separate and dedicated threads for:
//  - (i) sampling the data, 
//  - (ii) writing to the SD card; 
//  - (iii) communicating with the serial interface 
//  - (iv) communicating with the network. 
//  - Event Queues are recommended but not a requirement. In addition, you may also use interrupts if appropriate and where justified, again with suitable synchronization.
Thread tSample, tSDWrite, tSerialComm, tNetComm, tDatetime, tDatetimeChange;
Thread tDebug; // TODO: Delete before uploading to DLE

// Classes & Structs //
struct SensorData
{
    // TODO: Data types probably incorrect
    float temperature;
    float pressure;
    float light_level;
};
SensorData sensorData;

class FIFO_Buffer
{
    // TODO: Use C++ templates for higher marks
    // TODO: Use RTOS APIs to make it thread-safe
    // TODO: All thread synchronization should be handled by the class member functions

    public:
        char buffer[BUFFER_SIZE]; // TODO: Make this fixed-size in memory somehow (figure out the char array into string bs)        
        Thread tProducer, tConsumer;
    private:
        int itemCount = 0, freeSpace = BUFFER_SIZE;


    public:
        void produce(string message)
        {
            size_t message_length = message.length();

            // If there isn't enough space
            if(freeSpace < message_length)
            {
                printf("NOT ENOUGH SPACE\n");

                // TODO: Full buffer = critical error event, "reported" (presumably he means logged, as well)
            }
            else
            {
                printf("WRITING TO BUFFER...\n");   
                printf("%s", buffer);
                printf("Space: %d\n", freeSpace);
                printf("Count: %d\n", itemCount);

                // Write to the buffer, decrement freeSpace with the size of the message          
                strncat(buffer, message.c_str(), message_length);
                freeSpace -= message_length;
                itemCount += message_length;

                // Also, call to consume if threshold reached
                if(itemCount >= CONSUME_THRESHOLD)
                {
                    // TODO: FIFO read should be blocking (fifoBuffer is read in the write_sd() function. Should block production until read is finished, I guess)
                    consume();
                }
            }
        }

        // Consume the entire buffer (used for writing blocks to the SD card)
        // The "read" operation (should be blocking)
        void consume()
        {        
            printf("CONSUMING...\n"); 

            /* START Requirement 2 - SD Card Writing */

            // TODO: Instead of this, send signal to write_sd thread SOMEHOW. Or an interrupt or something. Anything to wake it up.
            write_block = false;

            /* END Requirement 2 - SD Card Writing */
        }

        // TODO: This should block produce(). Use a Mutex, I guess. Or a lock or something.
        string readBuffer()
        {
            char buffer_copy[BUFFER_SIZE];
            strcpy(buffer_copy, buffer);
            freeSpace = 1024;
            itemCount = 0;
            memset(buffer, 0, sizeof buffer); // Clear the buffer
            return buffer_copy;
        }

};
FIFO_Buffer fifoBuffer;

struct Datetime
{
    char day = 1;
    char month = 1;
    unsigned short year = 2021;
    char hour;
    char minute;
    char second;

    // Increment the time. Called every 1s
    void timeInc()
    {
        if(this->second == 59)
        {
            this->second = 0;

            if(this->minute == 59)
            {
                this->minute = 0;
                
                if(this->hour == 23)
                {
                    this->hour = 0;

                    if((this->day == 30 && (this->month == 4 || this->month == 6 || this->month == 9 || this->month == 11)) || 
                    (this->day == 28 && (this->month == 2)) || 
                    (this->day == 31 && (this->month == 1 || this->month == 3 || this->month == 5 || this->month == 7 || this->month == 8 || this->month == 10 || this->month == 12)))
                    {
                        this->day = 1;
                        
                        if(this->month == 12)
                        {
                            this->month = 1;

                            ++this->year;
                        }
                        else
                        {
                            ++this->month;
                        }
                    }
                    else
                    {
                        ++this->day;
                    }
                }
                else 
                {
                    ++this->hour;
                }
            }
            else
            {
                ++this->minute;
            }
        }
        else
        {
            ++this->second;
        }
    }    
};
Datetime dateTime;

/* Requirements *
 - Active: 1, 2, 3, 4, 8, 9, 12, 13
 - Passive: 5, 6, 7, 10, 11
*/

void changePart() 
{
  dt_part = (dt_part < 5) ? dt_part + 1 : 0; // If it's already 5, then set to 0
  date_changing = dt_part != 0;              // If 0, stop changing
}
void displayDatetime()
{
    // Requirement TODO: lcd_disp.printf()s should be done through the serial communication thread (tSerialComm)
    while(true)
    {
        lcd_disp.cls();
        lcd_disp.printf("%04d-%02d-%02d %02d:%02d", dateTime.year, dateTime.month, dateTime.day, dateTime.hour, dateTime.minute); // ISO 8601-compliant

        // Indicate being-changed part (why doesn't English have imperfect adjectival verbs?)
        if(date_changing)
        {            
            int offset = (dt_part >=2) ? 2 : 1; // Year needs +1 offset; others need +2 offset

            // Indicate which datetime part is being changed
            lcd_disp.locate(1, ((dt_part-1) * 3) + offset);
            lcd_disp.printf("^^");
        }
        
        ThisThread::sleep_for(1000ms);
    }
    
}
void timeInc()
{
    // Performance TODO: This will cause quite a bit of overhead. Find a way to call struct function with attach()
    //  - Otherwise, just keep put the time tracking logic in here, I guess... Maybe keep the data global. Ugly, though
    dateTime.timeInc();
}

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
    tDatetime.start(displayDatetime);
    tDatetimeChange.start(handleDatetimeChange);
    tSDWrite.start(write_sd); // TODO: If this doesn't work, write_sd() doesn't seem to work when running on a thread
    

    /* END Requirement 1 */
}

void handleDatetimeChange()
{
    /* START Requirement 4 - Set Date/Time */
    // TODO: Consider: 
    //          - Making btnB allow the user to go back a part
        
    btnA.rise(&changePart);
    ticker.attach(&timeInc, 1000ms);
    while (true) 
    {
        if (!date_changing)
            ThisThread::sleep_for(5000ms); // Check for initiation button press every 5 second

        if (dt_part == 1) // YEAR
        {
            // Read potentiometer and check if it's been moved up or down since last read
            float pot_val = pot.read();
            int direction = 0; // 0 == stable
            if(pot_val > 0.66) direction = 1;
            else if(pot_val < 0.33) direction = -1;

            printf("POT_VAL %f | DIRECTION %d | YEAR %d\n", pot_val, direction, dateTime.year);

            if(direction != 0) dateTime.year += direction;
            ThisThread::sleep_for(1000ms); // TODO: Make sure this can't be interrupted
        }
        else if (dt_part == 2) // MONTH
        {
            float pot_val = pot.read();

            dateTime.month = 12 * pot_val;
            if (dateTime.month == 0) dateTime.month = 1;
        }
        else if (dt_part == 3) // DAY
        {
            float pot_val = pot.read(); // Percentage of max value for day

            if (dateTime.month == 4 || dateTime.month == 6 || dateTime.month == 9 || dateTime.month == 11)
                dateTime.day = 30 * pot_val;
            else if (dateTime.month == 2) // Ah, February... the ultimate edge case.
                dateTime.day = 28 * pot_val;
            else
                dateTime.day = 31 * pot_val;
            if (dateTime.day == 0)
                dateTime.day = 1; // Minimum allowed day
        }
        else if (dt_part == 4) // HOUR
        {
            float pot_val = pot.read();

            dateTime.hour = 23 * pot_val; // Between 00:00 and 23:00, so slightly different than other percentile calculations
        }
        else if (dt_part == 5) // MINUTE
        {
            float pot_val = pot.read();

            dateTime.minute = 59 * pot_val;
        }

        // TODO: Better yet, stop timeInc from ticking until this operation is complete
        dateTime.second = 0; // Reset second so it doesn't change the minute outside of user control
    }

    /* END Requirement 4 - Set Date/Time */
}

int write_sd() 
{    
    // TODO: Why aren't there any signals, damn it?
    // TODO: Log the event
    while(true)
    {
        if(write_block)
        {
            ThisThread::sleep_for(1000ms);
        }
        else
        {
            write_block = true;
            string buffer_contents = fifoBuffer.readBuffer(); // TODO: Should be blocking

            printf("Initialise and write to a file\n");

            // call the SDBlockDevice instance initialisation method.
            if (0 != sd_mine.init()) {
                printf("Init failed \n");
                return -1;
            }

            FATFileSystem fs("sd", &sd_mine);
            FILE *fp = fopen("/sd/sensor_data.txt", "w");

            if (fp == NULL)
            {
                printf("Cannot be opened");
                // File could not be opened for write
                sd_mine.deinit();
                return -1;
            } 
            else 
            {        
                // TODO: This read should be blocking (will need to figure out the fucking internal class threads, then. Joyous)
                printf("BRO %s\n", buffer_contents.c_str());
                fprintf(fp, "%s", buffer_contents.c_str());
                fclose(fp);
                return sd_mine.deinit(); // Returns 0
            } 
        }
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
    FILE *fp = fopen("/sd/sensor_data.txt", "r");
    if (fp == NULL) 
    {
        error("Could not open or find file for read\n");
        sd_mine.deinit();
        return -1;
    }
    else 
    {
        // Read the file
        char buff[64];
        buff[63] = 0;
        while (!feof(fp)) 
        {
            fgets(buff, 63, fp);
            printf("%s", buff);
        }
        // Tidy up here
        fclose(fp);
        return sd_mine.deinit();
    }
}

void sampleEnvironment()
{
    while(true)
    {
        //printf("Collecting sampling data...\n");

        // Collect sample data
        sensorData.light_level = ldr;
        sensorData.temperature = bmp280.getTemperature();
        sensorData.pressure = bmp280.getPressure();

        // TODO: send data to FIFO buffer (on this thread(?))
        // TODO: MIght be overhead on to_string() call; if there is jitter, consider correcting this somehow        

        string message = "Temp: "+to_string(sensorData.temperature)+" || Pressure: "+to_string(sensorData.pressure)+" || Light: "+to_string(sensorData.light_level)+"\n";        

        if(message.length() <= 64)
            fifoBuffer.produce(message);
        else
            printf("ERROR: Sample message length too long (%d)!\n", message.length());

        // Wait a second (minus however long it took? Timer would probably cause overhead, though)
        ThisThread::sleep_for(1000ms);
    }
}