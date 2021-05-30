#include "mbed.h"
#include "uop_msb_2_0_0.h"
#include <chrono>
#include <cstdio>
#include <iostream>
#include "BMP280_SPI.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

#define BUFFER_SIZE       16
#define CONSUME_THRESHOLD 12  // Consume once 75% full (12 samples) to mitigate/prevent buffer full error

using namespace uop_msb_200;
using namespace std;

//Digital Inputs (you could use DigitalIn or InterruptIn instead)
Buttons btns;
InterruptIn btnA(BTN1_PIN);

//Analogue Inputs
AnalogIn ldr(AN_LDR_PIN);

// Potentiometer
AnalogIn pot(PA_0);

//LED Outputs
DigitalOut redLED(TRAF_RED1_PIN);

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
void write_sd();
void serialThread();
void serialMessage(string);
void logMessage(string, bool);
void logThread();
int read_sd();

BMP280_SPI bmp280(PB_5, PB_4, PB_3, PB_2);

// SD Card
static SDBlockDevice sd_mine(PB_5, PB_4, PB_3, PF_3); // SD Card Block Device
static InterruptIn sd_inserted(PF_4); // Interrupt for card insertion events // TODO: If SD card hasn't been inserted, this should switch a boolean to let SD writes happen

// Globals
int dt_part = 0; // Date/Time Set Part [0: wait, 1: day, 2: month, 3: year, 4: hour, 5: minute, 6: second]. Not enum because can't ++dt_part if enum
Semaphore semWrite;
Semaphore semSample(1);
chrono::milliseconds sample_rate = 1000ms;
Semaphore semLogging(1);
bool loggingEnabled = true;
EventQueue logQueue;
EventQueue serialQueue;

Semaphore semDateChanging;

// Threads //
// You should use at least 4 separate and dedicated threads for:
//  - (i) sampling the data, 
//  - (ii) writing to the SD card; 
//  - (iii) communicating with the serial interface 
//  - (iv) communicating with the network. 
//  - Event Queues are recommended but not a requirement. In addition, you may also use interrupts if appropriate and where justified, again with suitable synchronization.
Thread tSample, tSDWrite, tSerialComm, tNetComm, tDatetime, tDatetimeChange, tInput;
Thread tDebug; // TODO: Delete before uploading to DLE

// Classes & Structs //
struct SensorData
{
    // TODO: Data types probably incorrect
    float temperature;
    float pressure;
    float light_level;

    SensorData(){}
    SensorData(float temp, float pres, float light)
    {
        temperature = temp;
        pressure = pres;
        light_level = light;
    }

    char* getData()
    {
        char* data = (char*) malloc(64 * sizeof(char));
        sprintf(data, "Temp: %.2fC | Pressure: %.2fmBar | Light: %.4fV", this->temperature, this->pressure, this->light_level);
        return data;
    }
};

struct Datetime
{
    char day = 1;
    char month = 1;
    unsigned short year = 2021;
    unsigned short hour;
    unsigned short minute;
    unsigned short second;

    char* getTimestamp()
    {
        char* timestamp = (char*) malloc(20 * sizeof(char));
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", this->year, this->month, this->day, this->hour, this->minute, this->second); // ISO 8601-compliant
        return timestamp;
    }

    // Returns timestamp fit for LCD display: removes seconds as (a) they are not set; (b) they trail off the display and it looks ugly
    char* getTimestampLCD()
    {
        char* timestamp = (char*) malloc(16 * sizeof(char));
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d", this->year, this->month, this->day, this->hour, this->minute);
        return timestamp;
    }

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
                        else ++this->month;
                    }
                    else ++this->day;
                }
                else ++this->hour;
            }
            else ++this->minute;
        }
        else ++this->second;
    }    
};
Datetime dateTime;

class FIFO_Buffer
{
    // TODO: Use C++ templates for higher marks
    // TODO: Use RTOS APIs to make it thread-safe
    // TODO: All thread synchronization should be handled by the class member functions
    //  - However that doesn't seem bloody possible.

    struct BufferData
    {
        Datetime dateTime;
        SensorData sensorData;        

        BufferData(){}
        BufferData(Datetime time, SensorData data)
        {
            dateTime = time;
            sensorData = data;
        }

        char* getData()
        {
            char* data = (char*) malloc(128 * sizeof(char));
            sprintf(data, "[%s] %s\n", this->dateTime.getTimestamp(), this->sensorData.getData());
            return data;
        }
    };

    public:
        BufferData buffer[BUFFER_SIZE]; // No need to dynamically expand: buffer is to buffer SD writes, not memory
        Thread tProducer, tConsumer; // Can't start threads inside a class?
    private:
        int itemCount = 0, freeSpace = BUFFER_SIZE; // TODO: Convert these to semaphores...
        Mutex bufferLock;

    public:    
        void produce(SensorData sensorData)
        {
            // If there isn't enough space
            if(freeSpace <= 0)
            {
                logMessage("[ERROR] Buffer full.\n", true);
            }
            else
            {
                // Write to the buffer, update counters
                bufferLock.lock();
                    buffer[itemCount++] = BufferData(dateTime, sensorData);
                    --freeSpace;
                    //printf("%s", buffer[itemCount-1].getData());
                    //printf("Space: %d\n", freeSpace);
                    //printf("Count: %d\n", itemCount);
                bufferLock.unlock();

                // Also, call to consume if threshold reached
                if(itemCount >= CONSUME_THRESHOLD)
                {                    
                    consume();
                }
            }
        }

        // Important TODO: "ONCE A MINUTE BEING THE MOST FREQUENT, ONE AN HOUR BEING THE SLOWEST"
        // Consume the entire buffer (used for writing blocks to the SD card)
        // The "read" operation (should be blocking)
        void consume()
        {        
            //printf("CONSUMING...\n"); 

            /* START Requirement 2 - SD Card Writing */

            // Release sempahore for the tSDWrite thread.
            semWrite.release();

            /* END Requirement 2 - SD Card Writing */
        }

        // TODO: This should block produce(). Use a Mutex, I guess. Or a lock or something.
        string readBuffer(int end, int start = 0, bool flush = false)
        {            
            bufferLock.lock();
                BufferData buffer_copy[BUFFER_SIZE];            // Make a copy of buffer
                memcpy(buffer_copy, buffer, sizeof(buffer));
                if(end < 0 || end > itemCount) end = itemCount; // If READBUFFER -1 or READBUFFER <number_greater_than_itemcount>, return all
                // Clear the buffer
                if(flush)
                {
                    freeSpace = BUFFER_SIZE;                       
                    itemCount = 0;
                    memset(buffer, 0, sizeof buffer);   
                }          
            bufferLock.unlock();  

            // Convert buffer to string
            string buffer_string = "";
            for(int i = start; i < end; i++)
            {
                buffer_string += buffer_copy[i].getData();
            }
            return buffer_string;
        }

        string readLastRecord()
        {
            return readBuffer(itemCount, itemCount-1); // Read from penultimate record until the ultimate record
        }

};
FIFO_Buffer fifoBuffer;

/* Requirements *
 - Active: 1, 2, 3, 4, 8, 9, 12, 13
 - Passive: 5, 6, 7, 10, 11
*/

void changePart() 
{
    if(dt_part == 0) semDateChanging.release(); // Release a semaphore to allow date change
    dt_part = (dt_part < 5) ? dt_part + 1 : 0; // If it's already 5, then set to 0
    if(dt_part == 0) logMessage("Date/Time set.\n", false);
}
void displayDatetime()
{    
    while(true)
    {
        lcd_disp.cls();
        lcd_disp.printf("%s", dateTime.getTimestampLCD());

        // Indicate being-changed part (why doesn't English have imperfect adjectival verbs?)
        if(dt_part != 0)
        {            
            int offset = (dt_part >=2) ? 2 : 1; // Year needs +1 offset; others need +2 offset

            // Indicate which datetime part is being changed
            lcd_disp.locate(1, ((dt_part-1) * 3) + offset);
            lcd_disp.printf("^^");
        }

        if(dt_part == 0) dateTime.timeInc(); // Increment time only if it's not being changed by user     
        wait_us(1000000); // Wait 1s
    }
    
}

void getUserInput()
{
    while(true)
    {
        serialQueue.call(serialMessage, "\nEnter a command (see Table 2 for details). Press ENTER to finish: \n");

        string command = "", variable = "";
        char input_char;
        int i = 0;
        bool is_variable = false;
        do
        {
            input_char = getchar();
            printf("%c", input_char);

            if(input_char == 10) 
                break; // Break BEFORE appending to string
            else if(input_char == ' ')
            {
                is_variable = true; // Switch to variable after space
                continue;
            }

            // Before space, command. After space, variable.
            if(!is_variable) 
                command += input_char;
            else 
                variable += input_char;
        }
        while(i < 32);

        string concatenated = "Command received: " + command + variable + "\n";
        logMessage(concatenated, false);

        if(command == "READ")
        {
            if(variable == "NOW")
            {
                // Reads back the current (latest) record in the FIFO (date, time, temperature, pressure, light)
                serialQueue.call(serialMessage, fifoBuffer.readLastRecord());
            }
        }
        else if(command == "READBUFFER")
        {
            int n = stoi(variable);

            // N < 0: Entire buffer. N > 0: N records. Handled by readBuffer() // TODO: Do through serial thread properly
            serialQueue.call(serialMessage, fifoBuffer.readBuffer(n));
        }
        else if(command == "SETT")
        {            
            float t = stof(variable);
            int ms = t*1000;

            if(t >= 0.1f && t <= 30.0f)
            {
                // Set the sampling period to <t> seconds and return a string "“"T UPDATED TO <t>ms""

                sample_rate = (chrono::milliseconds) ms;
                char* message = (char*) malloc(24 * sizeof(char));
                sprintf(message, "T UPDATED TO %dms\n", ms);
                serialQueue.call(serialMessage, message);
            }
            else
            {
                // Out of range error
                logMessage("[ERROR] SETT variable out of range.\n", true);
            }
        }
        else if(command == "STATE")
        {
            if(variable == "ON")
            {
                // Start sampling
                semSample.release();

                // Echo confirmation string
                serialQueue.call(serialMessage, "SAMPLING: ACTIVE\n");
                
            }
            else if(variable == "OFF")
            {
                // Stop sampling                
                semSample.acquire();

                // Echo confirmation string
                serialQueue.call(serialMessage, "SAMPLING: INACTIVE\n");
            }
            else
            {
                logMessage("[ERROR] STATE variable must be ON or OFF.\n", true);
            }
        }
        else if(command == "LOGGING")
        {
             if(variable == "ON")
            {
                // Start sampling
                loggingEnabled = true;

                // Echo confirmation string
                serialQueue.call(serialMessage, "LOGGING: ACTIVE\n");
                
            }
            else if(variable == "OFF")
            {
                // Stop sampling                
                loggingEnabled = false;

                // Echo confirmation string
                serialQueue.call(serialMessage, "LOGGING: INACTIVE\n");
            }
            else
            {
                logMessage("[ERROR] LOGGING variable must be ON or OFF.\n", true);
            }
        }
        else if(command == "SD")
        {
            if(variable == "E")
            {
                // Flush AND eject the SD card (unmount)                
                semWrite.release();     // SD write function will flush buffer
                tSDWrite.flags_set(1);  // Flag will end write check loop and eject SD card

                // Echo confirmation string  // TODO: Do this through the thread etc. etc.
                serialQueue.call(serialMessage, "SD CARD: FLUSHED, EJECTED\n");
            }
            else if(variable == "F")
            {
                // Flush the SD card
                semWrite.release(); // SD write function will flush buffer

                // Echo confirmation string
                serialQueue.call(serialMessage, "SD CARD: FLUSHED\n");
            }
            else
            {
                logMessage("[ERROR] SD variable must be E or F.\n", true);
            }
        }

        concatenated = "Command parsed: " + command + variable + "\n";
        logMessage(concatenated, false);
    }

    

}

// Remember that main() runs in its own thread in the OS
int main()
{
    /* START REQUIREMENT 1 - Environmental Sensor */

    //Environmental sensor
    bmp280.initialize();

    //tDebug.start(debugStuff);
    tSerialComm.start(serialThread);
    tSample.start(sampleEnvironment);
    tDatetime.start(displayDatetime);
    tDatetimeChange.start(handleDatetimeChange);
    tSDWrite.start(write_sd);
    tInput.start(getUserInput);
    

    /* END Requirement 1 */
}

void handleDatetimeChange()
{
    /* START Requirement 4 - Set Date/Time */
    // TODO: Consider: 
    //          - Making btnB allow the user to go back a part
        
    btnA.rise(&changePart);
    while(true)
    {
        semDateChanging.acquire();
        if (dt_part == 1) // YEAR
        {
            // Read potentiometer and check if it's been moved up or down since last read
            float pot_val = pot.read();
            int direction = 0; // 0 == stable
            if(pot_val > 0.66) direction = 1;
            else if(pot_val < 0.33) direction = -1;

            if(direction != 0) dateTime.year += direction;
            ThisThread::sleep_for(1000ms); // To stop the year from zooming past the Great Collapse of the Universe
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
    }

    /* END Requirement 4 - Set Date/Time */
}

void write_sd() 
{    
    if(sd_mine.init() != 0) 
    {
        logMessage("[ERROR] SD mount failed.\n", true);
        return;
    }
    FATFileSystem fs("sd", &sd_mine);

    // TODO: Log the event
    while(ThisThread::flags_get() == 0) // Flag will be sent to unmount SD card
    {
        semWrite.acquire(); // Puts into waiting state until semaphore released by another process        
        string buffer_contents = fifoBuffer.readBuffer(-1, 0, true);   

        FILE *fp = fopen("/sd/sensor_data.txt", "w");

        if (fp == NULL)
        {
            logMessage("[ERROR] File cannot be opened.\n", true);
            sd_mine.deinit();
            return;
        } 
        else 
        {        
            //printf("WRITTEN:\n %s", buffer_contents.c_str());
            fprintf(fp, "%s", buffer_contents.c_str());
            fclose(fp);
            logMessage("Wrote data block to SD card.\n", false);
        } 
    }
    
    serialQueue.call(serialMessage, "SD CARD: UNMOUNTED\n");
    sd_mine.deinit();
    return;
}

void sampleEnvironment()
{
    while(true)
    {        
        // Wait for sample semaphore (1 available by default; stolen by STATE OFF command)
        semSample.acquire();

            // Collect sample data
            SensorData sensorData = SensorData(bmp280.getTemperature(), bmp280.getPressure(), ldr);
            logMessage("Sampled data.\n", false);            
            
            fifoBuffer.produce(sensorData);

        semSample.release();
        ThisThread::sleep_for(sample_rate);
    }
}

void serialThread()
{
    serialQueue.dispatch_forever();
}
void serialMessage(string message)
{
    printf("%s", message.c_str());
}
void logMessage(string message, bool isError)
{
    if(loggingEnabled || isError)
    {
        message = "[LOG] " + message;
        serialQueue.call(serialMessage, message);
        if(isError) redLED = 1;
    }       
}