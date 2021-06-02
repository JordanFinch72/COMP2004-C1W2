#include "mbed.h"
#include "uop_msb_2_0_0.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "BMP280_SPI.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

#define BUFFER_SIZE         2000 // After an hour, maximum 1,800 records will be flushed. See "SETT" for details.
#define CONSUME_MAX_SECONDS 60   // Max. 1 SD write every 60 seconds

using namespace uop_msb_200;
using namespace std;

// Environmental inputs
BMP280_SPI bmp280(PB_5, PB_4, PB_3, PB_2);
AnalogIn ldr(AN_LDR_PIN);

// User control inputs
Buttons btns;
InterruptIn btnA(BTN1_PIN);
AnalogIn potentiometer(PA_0);


// Outputs
LCD_16X2_DISPLAY lcdDisplay;
DigitalOut redLED(TRAF_RED1_PIN);
DigitalOut greenLED(TRAF_GRN1_PIN);

// Ticker
Ticker ticker;

// Functions //
// TODO: Make sure they're all here
void changePart();
void displayDatetime();
void getUserInput();
void handleDatetimeChange();
void sdWrite();
void sampleEnvironment();
void serialThread();
void serialMessage(string);
void logMessage(string, bool);

// SD Card
static SDBlockDevice sdBlockDevice(PB_5, PB_4, PB_3, PF_3); // SD Card Block Device
static InterruptIn sdInserted(PF_4); // Interrupt for card insertion events // TODO: If SD card hasn't been inserted, this should switch a boolean to let SD writes happen

// Globals
bool loggingEnabled = false; // Switched by user-input command to enable/disable logging
Semaphore semWrite;
Semaphore semSample(1);
unsigned short sampleRate = 1000;
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
osThreadId_t tDatetimeChangeId;

// Classes & Structs //
struct SensorData
{
    float temperature;
    float pressure;
    float lightLevel;

    SensorData(){}
    SensorData(float temp, float pres, float light)
    {
        temperature = temp;
        pressure = pres;
        lightLevel = light;
    }

    char* getData()
    {
        char* data = (char*) malloc(53 * sizeof(char));
        sprintf(data, "Temp: %.2fC | Pressure: %.2fmBar | Light: %.4fV", this->temperature, this->pressure, this->lightLevel);
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
    unsigned short changePart = 0; // Part of datetime being modified by user
                                   // [0: wait, 1: day, 2: month, 3: year, 4: hour, 5: // minute, 6: second]. 
                                   //   - Not enum because can't ++dtPart etc. if enum (makes code clunky and gross)

    char* getTimestamp()
    {
        char* timestamp = (char*) malloc(19 * sizeof(char));
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

class FIFOBuffer
{
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

        string getData()
        {
            string data;
            char* timestamp = this->dateTime.getTimestamp();
            char* sData = this->sensorData.getData();
            data = "[" + (string)timestamp + "] " + (string)sData + "\n";
            free(timestamp);
            free(sData);
            return data;
        }
    };

    public:
        BufferData* buffer = new BufferData[BUFFER_SIZE];      // No need to dynamically expand: buffer is to buffer SD writes, not memory
        unsigned short consumeThreshold = CONSUME_MAX_SECONDS; // Default sample rate 1s = 60 records before a minute passes (see SETT for details)
    private:
        int itemCount = 0, freeSpace = BUFFER_SIZE;
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
                bufferLock.unlock();
                //REPORT: printf("%s", buffer[itemCount-1].getData().c_str());
                //REPORT: printf("Space: %d\n", freeSpace);
                //REPORT: printf("Count: %d\n", itemCount);

                // Also, call to consume if threshold reached
                if(itemCount >= consumeThreshold)
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
            //REPORT: printf("CONSUMING...\n"); 

            /* START Requirement 2 - SD Card Writing */

            // Release sempahore for the tSDWrite thread.
            semWrite.release();

            /* END Requirement 2 - SD Card Writing */
        }
        
        string readBuffer(int end, int start, bool flush)
        {            
            //REPORT: printf("Locking buffer...\n");

            bufferLock.lock();
                BufferData* buffer_copy = new BufferData[itemCount];
                memmove(buffer_copy, buffer, itemCount * sizeof(BufferData)); // Copy itemCount items into buffer copy

                if(end < 0 || end > itemCount) end = itemCount; // If READBUFFER -1 or READBUFFER <number_greater_than_itemcount>, return all

                // Clear the buffer
                if(flush)
                {
                    delete[] buffer;
                    buffer = new BufferData[BUFFER_SIZE];
                    freeSpace = BUFFER_SIZE;                       
                    itemCount = 0;
                }          
            bufferLock.unlock();  

            //REPORT: printf("Building buffer string...\n");

            // Convert buffer to string
            string buffer_string = "";            
            int i = 0;
            for(i = start; i < end; ++i)
            {
                if(flush) greenLED = !greenLED; // Flash green LED when flushing
                buffer_string += buffer_copy[i].getData(); // TODO: Figure out why this loop can't build a string past 269 iterations
            }
            delete[] buffer_copy;
            printf("<%d>\n", i);
            return buffer_string;
        }

        string readLastRecord()
        {
            return readBuffer(itemCount, itemCount-1, false); // Read from penultimate record until the ultimate record
        }

};
FIFOBuffer fifoBuffer;

/* Requirements *
 - Active: 1, 2, 3, 4, 8, 9, 12, 13
 - Passive: 5, 6, 7, 10, 11
*/

void changePart() 
{    
    if (dateTime.changePart != 5) osSignalSet(tDatetimeChangeId, 1);
    dateTime.changePart = (dateTime.changePart < 5) ? dateTime.changePart + 1 : 0; // If it's already 5, then set to 0      
}
void displayDatetime()
{    
    while(true)
    {
        lcdDisplay.cls();
        char* timestampLCD = dateTime.getTimestampLCD();
        lcdDisplay.printf("%s", timestampLCD);
        free(timestampLCD);

        // Indicate being-changed part (why doesn't English have imperfect adjectival verbs?)
        if (dateTime.changePart != 0) 
        {
            int offset = (dateTime.changePart >= 2) ? 2 : 1; // Year needs +1 offset; others need +2 offset

            // Indicate which datetime part is being changed
            lcdDisplay.locate(1, ((dateTime.changePart - 1) * 3) + offset);
            lcdDisplay.printf("^^");
        }

        if (dateTime.changePart == 0) dateTime.timeInc(); // Increment time only if it's not being changed by user
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

            if(input_char == 10) // ENTER
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
        while(i < 16);

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

            // N < 0: Entire buffer. N > 0: N records. Handled by readBuffer()
            serialQueue.call(serialMessage, fifoBuffer.readBuffer(n, 0, false));
        }
        else if(command == "SETT")
        {            
            float t = stof(variable);

            if(t >= 0.1f && t <= 30.0f)
            {                
                // Update buffer consume threshold to compensate for time constraints (min. once per hour, max. once per minute) 
                // This code segment attempts to balance the fact that write should be as infrequent as possible, but also there's a limit on buffer memory (and board memory, for that matter)                
                /* -- This code cannot be implemented due to buffer read loop being unable to iterate more than 269 times -- 
                unsigned short newThreshold;
                if(t <= 1)
                    newThreshold = CONSUME_MAX_SECONDS / t; // Flush buffer once a minute (e.g. 60/0.1 = 600 records before a MINUTE passes; 60/0.2 = 300; 60/0.9 = 67; 60/1 = 60)
                else if(t > 1)
                    newThreshold = CONSUME_MAX_SECONDS * (CONSUME_MAX_SECONDS/t); // Flush buffer once an hour (e.g. 2s = (60*(60/2)) = 1,800 records before an HOUR passes; 30s = (60*(60/30)) = 120 records before an HOUR passes)

                fifoBuffer.consumeThreshold = newThreshold;
                */
                    
                // Set the sampling period to <t> seconds (<ms> millseconds), print string to console
                sampleRate = t*1000;
                string message = "T UPDATED TO " + to_string(sampleRate) + "ms";
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
                // Start logging
                loggingEnabled = true;

                // Echo confirmation string
                serialQueue.call(serialMessage, "LOGGING: ACTIVE\n");
                
            }
            else if(variable == "OFF")
            {
                // Stop logging                
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

                // Echo confirmation string
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

void handleDatetimeChange()
{
    /* START Requirement 4 - Set Date/Time */    

    tDatetimeChangeId = ThisThread::get_id();        
    btnA.rise(&changePart);
    while(true)
    {
        //REPORT: printf("Changing part %d...\n", dateTime.changePart);
        osSignalWait(1, 10000000); // 10,000 seconds I shall wait until called upon by -R-o-h-a-n- the changePart() ISR
        
        if(dateTime.changePart == 1) // YEAR
        {
            // Read potentiometer and check if it's been moved up or down since last read
            float pot_val = potentiometer.read();
            int direction = 0; // 0 == stable
            if(pot_val > 0.66)
                direction = 1;
            else if(pot_val < 0.33)
                direction = -1;

            if(direction != 0) dateTime.year += direction;
            wait_us(1000000); // Wait 1s to stop the year from zooming past the Heat Death of the Universe
        } 
        else if(dateTime.changePart == 2)          // MONTH
        {
            float pot_val = potentiometer.read();

            dateTime.month = 12 * pot_val;
            if(dateTime.month == 0) dateTime.month = 1; // Minimum allowed month
        } 
        else if(dateTime.changePart == 3) // DAY
        {
            float pot_val = potentiometer.read(); // Percentage of max value for day

            if(dateTime.month == 4 || dateTime.month == 6 || dateTime.month == 9 || dateTime.month == 11)
                dateTime.day = 30 * pot_val;
            else if(dateTime.month == 2) // Ah, February... the ultimate edge case.
                dateTime.day = 28 * pot_val;
            else
                dateTime.day = 31 * pot_val;

            if(dateTime.day == 0) dateTime.day = 1;   // Minimum allowed day
        } 
        else if(dateTime.changePart == 4) // HOUR
        {
            float pot_val = potentiometer.read();
            dateTime.hour = 23 * pot_val; // Between 00:00 and 23:00, so slightly different than other percentile calculations
        } 
        else if(dateTime.changePart == 5) // MINUTE
        {
            float pot_val = potentiometer.read();
            dateTime.minute = 59 * pot_val;
        }
    }

    /* END Requirement 4 - Set Date/Time */
}

void sdWrite() 
{
    if(sdBlockDevice.init() != 0) 
    {
         // PLEASE NOTE: This will sporadically fail for no apparent reason. I suspect hardware fault (as supplied SD card also did not work properly)
         // If this happens during testing, try running it again and it should work.
        logMessage("[ERROR] SD mount failed.\n", true);
        return;
    }
    greenLED = 1;

    FATFileSystem fs("sd", &sdBlockDevice);
    FILE* fp = fopen("/sd/data.txt", "w");    
    if(fp == NULL) 
    {
        logMessage("[ERROR] File cannot be opened.\n", true);
        sdBlockDevice.deinit();
        return;
    }    

    while (ThisThread::flags_get() == 0) // Flag will be sent to unmount SD card
    {
        semWrite.acquire(); // Puts into waiting state until semaphore released by another process                
        //REPORT: printf("Writing to card...");        
        string buffer_contents = fifoBuffer.readBuffer(-1, 0, true);        
        //REPORT: printf("%s", buffer_contents.c_str());
        fprintf(fp, "%s", buffer_contents.c_str());
        logMessage("Wrote data block to SD card.\n", false); 
    }

    fclose(fp);    
    sdBlockDevice.deinit();
    greenLED = 0;
    serialQueue.call(serialMessage, "SD CARD: UNMOUNTED\n");
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
            logMessage("Sampled data.\n", false); // TODO: Uncomment this
            
            fifoBuffer.produce(sensorData);

        semSample.release();
        wait_us(sampleRate*1000);
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
    if(isError)
    {
        error("%s", message.c_str());
        redLED = 1;
    } 
    else if(loggingEnabled)
    {
        message = "[LOG] " + message;
        serialQueue.call(serialMessage, message);
    }       
}

int main()
{
    /* START REQUIREMENT 1 - Environmental Sensor */

    // Reset LEDs
    redLED = 0;

    //Environmental sensor
    bmp280.initialize();
    
    // Start threads
    tSerialComm.start(serialThread);    
    tSample.start(sampleEnvironment);
    tDatetime.start(displayDatetime);
    tDatetimeChange.start(handleDatetimeChange);
    tSDWrite.start(sdWrite);
    tInput.start(getUserInput);    

    /* END Requirement 1 */
}