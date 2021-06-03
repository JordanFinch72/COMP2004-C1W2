#include "mbed.h"
#include "uop_msb_2_0_0.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "BMP280_SPI.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"
#include "EthernetInterface.h"
#include "TCPSocket.h"

// HTML Directives
#define HTTP_STATUS_LINE "HTTP/1.0 200 OK"
#define HTTP_HEADER_FIELDS "Content-Type: text/html; charset=utf-8"
#define HTTP_MESSAGE_BODY ""                                     \
"<html>" "\r\n"                                                  \
"  <head><title>Environmental Sensor Readings</title></head>" "\r\n"       \
"  <body style=\"display: flex; flex-flow: column wrap; align-items: center;\">" "\r\n"       \
"    <div style=\"display: flex; flex-flow: row wrap; align-items: center;\">" "\r\n"                         \
"      <h1>Datetime:</h1>" "\r\n"                                \
"      <p>{{0}}</p>" "\r\n"                                      \
"    </div>" "\r\n"                                              \
"    <div style=\"display: flex; flex-flow: row wrap; align-items: center;\">" "\r\n"                         \
"      <h1>Temperature:</h1>" "\r\n"                                \
"      <p>{{1}}C</p>" "\r\n"                                      \
"    </div>" "\r\n"                                              \
"    <div style=\"display: flex; flex-flow: row wrap; align-items: center;\">" "\r\n"                         \
"      <h1>Pressure:</h1>" "\r\n"                                \
"      <p>{{2}}mBar</p>" "\r\n"                                      \
"    </div>" "\r\n"                                              \
"    <div style=\"display: flex; flex-flow: row wrap; align-items: center;\">" "\r\n"                         \
"      <h1>LDR:</h1>" "\r\n"                                \
"      <p>{{3}}V</p>" "\r\n"                                      \
"    </div>" "\r\n"                                              \
"  </body>" "\r\n"                                               \
"</html>" "\r\n"
    
#define HTTP_TEMPLATE HTTP_STATUS_LINE "\r\n"   \
                      HTTP_HEADER_FIELDS "\r\n" \
                      "\r\n"                    \
                      HTTP_MESSAGE_BODY "\r\n"

#define BUFFER_SIZE         2000 // After an hour, maximum 1,800 records will be flushed. See "SETT" for details.
#define CONSUME_MAX_SECONDS 60   // Max. 1 SD write every 60 seconds

using namespace uop_msb_200;
using namespace std;

// Environmental inputs
BMP280_SPI bmp280(PB_5, PB_4, PB_3, PB_2);
AnalogIn ldr(AN_LDR_PIN);

// User control inputs
InterruptIn btnA(BTN1_PIN);
InterruptIn btnUser(USER_BUTTON);
AnalogIn potentiometer(PA_0);

// Outputs
LCD_16X2_DISPLAY lcdDisplay;
DigitalOut redLED(TRAF_RED1_PIN);
DigitalOut greenLED(TRAF_GRN1_PIN);

// Misc
Ticker ticker;
EthernetInterface ethernetInterface;

// Functions //
// TODO: Make sure they're all here
void changePart();
void displayDatetime();
void getUserInput();
void handleDatetimeChange();
void sdWrite();
void sdFlushEject();
void sampleEnvironment();
void serialThread();
void serialMessage(string);
void logMessage(string, bool);
void refreshServer();

// SD Card
static SDBlockDevice sdBlockDevice(PB_5, PB_4, PB_3, PF_3); // SD Card Block Device

// Globals
bool loggingEnabled = false; // Switched by user-input command to enable/disable logging
Semaphore semWrite;
Semaphore semSample(1);
unsigned short sampleRate = 1000;
EventQueue serialQueue;
Semaphore semDateChanging;

// Threads
Thread tSample, tSDWrite, tSerialComm, tNetComm, tDatetime, tDatetimeChange, tButton, tInput;
osThreadId_t tDatetimeChangeId, tSDWriteId;

// Classes & Structs //

// SensorData struct: encapsulates data gathered by the board's environmental sensors (via sampleEnvironment())
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

    // Formats struct data. Returns char pointer, which is later freed by the calling scope
    char* getData()
    {
        char* data = (char*) malloc(53 * sizeof(char));
        sprintf(data, "Temp: %.2fC | Pressure: %.2fmBar | Light: %.4fV", this->temperature, this->pressure, this->lightLevel);
        return data;
    }
};

// Datetime struct: encapsulates data pertaining to the date and time.
struct Datetime
{
	// Initial datetime = 2021-01-01 00:00:00 (pulling current datetime from a server would nullify the point of implentation (ii))
    char day = 1;
    char month = 1;
    unsigned short year = 2021;
    unsigned short hour;
    unsigned short minute;
    unsigned short second;
    unsigned short changePart = 0; // Part of datetime being modified by user
                                   // [0: wait, 1: day, 2: month, 3: year, 4: hour, 5: // minute, 6: second]. 
                                   //   - Not enum because can't ++dtPart etc. if enum (makes code clunky and gross)

    // Formats struct data. Returns char pointer, which is later freed by the calling scope
    char* getTimestamp()
    {
        char* timestamp = (char*) malloc(19 * sizeof(char));
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", this->year, this->month, this->day, this->hour, this->minute, this->second); // ISO 8601-compliant
        return timestamp;
    }

    // Formats struct data sans the seconds. Returns timestamp fit for LCD display.
    char* getTimestampLCD()
    {
        char* timestamp = (char*) malloc(16 * sizeof(char));
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d", this->year, this->month, this->day, this->hour, this->minute); // Removes seconds as (a) they are not set by the user; (b) they trail off the display and it looks ugly
        return timestamp;
    }

    // Increments the time. Called every 1s
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

// FIFOBuffer class: contains own struct and member functions/properties required to buffer data to stagger SD writes
class FIFOBuffer
{

    // BufferData struct: encapsulates data which will be stored in the buffer to stagger SD writes
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

        // Formats struct data (and sub-data) into a string. Returns formatted string. Called by readBuffer()
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
        int itemCount = 0, freeSpace = BUFFER_SIZE;            // Buffer index trackers
        Mutex bufferLock;                                      // Lock to prevent producer and consumer functions from manipulating buffer at the same time

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

        // Consume the entire buffer (used for writing blocks to the SD card)
        void consume()
        {        
            //REPORT: printf("CONSUMING...\n"); 

            /* START Requirement 2 - SD Card Writing */

            // Release sempahore for the tSDWrite thread.
            semWrite.release();

            /* END Requirement 2 - SD Card Writing */
        }
        
        // Reads buffer data according from specified <start> to <end>. Will flush (clear) the buffer if <flush> set to true. Returns string of concatenated buffer data
        string readBuffer(int end, int start, bool flush)
        {            
            //REPORT: printf("Locking buffer...\n");

            bufferLock.lock();
                // Create a copy of buffer for expedience
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

            // Read stringified buffer data from buffer copy
            string buffer_string = "";            
            int i = 0;
            for(i = start; i < end; ++i)
            {
                if(flush) greenLED = !greenLED; // Flash green LED when flushing
                buffer_string += buffer_copy[i].getData();
            }
            delete[] buffer_copy;
            //REPORT: printf("<%d>\n", i);
            return buffer_string;
        }

        // Returns the most recent record/sample in the buffer.
        string readLastRecord()
        {
            return readBuffer(itemCount, itemCount-1, false); // Read from penultimate record until the ultimate record
        }

};
FIFOBuffer fifoBuffer;

// ISR called by tDatetimeChange to cycle through parts to change
void changePart() 
{    
    if (dateTime.changePart != 5) osSignalSet(tDatetimeChangeId, 1); // Signal tDatetimeChange thread to start letting the user change the date/time (if not done being set)
    dateTime.changePart = (dateTime.changePart < 5) ? dateTime.changePart + 1 : 0; // Cycle through parts to change. If it's already 5, then set to 0
}

// Runs on own thread to update LCD display with current date and time every 1000ms (in synch with timeInc())
void displayDatetime()
{    
    while(true)
    {
        lcdDisplay.cls(); // Clear the display

        // Print out LCD-friendly timestamp to the display
        char* timestampLCD = dateTime.getTimestampLCD();
        lcdDisplay.printf("%s", timestampLCD);
        free(timestampLCD);

        // Indicate being-changed part if appropriate (why doesn't English have imperfect adjectival verbs?)
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

// Runs on own thread to permanently respond to user commands. Waits until user inputs commands and hits ENTER before reacting
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

        // Log the command as per requirements
        string concatenated = "Command received: " + command + " " + variable + "\n";
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
				// TODO: Uncomment this and see if it works. Otherwise, no worries.
				// (Will still throw errors after 269 or whatever, so implement it around that, I guess)

				// TODO: After this, once FULLY tested, test to see if writing to SD with "a" works. If so, do that

				// TODO: Finally, comment every function, etc. better (with @params and shit) and finish the report up

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
				sdFlushEject();

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

        // Log as per requirement
        concatenated = "Command parsed: " + command + " " + variable + "\n";
        logMessage(concatenated, false);
    }
}

// Runs on own thread. Waits for button press signal to allow user to change part of the date/time (displayed on the LCD display)
void handleDatetimeChange()
{
    /* START Requirement 4 - Set Date/Time */    

    tDatetimeChangeId = ThisThread::get_id(); // Acquire ID for signalling 
    btnA.rise(&changePart);
    while(true)
    {
        //REPORT: printf("Changing part %d...\n", dateTime.changePart);
        osSignalWait(1, 10000000); // 10,000 seconds I shall wait until called upon by -R-o-h-a-n- the changePart() ISR
        
        if(dateTime.changePart == 1) // YEAR
        {
            float pot_val = potentiometer.read(); // Read potentiometer rotation (1.0 == all the way clockwise, 0.0 == all the way counter-clockwise)
            int direction = 0; // 0 == stable
            if(pot_val > 0.66)
                direction = 1;
            else if(pot_val < 0.33)
                direction = -1;

            if(direction != 0) dateTime.year += direction;
            wait_us(1000000); // Wait 1s between reads to stop the year from zooming past the Heat Death of the Universe
        } 
        else if(dateTime.changePart == 2) // MONTH
        {
            float pot_val = potentiometer.read();       // Read potentiometer rotation

            dateTime.month = 12 * pot_val;              // Multiplication of maximum value used to set value
            if(dateTime.month == 0) dateTime.month = 1; // Minimum allowed month
        } 
        else if(dateTime.changePart == 3) // DAY
        {
            float pot_val = potentiometer.read(); 

            // Maximum value will depend on the currently-set month
            if(dateTime.month == 4 || dateTime.month == 6 || dateTime.month == 9 || dateTime.month == 11)
                dateTime.day = 30 * pot_val;
            else if(dateTime.month == 2)                // February: the ultimate edge case.
                dateTime.day = 28 * pot_val;
            else
                dateTime.day = 31 * pot_val;

            if(dateTime.day == 0) dateTime.day = 1;     // Minimum allowed day
        } 
        else if(dateTime.changePart == 4) // HOUR
        {
            float pot_val = potentiometer.read();
            dateTime.hour = 23 * pot_val;               // Between 00:00 and 23:00, so slightly different than other percentile calculations
        } 
        else if(dateTime.changePart == 5) // MINUTE
        {
            float pot_val = potentiometer.read();
            dateTime.minute = 59 * pot_val;
        }
    }

    /* END Requirement 4 - Set Date/Time */
}

// Runs on own thread, waiting for semaphore release signal to write to SD card in blocks. Will unmount and terminate when flag is set after user command.
void sdWrite() 
{
	while(true)
	{
		SDBlockDevice sdBlockDevice(PB_5, PB_4, PB_3, PF_3); // Re-initialise
		
		tSDWriteId = ThisThread::get_id();
		ThisThread::flags_clear(1 | 2); // Clear flags

		// Mount the SD card
		if(sdBlockDevice.init() != 0) 
		{
			// PLEASE NOTE: This will sporadically fail for no apparent reason. I suspect hardware fault (as supplied SD card also did not work properly)
			// If this happens during testing, try running it again and it should work.
			logMessage("[ERROR] SD mount failed.\n", true);
		}
		else
		{
			logMessage("SD mounted.\n", false);
			greenLED = 1;
		}

		// Open the file
		FATFileSystem fs("sd", &sdBlockDevice);
		FILE* fp = fopen("/sd/data.txt", "w");    
		if(fp == NULL) 
		{
			logMessage("[ERROR] File cannot be opened.\n", true);
			sdBlockDevice.deinit();
		}    

		// Runs until flag is sent to unmount the card
		while (ThisThread::flags_get() == 0) 
		{
			semWrite.acquire(); // Puts into waiting state until semaphore released by another process                
			//REPORT: printf("Writing to card...");        
			string buffer_contents = fifoBuffer.readBuffer(-1, 0, true);        
			printf("%s", buffer_contents.c_str());
			fprintf(fp, "%s", buffer_contents.c_str());
			logMessage("Wrote data block to SD card.\n", false); 
			greenLED = 1;
		}

		// Close file, unmount card, echo confirmation (spec didn't say "log it")
		fclose(fp);
		sdBlockDevice.deinit();
		greenLED = 0;
		serialQueue.call(serialMessage, "SD CARD: UNMOUNTED\n");

		// While it's unmounted, put in waiting state until re-mounted
		osSignalWait(2, 10000000);
		osSignalClear(tSDWriteId, 2);	
	}	
	return;
}

void buttonThread()
{
	btnUser.rise(&sdFlushEject);
}
void sdFlushEject()
{	
	if(greenLED == 0) // If unmounted, mount
	{		
		osSignalSet(tSDWriteId, 2);		
		semWrite.release(); // Immediately write to SD card
   		greenLED = 1;		
	}
	else // If mounted, unmount
	{
		tSDWrite.flags_set(1);  // Flag will end write check loop and eject SD card
		semWrite.release();     // SD write function will flush buffer
	}
	wait_us(1000000); // Wait 1s to prevent accidental double-tapping of button
}

// Runs on own thread every <sampleRate> milliseconds. 
// Semaphore self-releases but will be hogged upon user command to put thread into waiting state and disable sampling.
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
        wait_us(sampleRate*1000);
    }
}

// Runs on own thread, constantly dispatching events as they come
void serialThread()
{
    serialQueue.dispatch_forever();
}

// Prints <message> through the serial interface
void serialMessage(string message)
{
    printf("%s", message.c_str());
}

// Formats <message> as a logged message before sending to serialQueue. Triggers error if <isError> is true.
void logMessage(string message, bool isError)
{
    if(isError)
    {
        redLED = 1;
        error("%s", message.c_str());
    } 
    else if(loggingEnabled)
    {
        message = "[LOG] " + message;
        serialQueue.call(serialMessage, message);
    }       
}

// Initialises an onboard web page server and displays data upon user refresh
void refreshServer()
{    
	// Initialise ethernet connection
    ethernetInterface.connect();

    // Get the network address
    SocketAddress socketAddress;
    ethernetInterface.get_ip_address(&socketAddress);

    // Retrieve and log network address
	string ip_address = socketAddress.get_ip_address();
	if(!ip_address.empty())
		serialQueue.call(serialMessage, "IP Address: " + ip_address + "\n"); // Logging is OFF by default; also not a "logged" message per sé
	else
		logMessage("IP Address could not be retrieved.\n", true);
    
	// Open and bind socket to port 80 (a popular port; may need changing if blocked by other programs)
    TCPSocket socket;
    socket.open(&ethernetInterface);
    socket.bind(80);

    //Set socket to listening mode (up to 5 connections)
    nsapi_error_t socketError = socket.listen(5);
    if(socketError != 0) 
	{
        socket.close();	
		logMessage("Socket listening error ("+to_string(socketError)+")\n", true);
    }

    while(true)
    {
        TCPSocket* socketPtr = socket.accept(); // Wait until socket connection received (e.g. from browser refresh)        

        // Retrieve time and sensor data
		string timestamp = dateTime.getTimestamp();
		float temp = bmp280.getTemperature();
		float pres = bmp280.getPressure();
		char temperature[24];
		sprintf(temperature, "%.2f", temp);
		char pressure[24];
		sprintf(pressure, "%.4f", pres);
		char light_level[24];
		sprintf(light_level, "%.4f", (float) ldr);
        
        // Parse variables in HTML response 
		// (Mustache.js, eat your heart out)
		string html = string(HTTP_TEMPLATE);		// Stringify HTML template		
		size_t placeholder = html.find("{{0}}");   	// Find datetime placeholder {{0}}
        if(placeholder) 
		{
            html.replace(placeholder, 5, timestamp);
        }
		placeholder = html.find("{{1}}");   		// Find temperature placeholder {{1}}
        if(placeholder) 
		{
            html.replace(placeholder, 5, temperature);
        }
		placeholder = html.find("{{2}}");   		// Find pressure placeholder {{2}}
        if(placeholder) 
		{
            html.replace(placeholder, 5, pressure);
        }
		placeholder = html.find("{{3}}");   		// Find light level placeholder {{3}}
        if(placeholder) 
		{
            html.replace(placeholder, 5, light_level);
        }     

        // Send parsed response
        nsapi_size_or_error_t result = socketPtr->send(html.c_str(), strlen(html.c_str()));
		if(result <= 0)
			logMessage("0 bytes sent through network socket.", true);
        
        socketPtr->close(); // Close socket

		wait_us(1000000); // Wait 1s to prevent spam
    }
}

// The main thread
int main()
{
    // Reset LEDs
    redLED = 0;
    greenLED = 0;

    //Environmental sensor
    bmp280.initialize();
    
    // Start threads
    tSerialComm.start(serialThread);    
    tSample.start(sampleEnvironment);
    tDatetime.start(displayDatetime);
    tDatetimeChange.start(handleDatetimeChange);
    tSDWrite.start(sdWrite);
    tNetComm.start(refreshServer);
	tButton.start(buttonThread);
    tInput.start(getUserInput); 
}