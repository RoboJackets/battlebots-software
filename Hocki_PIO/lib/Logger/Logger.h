#ifndef LOGGER_H
#define LOGGER_H

#define LOG_LENGTH 16

#include <fstream>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "AccelReading.h"

//Construct a logger, then call [logger].log() whenever you need to write something to SD.

class Logger {
    public:
        Logger();                    //Constructor
        void begin(String logName);                    //Constructor
        void log(String field, String value); //Writes "[field]: [value]" to the log file
        void log(String field);                    //Writes "[field]" to the log file
        void log(String field, int value);          //Writes "[field]: [value]" to file, but string-ifys ints for you
        void log(String field, double value);        //Writes "[field]: [value]" to file, but string-ifys floats for you
        void logStampedArray(String field, int times[], double values[], int length); //Logs an array of timestamped values to file
        void close();
        void flush();
        void addToOutputString(float reading);
        void addLine(AccelReading val1, AccelReading val2, AccelReading val3, AccelReading val4);
        void dump();
        volatile int lineCount = 0;
        volatile bool dumpFlag = false; // tells the main loop to dump
        ~Logger();                                      //Destructor

    private:        
        File logFile;                                   //Log file
        AccelReading logs1[LOG_LENGTH];
        AccelReading logs2[LOG_LENGTH];
        volatile bool whichLog = false; // false = logs1, true = logs2
        volatile int logIdx = 0;

};

#endif