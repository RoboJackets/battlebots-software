#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <SPI.h>
#include <SD.h>
#include "AccelReading.h"

//Construct a logger, then call [logger].log() whenever you need to write something to SD.

class Logger{
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
        int lineCount = 0;
        ~Logger();                                      //Destructor

    private:        
        File logFile;                                   //Log file

};

#endif