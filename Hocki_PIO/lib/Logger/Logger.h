#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <SPI.h>
#include <SD.h>

//Construct a logger, then call [logger].log() whenever you need to write something to SD.

class Logger{
    public:
        Logger(std::string logName);                    //Constructor
        void log(std::string field, std::string value); //Writes "[field]: [value]" to the log file
        void log(std::string field);                    //Writes "[field]" to the log file
        void log(std::string field, int value);          //Writes "[field]: [value]" to file, but string-ifys ints for you
        void log(std::string field, double value);        //Writes "[field]: [value]" to file, but string-ifys floats for you
        ~Logger();                                      //Destructor

    private:        
        File logFile;                                   //Log file

};

#endif