#include "Logger.h"


Logger::Logger(){
}

void Logger::begin(String logName = "log.txt")
{
    SD.begin(BUILTIN_SDCARD);                       //Initiate built-in SD card
    logFile = SD.open(logName.c_str(), FILE_WRITE); //Open log file

}

void Logger::log(String field, String value){
    logFile.write((field + ": " + value + "\n").c_str());                    //Write "[field]: [value] to file"
}

void Logger::log(String field){
    logFile.write((field + "\n").c_str());                                   //Write to "[field]" to file
}

void Logger::log(String field, int value){
    String buffer = String(value);
    logFile.write((field + ": " + buffer + "\n").c_str());    //Write "[field]: [value] to file"
}

void Logger::log(String field, double value){
    String buffer = String(value);
    logFile.write((field + ": " + buffer + "\n").c_str());    //Write "[field]: [value] to file"
}

// Logs an entire array to a particular field
void Logger::logStampedArray(String field, double times[], double values[], int length){ //Logs an array of timestamped values to file
    for(int i = 0; i < length; ++i){
        log(field, String(times[i]) + "," + String(values[i]));
    }
}

void Logger::close() {
    logFile.close();
}

void Logger::flush() {
    logFile.flush();
}

Logger::~Logger(){
    logFile.close();
}

