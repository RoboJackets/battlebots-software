#include "Logger.h"


Logger::Logger(std::string logName = "log.txt"){

                           
    SD.begin(BUILTIN_SDCARD);                       //Initiate built-in SD card
    logFile = SD.open(logName.c_str(), FILE_WRITE); //Open log file
    
}

void Logger::log(std::string field, std::string value){
    logFile.write((field + ": " + value + "\n").c_str());                    //Write "[field]: [value] to file"
}

void Logger::log(std::string field){
    logFile.write((field + "\n").c_str());                                   //Write to "[field]" to file
}

void Logger::log(std::string field, int value){
    char buffer[10];
    sprintf(buffer, "%d", value);
    logFile.write((field + ": " + buffer + "\n").c_str());    //Write "[field]: [value] to file"
}

void Logger::log(std::string field, double value){
    char buffer[10];
    sprintf(buffer, "%f", value);
    logFile.write((field + ": " + buffer + "\n").c_str());    //Write "[field]: [value] to file"
}

Logger::~Logger(){
    logFile.close();
}

