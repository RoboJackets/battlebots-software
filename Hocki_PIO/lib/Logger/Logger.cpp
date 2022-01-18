#include "Logger.h"


Logger::Logger(){
}

void Logger::begin(String logName = "log.txt")
{
    SD.begin(BUILTIN_SDCARD);                       //Initiate built-in SD card
    logFile = SD.open(logName.c_str(), FILE_WRITE); //Open log file
    Serial.println("Opened Logfile");
    Serial.print(logName);
}

void Logger::log(String field, String value){
    logFile.write((field + ": " + value + "\n").c_str());                    //Write "[field]: [value] to file"
}

void Logger::log(String field){
    logFile.write((field).c_str());                                   //Write to "[field]" to file
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
void Logger::logStampedArray(String field, int times[], double values[], int length){ //Logs an array of timestamped values to file
    for(int i = 0; i < length; ++i){
        log(field, String(times[i]) + "," + String(values[i], 4));
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
void Logger::addToOutputString(float reading) {
    log((int)(reading*100.0));
    log(',');
}

void Logger::addLine(AccelReading val1, AccelReading val2, AccelReading val3, AccelReading val4)
{
    /*
    val1 = accel1.getXYZ();
    val2 = accel2.getXYZ();
    val3 = accel3.getXYZ();
    val4 = accel4.getXYZ();
    val1 = AccelReading(199.99, 199.99, 199.99, 1);
    val2 = AccelReading(9.99, 9.99, 9.99, 1);
    val3 = AccelReading(99.99, 99.99, 99.99, 1);
    val4 = AccelReading(9.0, 9.0, 9.0, 1);
    */

    addToOutputString(val1.x);
    addToOutputString(val1.y);
    addToOutputString(val1.z);

    addToOutputString(val2.x);
    addToOutputString(val2.y);
    addToOutputString(val2.z);

    addToOutputString(val3.x);
    addToOutputString(val3.y);
    addToOutputString(val3.z);

    addToOutputString(val4.x);
    addToOutputString(val4.y);
    addToOutputString(val4.z);

    log("\n");
    lineCount ++;
    if(lineCount% 14  ==  0)
    {
        flush();
        Serial.println("Flushed");
    }
    Serial.println("Added");
}