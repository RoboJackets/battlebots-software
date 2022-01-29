#include "Logger.h"


Logger::Logger() {
}

void Logger::begin(String logName = "log.txt") {
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

void Logger::addLine(AccelReading val1, AccelReading val2, AccelReading val3, AccelReading val4, float vavg, float pos)
{
    if(!whichLog)
    {
        logs1[logIdx++] = val1;
        logs1[logIdx++] = val2;
        logs1[logIdx++] = val3;
        logs1[logIdx++] = val4;
        rotlogs1[rlogIdx++] = vavg;
        rotlogs1[rlogIdx++] = pos;
    }
    else
    {
        logs2[logIdx++] = val1;
        logs2[logIdx++] = val2;
        logs2[logIdx++] = val3;
        logs2[logIdx++] = val4;
        rotlogs2[rlogIdx++] = vavg;
        rotlogs2[rlogIdx++] = pos;
    }

    if(logIdx == LOG_LENGTH) 
    {
        // Signal to main loop to dump file
        dumpFlag = true;
        whichLog = !whichLog;
        logIdx = 0;
        rlogIdx = 0;
    }
    lineCount ++;
}

void Logger::dump()
{
    int j = 0;
    for(int i = 0; i < LOG_LENGTH; i+= 4)
    {
        if(whichLog)
        {
            addToOutputString(logs1[i].x);
            addToOutputString(logs1[i].y);
            addToOutputString(logs1[i].z);

            addToOutputString(logs1[i+1].x);
            addToOutputString(logs1[i+1].y);
            addToOutputString(logs1[i+1].z);

            addToOutputString(logs1[i+2].x);
            addToOutputString(logs1[i+2].y);
            addToOutputString(logs1[i+2].z);

            addToOutputString(logs1[i+3].x);
            addToOutputString(logs1[i+3].y);
            addToOutputString(logs1[i+3].z);

            addToOutputString(rotlogs1[j]);
            addToOutputString(rotlogs1[j+1]);

            log("\n");
        }
        else
        {
            addToOutputString(logs2[i].x);
            addToOutputString(logs2[i].y);
            addToOutputString(logs2[i].z);

            addToOutputString(logs2[i+1].x);
            addToOutputString(logs2[i+1].y);
            addToOutputString(logs2[i+1].z);

            addToOutputString(logs2[i+2].x);
            addToOutputString(logs2[i+2].y);
            addToOutputString(logs2[i+2].z);

            addToOutputString(logs2[i+3].x);
            addToOutputString(logs2[i+3].y);
            addToOutputString(logs2[i+3].z);

            addToOutputString(rotlogs2[j]);
            addToOutputString(rotlogs2[j+1]);

            log("\n");

        }
        j += 2;
    }
    flush();
    dumpFlag = false;
}