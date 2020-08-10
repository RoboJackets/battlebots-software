#include "Telemetry.h"

Telemetry::Telemetry() {
    
    // start SD
    if(!SC.begin(BUILTIN_SDCARD)) {
        Serial.println("Telemetry Init Failed: SD Card Unavailable");
    }
    else {
        Serial.println("Telemetry Init Successful");
    }

    // determine what base directory to use
    File base = SD.open("/");
    File entry;
    String fname;
    int dirN = 0, tmp;
    while(true) {
        entry = base.openNextFile();
        if(!entry) {
            break;
        }
        if(!entry.isDirectory()) {
            continue;
        }
        fname = entry.name();
        if(!fname.substring(0, 5).equals("TELEM")) {
            continue;
        }
        tmp = fname.substring(5, 8).toInt();
        dirN = tmp > dirN ? tmp : dirN;
    }
    dirN += 1;
    dir = "TELEM" + String(dirN);
    SD.mkdir(dir);
    Serial.println("Making root dir: " + dir);
}

Telemetry::~Telemetry() {
    delete dir;
}

uint8_t Telemetry::writeData(unsigned long time_us, Matrix<float, -1, 1> data, uint8_t telem_type) {
    String fname = "/" + dir + "/";
    switch(telem_type) {
        case ACCX_TELEM:
            fname += "ACCX.csv";
            break;
        case ACCY_TELEM:
            fname += "ACCY.csv";
            break;
        case ACCZ_TELEM:
            fname += "ACCZ.csv";
            break;
        case ACCC_TELEM:
            fname += "ACCC.csv";
            break;
        case MAGA_TELEM:
            fname += "MAGA.csv";
            break;
        case MAGB_TELEM:
            fname += "MAGB.csv";
            break;
        case IR_TELEM:
            fname += "IR.csv";
            break;
        case MOTOR_TELEM:
            fname += "MOTOR.csv";
            break;
        case MISC_TELEM:
            fname += "MISC.csv"
            break;
        default:
            Serial.println("Invalid telem_type");
            return 0;
    }
    File telemFile = SD.open(fname, FILE_WRITE);
    if(telemFile) {}
    else {
        Serial.println("Cannot open file " + fname);
        return 0;
    }
    uint8_t datapoints = data.rows();
    if(datapoints <= 0) {
        String.println("Cannot write no data to csv");
        return 0;
    }
    telemFile.print(String(time_us));
    telemFile.print(",");
    for(uint8_t k = 0; k < datapoints-1; k++) {
        telemFile.print(String(data(k, 0)));
        telemFile.print(",");
    }
    telemFile.println(String(data(datapoints-1, 0)));
    telemFile.close();
    return 1; // success!
}

uint8_t Telemetry::writeMsg(unsigned long time_us, String msg, uint8_t telem_type) {
    String fname = "/" + dir + "/";
    switch(telem_type) {
        case ACCX_TELEM:
            fname += "ACCX.csv";
            break;
        case ACCY_TELEM:
            fname += "ACCY.csv";
            break;
        case ACCZ_TELEM:
            fname += "ACCZ.csv";
            break;
        case ACCC_TELEM:
            fname += "ACCC.csv";
            break;
        case MAGA_TELEM:
            fname += "MAGA.csv";
            break;
        case MAGB_TELEM:
            fname += "MAGB.csv";
            break;
        case IR_TELEM:
            fname += "IR.csv";
            break;
        case MOTOR_TELEM:
            fname += "MOTOR.csv";
            break;
        case MISC_TELEM:
            fname += "MISC.csv"
            break;
        default:
            Serial.println("Invalid telem_type");
            return 0;
    }
    File telemFile = SD.open(fname, FILE_WRITE);
    if(telemFile) {}
    else {
        Serial.println("Cannot open file " + fname);
        return 0;
    }
    uint8_t datapoints = data.rows();
    if(datapoints <= 0) {
        String.println("Cannot write no data to csv");
        return 0;
    }
    telemFile.print(String(time_us));
    telemFile.print(",");
    telemFile.println(msg);
    telemFile.close();
    return 1; // success!
}

