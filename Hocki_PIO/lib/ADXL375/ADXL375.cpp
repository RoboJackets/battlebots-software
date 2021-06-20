#include "Arduino.h"
#include "ADXL375.h"
#include "SPI.h"

ADXL375::ADXL375(uint8_t cs_pin, uint8_t int1_pin, uint8_t int2_pin, int spi_rate)
{
    _cs_pin = cs_pin;
    _int1_pin = int1_pin;
    _int2_pin = int2_pin; 
    _spi_settings= new SPISettings(spi_rate, MSBFIRST, SPI_MODE3);
}

ADXL375::~ADXL375() {
    free(_spi_settings);
}

void ADXL375::init()
{
  
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);

  // set the data sampling rate to 3200Hz
  setDataRate(0b00001111);
}

void ADXL375::startMeasuring()
{
  writeRegister(ADXL375_REG_DATA_FORMAT, 0x0B);
  writeRegister(ADXL375_REG_POWER_CTL, 0x08);
}

void ADXL375::setDataRate(uint8_t rate)
{
  writeRegister(ADXL375_REG_BW_RATE, rate);
}

AccelReading ADXL375::getXYZ()
{
  uint8_t data[6];
  _multiReadRegister(ADXL375_REG_DATAX0, data, 6);

  AccelReading xyz;
  xyz.init(
    data[0] | data[1]<<8,
    data[2] | data[3]<<8,
    data[4] | data[5]<<8,
    ADXL375_XYZ_READ_SCALE_FACTOR
  );
  
  return xyz;
}

uint8_t ADXL375::readRegister(uint8_t regAddress)
{
  uint8_t data[1];
  _multiReadRegister(regAddress, data, 1);
  return data[0];
}

// input is in g
void ADXL375::setShockThreshold(uint8_t shockThreshold)
{
  uint8_t scaledValue = shockThreshold * 1000 / ADXL375_THRESH_SHOCK_SCALE;
  writeRegister(ADXL375_REG_THRESH_SHOCK, scaledValue);
}

// returns the number of items in the FIFO buffer
uint8_t ADXL375::getFIFOBufferSize()
{
  return readRegister(ADXL375_REG_FIFO_STATUS) & 0b00111111;
}

void ADXL375::startContinuousOperation(float ofsx, float ofxy, float ofsz) {
    
    setFIFOMode(ADXL375_FIFO_MODE_BYPASS);

    // DATA_READY -> INT 1
    // OVERRUN -> INT 2
    writeRegister(ADXL375_REG_INT_ENABLE, 0b10000001);
    writeRegister(ADXL375_REG_INT_MAP, 0b00000001);

    int8_t ofsx_i = (int8) ((ofsx / 9.81) / 0.197); 
    int8_t ofsy_i = (int8) ((ofsy / 9.81) / 0.197); 
    int8_t ofsz_i = (int8) ((ofsz / 9.81) / 0.197);

    writeRegister(ADXL375_REG_OFSX, (uint8_t) ofsx_i);
    writeRegister(ADXL375_REG_OFSY, (uint8_t) ofsy_i);
    writeRegister(ADXL375_REG_OFSZ, (uint8_t) ofsz_i);

    startMeasuring();
}

void ADXL375::activateSelfTest() {
    writeRegister(ADXL375_REG_DATA_FORMAT, 0x8B); // set D7 high   
}

void ADXL375::deactivateSelfTest() {
    writeRegister(ADXL375_REG_DATA_FORMAT, 0x0B); // set D7 low   
}

void ADXL375::startShockDetection()
{
  setShockAxes(true,true,true);

  // shock duration to test for 625 μs/LSB
  writeRegister(ADXL375_REG_DUR, 2);

  // reset then enable FIFO trigger mode
  setFIFOMode(ADXL375_FIFO_MODE_BYPASS);
  setFIFOMode(ADXL375_FIFO_MODE_TRIGGER, ADXL375_TRIGGER_INT1_PIN, 16);

  // set the interrupt bit for shock detection
  uint8_t intEnable = readRegister(ADXL375_REG_INT_ENABLE);
  writeRegister(ADXL375_REG_INT_ENABLE, intEnable | 0b01000000);

  startMeasuring();
}

void ADXL375::stop() {
    writeRegister(ADXL375_REG_THRESH_SHOCK, 0x00);
    writeRegister(ADXL375_REG_OFSX, 0x00);
    writeRegister(ADXL375_REG_OFSY, 0x00);
    writeRegister(ADXL375_REG_OFSZ, 0x00);
    writeRegister(ADXL375_REG_DUR, 0x00);
    writeRegister(ADXL375_REG_LATENT, 0x00);
    writeRegister(ADXL375_REG_WINDOW, 0x00);
    writeRegister(ADXL375_REG_THRESH_ACT, 0x00);
    writeRegister(ADXL375_REG_THRESH_INACT, 0x00);
    writeRegister(ADXL375_REG_TIME_INACT, 0x00);
    writeRegister(ADXL375_REG_ACT_INACT_CTL, 0x00);
    writeRegister(ADXL375_REG_SHOCK_AXES, 0x00);
    writeRegister(ADXL375_REG_BW_RATE, 0x0A);
    writeRegister(ADXL375_REG_POWER_CTL, 0x00);
    writeRegister(ADXL375_REG_INT_ENABLE, 0x00);
    writeRegister(ADXL375_REG_INT_MAP, 0x00);
    writeRegister(ADXL375_REG_DATA_FORMAT, 0x00);
    writeRegister(ADXL375_REG_FIFO_CTL, 0x00);
}

void ADXL375::setShockAxes(bool x, bool y, bool z)
{
  uint8_t shockAxesData = 0b00000000;
  if(x) shockAxesData |= 0b100;
  if(y) shockAxesData |= 0b010;
  if(z) shockAxesData |= 0b001;

  // the axes we want to detect shocks on
  writeRegister(ADXL375_REG_SHOCK_AXES, shockAxesData);
}

void ADXL375::setFIFOMode(uint8_t mode, uint8_t pin, uint8_t samples)
{
  writeRegister(ADXL375_REG_FIFO_CTL, mode<<6 | pin<<5 | samples);
}

void ADXL375::_multiReadRegister(uint8_t regAddress, uint8_t values[], uint8_t numberOfBytes)
{
  // Since we're performing a read operation, the most significant bit of the register address should be set.
  regAddress |= 0x80;

  // set the multi-read byte if required
  if (numberOfBytes > 1) {
    regAddress |= 0x40;
  }
  
  SPI.beginTransaction(*_spi_settings);
  // set the Chip select pin low to start an SPI packet.
  digitalWrite(_cs_pin, LOW);
  // transfer the starting register address that needs to be read.
  SPI.transfer(regAddress);
  
  // read the data
  for(int i=0; i<numberOfBytes; i++){
    values[i] = SPI.transfer(0x00);
  }

  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}

void ADXL375::writeRegister(uint8_t regAddress, uint8_t value)
{
  SPI.beginTransaction(*_spi_settings);
    //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(_cs_pin, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(regAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}

