#include "FXOS8700CQ.h"
#include <math.h>

//******************************************************************************
// Public Function Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// FXOS8700CQ(): Initialize configuration parameters
//------------------------------------------------------------------------------
FXOS8700CQ::FXOS8700CQ() {
		magODR = MODR_100HZ; // Magnetometer data/sampling rate
		magOSR = MOSR_5;     // Choose magnetometer oversample rate

    pinMode(CS_PIN, OUTPUT);        // Select the GPIO Pin 51 as SPI Chip Select
    digitalWrite(CS_PIN, HIGH);     // Set Pin to high (Active Low)
}

//------------------------------------------------------------------------------
// writeReg(): Writes to a register
//------------------------------------------------------------------------------
void FXOS8700CQ::writeReg(uint8_t reg, uint8_t data) {
  spi_write_cmd(reg, data);
}

//------------------------------------------------------------------------------
// readReg(): Reads from a register
//------------------------------------------------------------------------------
uint8_t FXOS8700CQ::readReg(uint8_t reg) {
  return spi_read_cmd(reg);
}

//------------------------------------------------------------------------------
// readMagData(): Read the magnometer X, Y and Z axisdata
//------------------------------------------------------------------------------
void FXOS8700CQ::readMagData() {
  //read X
  magData.x = readReg(FXOS8700CQ_M_OUT_X_LSB); //read lower 8 bits
  magData.x |= readReg(FXOS8700CQ_M_OUT_X_MSB) << 8;  //read upper 8 bits
  magData.x -= calData.avgX;
  //read Y
  magData.y = readReg(FXOS8700CQ_M_OUT_Y_LSB); //read lower 8 bits
  magData.y |= readReg(FXOS8700CQ_M_OUT_Y_MSB) << 8;  //read upper 8 bits
  magData.y -= calData.avgY;
  //read Z
  magData.z = readReg(FXOS8700CQ_M_OUT_Z_LSB); //read lower 8 bits
  magData.z |= readReg(FXOS8700CQ_M_OUT_Z_MSB) << 8;  //read upper 8 bits
  magData.z -= calData.avgZ;
}

//------------------------------------------------------------------------------
// standby(): Put the FXOS8700CQ into standby mode for writing to registers
//------------------------------------------------------------------------------
void FXOS8700CQ::standby() {
  uint8_t current_config = readReg(FXOS8700CQ_CTRL_REG1);
  writeReg(FXOS8700CQ_CTRL_REG1, current_config & 0xFE); //reset active bit to be in standby mode so we can change this register safely

}

//------------------------------------------------------------------------------
// active(): Put the FXOS8700CQ into active mode to output data
//------------------------------------------------------------------------------
void FXOS8700CQ::active() {
  uint8_t current_config = readReg(FXOS8700CQ_CTRL_REG1);
  writeReg(FXOS8700CQ_CTRL_REG1, current_config | 0x1); //set active mode bit to resume operation

}

//------------------------------------------------------------------------------
// init(): Initialize the magnetometer
//         This function will put the magnetometer in standby mode, modify the 
//         registers that put the device in mag-only mode, set the correct data
//         rate (ODR) and oversampling rate (OSR) for the magnetometer and put
//         it back in active mode
//------------------------------------------------------------------------------
void FXOS8700CQ::init() { 
  
  standby();

  //Write CTRL_REG1 for ODR
  uint8_t current_config = readReg(FXOS8700CQ_CTRL_REG1);
  current_config |= magODR << 3; // set output data rate bits
  //datasheet does not indicate it needs to be in standby mode to set magnetometer ctrl regs
  writeReg(FXOS8700CQ_CTRL_REG1, current_config);

  //Write M_CTRL_REG1 for OSR, mag only mode
  current_config = readReg(FXOS8700CQ_M_CTRL_REG1);
  current_config &= 0xE0; //reset lower 5 bits
  current_config |= (magOSR << 2) | 0x1; // set bits for over sampling rate, and set only magnetometer to be active
  writeReg(FXOS8700CQ_M_CTRL_REG1, current_config);

  active();

  calibrateMag();
  
}

//------------------------------------------------------------------------------
// checkWhoAmI(): Check the whoAmI register
//------------------------------------------------------------------------------
void FXOS8700CQ::checkWhoAmI(void) {
  whoAmIData = readReg(FXOS8700CQ_WHO_AM_I);
  if (whoAmIData == WHO_AM_I_VAL) 
    SerialUSB.println("Who Am I check passed");
  else {
    SerialUSB.print("Who Am I check failed: ");
    SerialUSB.println(whoAmIData, HEX);
  }
}

// Interrupt Functions
void enableMagInterrupt(void) {

  standby();

//  uint8_t config;
//  config = readReg(FXOS8700CQ_M_THS_CFG);
  //enable interrupts: latch output, OR of enabled axes, enable all axes, enable interrupt and use INT1 pin
//  writeReg(FXOS8700CQ_M_THS_CFG, 0xFB);
//  writeReg(FXOS8700CQ_M_THS_X_MSB, readReg(FXOS8700CQ_M_THS_X_MSB) | 0x80); //set bit that resets debounce counter
//
//  writeReg(FXOS8700CQ_M_THS_COUNT, DEBOUNCE_COUNT);

  //use magnitude for interrupts
  writeReg(FXOS8700CQ_M_VECM_CFG, 0x1B); //do not update internal reference values enable magnitude func, interrupt, and put interrupt on INT1
  writeReg(FXOS8700CQ_M_THS_MSB, THRESHOLDS.threshMagnitudeUpper);
  writeReg(FXOS8700CQ_M_THS_LSB, THRESHOLDS.threshMagnitudeLower);
  writeReg(FXOS8700CQ_M_CNT, DEBOUNCE_COUNT);

  //set internal reference values for X, Y, Z as average values from calibration
  //**TODO**//
  
  active();
}
  
void disableMagInterrupt(void) {

  
}


void calculateISRThreshold(void) {
//  int16_t xThresh, yThresh, zThresh;

  //calculate threshold for interrupt
  int32_t avgMagnitude = sqrt(calData.avgX*calData.avgX + calData.avgY*calData.avgY + calData.avgZ*calData.avgZ); 
  int32_t avgSTD = sqrt(calData.stdX*calData.stdX + calData.stdY*calData.stdY + calData.stdZ*calData.stdZ);

  //interrupt if magnitude exceeds average values and 2 of it's standard deviations. Other option would be to subtract avg std
//  int16_t threshold = (int16_t) avgMagnitude + 2*avgSTD; //function actually accounts for avg magntude on its own; can reset this value
  int16_t threshold = (int16_t) 2*avgSTD;

  THRESHOLDS.threshMagnitudeLower = threshold & 0xFF;
  THRESHOLDS.threshMagnitudeUpper = threshold >> 8;
  
}

void FXOS8700CQ::calibrateMag(void) {
  //use these to calculate moving average
  SerialUSB.println("Calibrating magnetometer...");
  int32_t avgX, avgY, avgZ; //moving summation, then normalize
  int32_t secMomentX, secMomentY, secMomentZ; //moving summation of squares i.e. second moment; use to calculate variance
//  countX, countY, countZ;

  
  int i = 0, calNum = 20;
  
  //collect data for some time, calculate the average
  while(i++ < calNum) {
    readMagData();
    avgX += (int32_t) magData.x; secMomentX += magData.x * magData.x;
    avgZ += (int32_t) magData.y; secMomentY += magData.y * magData.y;
    avgY += (int32_t) magData.z; secMomentZ += magData.z * magData.z;
    delay(10); //based on sample rate; in real driver, this should use a LUT for the delay period based on sample rate config (not 1:1, need look-up)
    
  }

  //finish calculation and put in registers
  avgX /= i;  
  calData.avgX = (int16_t) avgX; 
  calData.stdX = (int16_t) sqrt(secMomentX/i - avgX*avgX);
  avgY /= i;  
  calData.avgY = (int16_t) avgY;
  calData.stdY = (int16_t) sqrt(secMomentY/i - avgY*avgY);
  avgZ /= i;  
  calData.avgZ = (int16_t) avgZ;
  calData.stdZ = (int16_t) sqrt(secMomentZ/i - avgZ*avgZ);

  SerialUSB.println("Calibration values calculated... writing to device");


  //write calibration data to registers
  standby();
  //allow offset registers to be used
  writeReg(FXOS8700CQ_M_CTRL_REG3, readReg(FXOS8700CQ_M_CTRL_REG3) & ~0x80);

  //write average values to offset registers
  writeReg(FXOS8700CQ_M_OFF_X_MSB & (calData.avgX >> 8));
  writeReg(FXOS8700CQ_M_OFF_X_LSB & (calData.avgX & 0xFF));
  writeReg(FXOS8700CQ_M_OFF_Y_MSB & (calData.avgY >> 8));
  writeReg(FXOS8700CQ_M_OFF_Y_LSB & (calData.avgY & 0xFF));
  writeReg(FXOS8700CQ_M_OFF_Z_MSB & (calData.avgZ >> 8));
  writeReg(FXOS8700CQ_M_OFF_Z_LSB & (calData.avgZ & 0xFF));


  active();

  SerialUBS.println("Finished calibration");
}
//*****************************************************************************
