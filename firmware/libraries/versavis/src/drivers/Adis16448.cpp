////////////////////////////////////////////////////////////////////////////////
//  July 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////
//  Adis16448.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides all the functions necessary to interface the Adis16448
//  IMU with an 8-Bit Atmel-based Arduino development board. Functions for SPI
//  configuration, reads and writes, and scaling are included. This library may
//  be used for the entire ADIS164XX family of devices with some modification.
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free
//  Software Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for
//  more details.
//
//  You should have received a copy of the GNU Lesser Public License along with
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

#include "drivers/Adis16448.h"
#include <project_DMA_SPI.h>

DMA_SPI DMASPI(SERCOM4, 0, 1, SERCOM4_DMAC_ID_TX, SERCOM4_DMAC_ID_RX);

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - Data ready pin
////////////////////////////////////////////////////////////////////////////
Adis16448::Adis16448(uint8_t chip_select) {
  _CS = chip_select;

  // Set default pin states.
  pinMode(_CS, OUTPUT);    // Set CS pin to be an output
  digitalWrite(_CS, HIGH); // Initialize CS pin to be high
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
Adis16448::~Adis16448() {
  // Close SPI bus
  DMASPI.disable();
  SPI.end();
}

void Adis16448::setDefault() {
  // Set all registers that are stored in flash backup to default values.
  regWrite(XGYRO_OFF, 0x0);
  regWrite(YGYRO_OFF, 0x0);
  regWrite(ZGYRO_OFF, 0x0);
  regWrite(XACCL_OFF, 0x0);
  regWrite(YACCL_OFF, 0x0);
  regWrite(ZACCL_OFF, 0x0);
  regWrite(XMAGN_HIC, 0x0);
  regWrite(YMAGN_HIC, 0x0);
  regWrite(ZMAGN_HIC, 0x0);
  regWrite(XMAGN_SIC, 0x0);
  regWrite(YMAGN_SIC, 0x0);
  regWrite(ZMAGN_SIC, 0x0);
  regWrite(MSC_CTRL, 0x0006);
  regWrite(SMPL_PRD, 0x0001);
  regWrite(SENS_AVG, 0x0402);
  regWrite(ALM_MAG1, 0x0);
  regWrite(ALM_MAG2, 0x0);
  regWrite(ALM_SMPL1, 0x0);
  regWrite(ALM_SMPL2, 0x0);
  regWrite(ALM_CTRL, 0x0);
}

void Adis16448::setup() {
  // Configure SPI.
  SPI.begin(); // Initialize SPI bus
  setDefault();

  // Factory calibration.
  regWrite(GLOB_CMD, 0b10);

  // Software reset.
  regWrite(GLOB_CMD, 0b10000000);
  delay(300);

  regWrite(MSC_CTRL, 0x16); // DR with active, HIGH on DIO1, activate CRC
  delay(20);
  regWrite(GPIO_CTRL, 0x202); // DIO2 output (LED).
  delay(20);
  // regWrite(SMPL_PRD, 0x0); // external clock, no averaging.
  delay(20);
  regWrite(SENS_AVG,
           0x400); // +-1000 dps range, B=0 for no bartlett window.
  delay(20);

  DMASPI.init();
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
////////////////////////////////////////////////////////////////////////////
void Adis16448::beginTransaction() {
  // max. 1 MHz burst read, MSBFIRST, SPI_MODE3 according to datasheet.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(_CS, LOW); // Set CS low to enable device
}

void Adis16448::endTransaction() {
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  SPI.endTransaction();
}

////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////
int16_t Adis16448::regRead(uint8_t regAddr) {
  // Read registers using SPI

  // Write register address to be read
  beginTransaction();
  SPI.transfer(regAddr); // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement
  endTransaction();

  // Read data from requested register
  beginTransaction();
  uint8_t _msbData =
      SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData =
      SPI.transfer(0x00); // Send (0x00) and place lower byte into variable
  endTransaction();

  int16_t _dataOut =
      (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return (_dataOut);
}

////////////////////////////////////////////////////////////////////////////////
// Reads all gyro, accel, and magnetometer registers
////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (pointer) array of signed 16 bit 2's complement numbers
////////////////////////////////////////////////////////////////////////////////
int16_t *Adis16448::sensorRead() {
  // Read registers using SPI
  // Initialize sensor data arrays and stall time variable
  uint8_t sensorData[22];
  static int16_t joinedData[11];

  // Write each requested register address and read back it's data
  beginTransaction();
  sensorData[0] = SPI.transfer(DIAG_STAT); // Initial SPI read. Returned data
                                           // for this transfer is invalid
  sensorData[1] = SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16
                                      // bit transaction requirement
  joinedData[0] = (sensorData[0] << 8) |
                  (sensorData[1] & 0xFF); // Concatenate two bytes into word
  endTransaction();
  beginTransaction();
  sensorData[2] = SPI.transfer(
      XGYRO_OUT); // Write next address to device and read upper byte
  sensorData[3] = SPI.transfer(0x00); // Read lower byte
  joinedData[1] = (sensorData[2] << 8) |
                  (sensorData[3] & 0xFF); // Concatenate two bytes into word
  endTransaction();
  beginTransaction();
  sensorData[4] = SPI.transfer(YGYRO_OUT);
  sensorData[5] = SPI.transfer(0x00);
  joinedData[2] = (sensorData[4] << 8) | (sensorData[5] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[6] = SPI.transfer(ZGYRO_OUT);
  sensorData[7] = SPI.transfer(0x00);
  joinedData[3] = (sensorData[6] << 8) | (sensorData[7] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[8] = SPI.transfer(XACCL_OUT);
  sensorData[9] = SPI.transfer(0x00);
  joinedData[4] = (sensorData[8] << 8) | (sensorData[9] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[10] = SPI.transfer(YACCL_OUT);
  sensorData[11] = SPI.transfer(0x00);
  joinedData[5] = (sensorData[10] << 8) | (sensorData[11] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[12] = SPI.transfer(ZACCL_OUT);
  sensorData[13] = SPI.transfer(0x00);
  joinedData[6] = (sensorData[12] << 8) | (sensorData[13] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[14] = SPI.transfer(XMAGN_OUT);
  sensorData[15] = SPI.transfer(0x00);
  joinedData[7] = (sensorData[14] << 8) | (sensorData[15] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[16] = SPI.transfer(YMAGN_OUT);
  sensorData[17] = SPI.transfer(0x00);
  joinedData[8] = (sensorData[16] << 8) | (sensorData[17] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[18] = SPI.transfer(ZMAGN_OUT);
  sensorData[19] = SPI.transfer(0x00);
  joinedData[9] = (sensorData[18] << 8) | (sensorData[19] & 0xFF);
  endTransaction();
  beginTransaction();
  sensorData[20] =
      SPI.transfer(FLASH_CNT); // Final transfer. Data after this invalid
  sensorData[21] = SPI.transfer(0x00);
  joinedData[10] = (sensorData[20] << 8) | (sensorData[21] & 0xFF);
  endTransaction();

  return (joinedData); // Return pointer with data
}

////////////////////////////////////////////////////////////////////////////////
// Reads all gyro, accel, magn, baro, and tmp registers in one instance (faster)
////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (pointer) array of signed 16 bit 2's complement numbers
////////////////////////////////////////////////////////////////////////////////
int16_t *Adis16448::sensorReadAll() {
  // Read registers using SPI
  // Initialize sensor data arrays.
  uint8_t sensorData[14];
  // Write each requested register address and read back it's data
  beginTransaction();
  SPI.transfer(GLOB_CMD); // Initial SPI read. Returned data for this transfer
                          // is invalid
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement

  // DIAG_STAT
  sensorData[0] =
      SPI.transfer(0x00); // Write next address to device and read upper byte
  sensorData[1] = SPI.transfer(0x00); // Read lower byte
  // XGYRO_OUT
  sensorData[2] = SPI.transfer(0x00);
  sensorData[3] = SPI.transfer(0x00);
  // YGYRO_OUT
  sensorData[4] = SPI.transfer(0x00);
  sensorData[5] = SPI.transfer(0x00);
  // ZGYRO_OUT
  sensorData[6] = SPI.transfer(0x00);
  sensorData[7] = SPI.transfer(0x00);
  // XACCL_OUT
  sensorData[8] = SPI.transfer(0x00);
  sensorData[9] = SPI.transfer(0x00);
  // YACCL_OUT
  sensorData[10] = SPI.transfer(0x00);
  sensorData[11] = SPI.transfer(0x00);
  // ZACCL_OUT
  sensorData[12] = SPI.transfer(0x00);
  sensorData[13] = SPI.transfer(0x00);
  // XMAGN_OUT
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  // YMAGN_OUT
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  // ZMAGN_OUT
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  // BARO_OUT
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  // TEMP_OUT
  SPI.transfer(0x00); // Final transfer. Data after this invalid
  SPI.transfer(0x00);

  endTransaction();

  // Concatenate two bytes into word
  static int16_t joinedData[7];
  joinedData[0] = (sensorData[0] << 8) | (sensorData[1] & 0xFF);   // DIAG_STAT
  joinedData[1] = (sensorData[2] << 8) | (sensorData[3] & 0xFF);   // XGYRO_OUT
  joinedData[2] = (sensorData[4] << 8) | (sensorData[5] & 0xFF);   // YGYRO_OUT
  joinedData[3] = (sensorData[6] << 8) | (sensorData[7] & 0xFF);   // ZGYRO_OUT
  joinedData[4] = (sensorData[8] << 8) | (sensorData[9] & 0xFF);   // XACCL_OUT
  joinedData[5] = (sensorData[10] << 8) | (sensorData[11] & 0xFF); // YACCL_OUT
  joinedData[6] = (sensorData[12] << 8) | (sensorData[13] & 0xFF); // ZACCL_OUT

  return (joinedData); // Return pointer with data
}

////////////////////////////////////////////////////////////////////////////////
// Reads all gyro, accel, magn, baro, and tmp registers in one instance (faster)
// including
// CRC-16 Checksum.
////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (pointer) array of signed 16 bit 2's complement numbers
////////////////////////////////////////////////////////////////////////////////
int16_t *Adis16448::sensorReadAllCRC(uint8_t *tx, uint8_t *rx, size_t n) {
  // Read registers using SPI
  memset(tx, 0, n);
  tx[0] = GLOB_CMD; // Initial command.
  // Burst read all bytes.
  uint8_t tx_tmp[1];
  uint8_t rx_tmp[1];
  beginTransaction();
  DMASPI.transfer(tx, rx, n);
  while (!DMASPI.transferDone())
    ;
  DMASPI.disable();
  endTransaction();

  // Copy and merge data.
  static int16_t joinedData[13];
  uint8_t *p = rx + 2; // Set pointer to third transfered byte.
  for (uint8_t i = 0; i < 13; ++i) {
    joinedData[i] = (*(p++) << 8);    // MSB
    joinedData[i] |= (*(p++) & 0xFF); // LSB
  }

  return (joinedData); // Return pointer with data
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int Adis16448::regWrite(uint8_t regAddr, int16_t regData) {
  // Write register address and data
  uint16_t addr =
      (((regAddr & 0x7F) | 0x80)
       << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord =
      (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord =
      ((addr | 0x100) |
       ((regData >> 8) &
        0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  beginTransaction();
  SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord);  // Write low byte from high word to SPI bus
  endTransaction();

  // Write lowWord to SPI bus
  beginTransaction();            // Set CS low to enable device
  SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord);  // Write low byte from low word to SPI bus
  endTransaction();              // Set CS high to disable device

  return (1);
}

////////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on burst data.
// Returns the calculated checksum.
////////////////////////////////////////////////////////////////////////////////
// *burstArray - array of burst data
// return - (int16_t) signed calculated checksum
////////////////////////////////////////////////////////////////////////////////
int16_t Adis16448::checksum(int16_t *burstArray) {
  unsigned char i;        // Tracks each burstArray word
  unsigned int data;      // Holds the lower/Upper byte for CRC computation
  unsigned int crc;       // Holds the CRC value
  unsigned int lowerByte; // Lower Byte of burstArray word
  unsigned int upperByte; // Upper Byte of burstArray word
  unsigned int POLY;      // Divisor used during CRC computation
  POLY = 0x1021;          // Define divisor
  crc = 0xFFFF;           // Set CRC to ‐1 prior to beginning CRC computation
  // Compute CRC on burst data starting from XGYRO_OUT and ending with TEMP_OUT.
  // Start with the lower byte and then the upper byte of each word.
  // i.e. Compute XGYRO_OUT_LSB CRC first and then compute XGYRO_OUT_MSB CRC.
  for (i = 1; i < 12; i++) {
    upperByte = (burstArray[i] >> 8) & 0xFF;
    lowerByte = (burstArray[i] & 0xFF);
    updateCRC(&crc, &lowerByte, POLY);
    updateCRC(&crc, &upperByte, POLY);
  }
  crc = ~crc; // Compute complement of CRC
  data = crc;
  crc = (crc << 8) |
        (data >> 8 & 0xFF); // Perform byte swap prior to returning CRC
  return crc;
}

void Adis16448::updateCRC(unsigned int *crc, unsigned int *data,
                          const unsigned int &POLY) {
  unsigned char ii; // Counter for each bit of the current burstArray word
  for (ii = 0; ii < 8; ii++, *data >>= 1) {
    if ((*crc & 0x0001) ^ (*data & 0x0001))
      *crc = (*crc >> 1) ^ POLY;
    else
      *crc >>= 1;
  }
}
