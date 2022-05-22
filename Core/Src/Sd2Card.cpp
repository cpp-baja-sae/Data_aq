/* Arduino Sd2Card Library
 Copyright (C) 2009 by William Greiman

 This file is part of the Arduino Sd2Card Library

 This Library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This Library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with the Arduino Sd2Card Library.  If not, see
 <http://www.gnu.org/licenses/>.
 */
#define USE_SPI_LIB
#include "Sd2Card.h"
//------------------------------------------------------------------------------
int millis() {
	return 0;
}
//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
	// Our weird card requires this.
	chipSelectHigh();
	spiSend(0XFF);
	chipSelectLow();

	// end read if in partialBlockRead mode
	readEnd();

	// wait up to 300 ms if busy
//  waitNotBusy(300);

// send command

	// send CRC
	uint8_t crc = 0XFF;
	if (cmd == CMD0) {
		crc = 0X95;  // correct crc for CMD0 with arg 0
	}
	if (cmd == 1) {
		crc = 0XF9;  // correct crc for CMD1 with arg 0
	}
	if (cmd == CMD8) {
		crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
	}
	if (cmd == CMD55) {
		crc = 0X65;  // correct crc for CMD55 with arg 0
	}
	if (cmd == ACMD41) {
		if (arg == 0x40000000) {
			crc = 0x77;
		} else {
			crc = 0xe5;
		}
	}

	uint8_t data[6] = {0};
	data[0] = cmd | 0x40;
	data[1] = arg >> 24;
	data[2] = arg >> 16;
	data[3] = arg >> 8;
	data[4] = arg;
	data[5] = crc;
	spiSendMulti(data, 6);

	// wait for response
	for (uint8_t i = 0; ((status_ = spiRecv()) & 0X80) && i != 0XFF; i++)
		;
	return status_;
}
//------------------------------------------------------------------------------
/**
 Determine the size of an SD flash memory card.

 \return The number of 512 byte data blocks in the card
 or zero if an error occurs.
 */
uint32_t Sd2Card::cardSize(void) {
	csd_t csd;
	if (!readCSD(&csd)) {
		return 0;
	}
	if (csd.v1.csd_ver == 0) {
		uint8_t read_bl_len = csd.v1.read_bl_len;
		uint16_t c_size = (csd.v1.c_size_high << 10) | (csd.v1.c_size_mid << 2)
				| csd.v1.c_size_low;
		uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
				| csd.v1.c_size_mult_low;
		return (uint32_t) (c_size + 1) << (c_size_mult + read_bl_len - 7);
	} else if (csd.v2.csd_ver == 1) {
		uint32_t c_size = ((uint32_t) csd.v2.c_size_high << 16)
				| (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
		return (c_size + 1) << 10;
	} else {
		error(SD_CARD_ERROR_BAD_CSD);
		return 0;
	}
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/** Erase a range of blocks.

 \param[in] firstBlock The address of the first block in the range.
 \param[in] lastBlock The address of the last block in the range.

 \note This function requests the SD card to do a flash erase for a
 range of blocks.  The data on the card after an erase operation is
 either 0 or 1, depends on the card vendor.  The card must support
 single block erase.

 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
	if (!eraseSingleBlockEnable()) {
		error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
		goto fail;
	}
	if (type_ != SD_CARD_TYPE_SDHC) {
		firstBlock <<= 9;
		lastBlock <<= 9;
	}
	if (cardCommand(CMD32, firstBlock) || cardCommand(CMD33, lastBlock)
			|| cardCommand(CMD38, 0)) {
		error(SD_CARD_ERROR_ERASE);
		goto fail;
	}
	if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
		error(SD_CARD_ERROR_ERASE_TIMEOUT);
		goto fail;
	}
	chipSelectHigh();
	return true;

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/** Determine if card supports single block erase.

 \return The value one, true, is returned if single block erase is supported.
 The value zero, false, is returned if single block erase is not supported.
 */
uint8_t Sd2Card::eraseSingleBlockEnable(void) {
	csd_t csd;
	return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}
//------------------------------------------------------------------------------
/**
 Initialize an SD flash memory card.

 \param[in] sckRateID SPI clock rate selector. See setSckRate().
 \param[in] chipSelectPin SD chip select pin number.

 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.  The reason for failure
 can be determined by calling errorCode() and errorData().
 */
uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
	errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
	chipSelectPin_ = chipSelectPin;
	// 16-bit init start time allows over a minute
	unsigned int t0 = millis();
	uint32_t arg;

	chipSelectHigh();
	uint8_t data[20] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	spiSendMulti(data, 20);
	chipSelectLow();

	// command to go idle in SPI mode
	while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
		unsigned int d = millis() - t0;
		if (d > SD_INIT_TIMEOUT) {
			error(SD_CARD_ERROR_CMD0);
			goto fail;
		}
	}
	// check SD version
	if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
		type(SD_CARD_TYPE_SD1);
	} else {
		// only need last byte of r7 response
		for (uint8_t i = 0; i < 4; i++) {
			status_ = spiRecv();
		}
		if (status_ != 0XAA) {
			error(SD_CARD_ERROR_CMD8);
			goto fail;
		}
		type(SD_CARD_TYPE_SD2);
	}
	// initialize card and send host supports SDHC if SD2
	arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

	while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
		chipSelectHigh();
		for (uint8_t i = 0; i < 20; i++) {
			spiSend(0XFF);
		}
		chipSelectLow();
		// check for timeout
		unsigned int d = millis() - t0;
		if (d > SD_INIT_TIMEOUT) {
			error(SD_CARD_ERROR_ACMD41);
			goto fail;
		}
	}
	// if SD2 read OCR register to check for SDHC card
	if (type() == SD_CARD_TYPE_SD2) {
		if (cardCommand(CMD58, 0)) {
			error(SD_CARD_ERROR_CMD58);
			goto fail;
		}
		if ((spiRecv() & 0XC0) == 0XC0) {
			type(SD_CARD_TYPE_SDHC);
		}
		// discard rest of ocr - contains allowed voltage range
		for (uint8_t i = 0; i < 3; i++) {
			spiRecv();
		}
	}
	chipSelectHigh();

#ifndef SOFTWARE_SPI
	return setSckRate(sckRateID);
#else  // SOFTWARE_SPI
  return true;
  #endif  // SOFTWARE_SPI

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/**
 Enable or disable partial block reads.

 Enabling partial block reads improves performance by allowing a block
 to be read over the SPI bus as several sub-blocks.  Errors may occur
 if the time between reads is too long since the SD card may timeout.
 The SPI SS line will be held low until the entire block is read or
 readEnd() is called.

 Use this for applications like the Adafruit Wave Shield.

 \param[in] value The value TRUE (non-zero) or FALSE (zero).)
 */
void Sd2Card::partialBlockRead(uint8_t value) {
	readEnd();
	partialBlockRead_ = value;
}
//------------------------------------------------------------------------------
/**
 Read a 512 byte block from an SD card device.

 \param[in] block Logical block to be read.
 \param[out] dst Pointer to the location that will receive the data.

 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readBlock(uint32_t block, uint8_t *dst) {
	return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 Read part of a 512 byte block from an SD card.

 \param[in] block Logical block to be read.
 \param[in] offset Number of bytes to skip at start of block
 \param[out] dst Pointer to the location that will receive the data.
 \param[in] count Number of bytes to read
 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readData(uint32_t block, uint16_t offset, uint16_t count,
		uint8_t *dst) {
	if (count == 0) {
		return true;
	}
	if ((count + offset) > 512) {
		goto fail;
	}
	if (!inBlock_ || block != block_ || offset < offset_) {
		block_ = block;
		// use address if not SDHC card
		if (type() != SD_CARD_TYPE_SDHC) {
			block <<= 9;
		}
		if (cardCommand(CMD17, block)) {
			error(SD_CARD_ERROR_CMD17);
			goto fail;
		}
		if (!waitStartBlock()) {
			goto fail;
		}
		offset_ = 0;
		inBlock_ = 1;
	}

#ifdef OPTIMIZE_HARDWARE_SPI
  // start first spi transfer
  SPDR = 0XFF;

  // skip data before offset
  for (; offset_ < offset; offset_++) {
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = 0XFF;
  }
  // transfer data
  n = count - 1;
  for (uint16_t i = 0; i < n; i++) {
    while (!(SPSR & (1 << SPIF)))
      ;
    dst[i] = SPDR;
    SPDR = 0XFF;
  }
  // wait for last byte
  while (!(SPSR & (1 << SPIF)))
    ;
  dst[n] = SPDR;

  #else  // OPTIMIZE_HARDWARE_SPI

	// skip data before offset
	for (; offset_ < offset; offset_++) {
		spiRecv();
	}
	// transfer data
	spiRecvMulti(dst, count);
#endif  // OPTIMIZE_HARDWARE_SPI

	offset_ += count;
	if (!partialBlockRead_ || offset_ >= 512) {
		// read rest of data, checksum and set chip select high
		readEnd();
	}
	return true;

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void Sd2Card::readEnd(void) {
	if (inBlock_) {
		// skip data and crc
		while (offset_++ < 514) {
			spiRecv();
		}
		chipSelectHigh();
		inBlock_ = 0;
	}
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t Sd2Card::readRegister(uint8_t cmd, void *buf) {
	uint8_t *dst = reinterpret_cast<uint8_t*>(buf);
	if (cardCommand(cmd, 0)) {
		error(SD_CARD_ERROR_READ_REG);
		goto fail;
	}
	if (!waitStartBlock()) {
		goto fail;
	}
	// transfer data
	for (uint16_t i = 0; i < 16; i++) {
		dst[i] = spiRecv();
	}
	spiRecv();  // get first crc byte
	spiRecv();  // get second crc byte
	chipSelectHigh();
	return true;

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/**
 Set the SPI clock rate.

 \param[in] sckRateID A value in the range [0, 6].

 The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 for \a scsRateID = 6.

 \return The value one, true, is returned for success and the value zero,
 false, is returned for an invalid value of \a sckRateID.
 */
uint8_t Sd2Card::setSckRate(uint8_t sckRateID) {
	if (sckRateID > 6) {
		error(SD_CARD_ERROR_SCK_RATE);
		return false;
	}
	return true;
}
#ifdef USE_SPI_LIB
//------------------------------------------------------------------------------
// set the SPI clock frequency
uint8_t Sd2Card::setSpiClock(uint32_t clock) {
	return true;
}
#endif
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(unsigned int timeoutMillis) {
	do {
		if (spiRecv() == 0xFF) {
			return true;
		}
	} while (true);
	return false;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t Sd2Card::waitStartBlock(void) {
	unsigned int t0 = millis();
	while ((status_ = spiRecv()) == 0XFF) {
		unsigned int d = millis() - t0;
		if (d > SD_READ_TIMEOUT) {
			error(SD_CARD_ERROR_READ_TIMEOUT);
			goto fail;
		}
	}
	if (status_ != DATA_START_BLOCK) {
		error(SD_CARD_ERROR_READ);
		goto fail;
	}
	return true;

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/**
 Writes a 512 byte block to an SD card.

 \param[in] blockNumber Logical block to be written.
 \param[in] src Pointer to the location of the data to be written.
 \param[in] blocking If the write should be blocking.
 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t *src,
		uint8_t blocking) {
#if SD_PROTECT_BLOCK_ZERO
	// don't allow write to first block
	if (blockNumber == 0) {
		error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
		goto fail;
	}
#endif  // SD_PROTECT_BLOCK_ZERO

	// use address if not SDHC card
	if (type() != SD_CARD_TYPE_SDHC) {
		blockNumber <<= 9;
	}
	if (cardCommand(CMD24, blockNumber)) {
		error(SD_CARD_ERROR_CMD24);
		goto fail;
	}
	if (!writeData(DATA_START_BLOCK, src)) {
		goto fail;
	}
	if (blocking) {
		// wait for flash programming to complete
		if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
			error(SD_CARD_ERROR_WRITE_TIMEOUT);
			goto fail;
		}
		// response is r2 so get and check two bytes for nonzero
		if (cardCommand(CMD13, 0) || spiRecv()) {
			error(SD_CARD_ERROR_WRITE_PROGRAMMING);
			goto fail;
		}
	}
	chipSelectHigh();
	return true;

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/** Write one data block in a multiple block write sequence */
uint8_t Sd2Card::writeData(const uint8_t *src) {
	// wait for previous write to finish
	if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
		error(SD_CARD_ERROR_WRITE_MULTIPLE);
		chipSelectHigh();
		return false;
	}
	return writeData(WRITE_MULTIPLE_TOKEN, src);
}
//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
uint8_t Sd2Card::writeData(uint8_t token, const uint8_t *src) {
	spiSend(token);
	spiSendMulti(src, 512);
	spiSend(0xff);  // dummy crc
	spiSend(0xff);  // dummy crc

	status_ = spiRecv();
	if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
		;
		status_ = spiRecv();
		status_ = spiRecv();
		status_ = spiRecv();
		status_ = spiRecv();
		status_ = spiRecv();
		error(SD_CARD_ERROR_WRITE);
		chipSelectHigh();
		return false;
	}
	return true;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.

 \param[in] blockNumber Address of first block in sequence.
 \param[in] eraseCount The number of blocks to be pre-erased.

 \note This function is used with writeData() and writeStop()
 for optimized multiple block writes.

 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount) {
#if SD_PROTECT_BLOCK_ZERO
	// don't allow write to first block
	if (blockNumber == 0) {
		error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
		goto fail;
	}
#endif  // SD_PROTECT_BLOCK_ZERO
	// send pre-erase count
	if (cardAcmd(ACMD23, eraseCount)) {
		error(SD_CARD_ERROR_ACMD23);
		goto fail;
	}
	// use address if not SDHC card
	if (type() != SD_CARD_TYPE_SDHC) {
		blockNumber <<= 9;
	}
	if (cardCommand(CMD25, blockNumber)) {
		error(SD_CARD_ERROR_CMD25);
		goto fail;
	}
	return true;

	fail: chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.

 \return The value one, true, is returned for success and
 the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStop(void) {
	if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
		goto fail;
	}
	spiSend(STOP_TRAN_TOKEN);
	if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
		goto fail;
	}
	chipSelectHigh();
	return true;

	fail: error(SD_CARD_ERROR_STOP_TRAN);
	chipSelectHigh();
	return false;
}
//------------------------------------------------------------------------------
/** Check if the SD card is busy

 \return The value one, true, is returned when is busy and
 the value zero, false, is returned for when is NOT busy.
 */
uint8_t Sd2Card::isBusy(void) {
	chipSelectLow();
	uint8_t b = spiRecv();
	chipSelectHigh();

	return (b != 0XFF);
}
