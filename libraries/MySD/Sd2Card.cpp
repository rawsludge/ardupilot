//
//  Sd2Card.cpp
//  tinyFat
//
//  Created by Ahmet ÖZTÜRK on 28/12/14.
//
//

#if defined(__AVR__)
    #if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
    #else
        #include "WProgram.h"
    #endif
#endif

#include "Sd2Card.h"

extern const AP_HAL::HAL& hal;

// functions for hardware SPI
/** Send a byte to the card */
void Sd2Card::spiSend(uint8_t data) {
    _spi->transfer(data);
}
/** Receive a byte from the card */
uint8_t Sd2Card::spiRec(void) {
    return  _spi->transfer(0xFF);
}

bool Sd2Card::_sem_take(uint8_t timeout)
{
    if (hal.scheduler->in_timerprocess()) {
        return _spi_sem->take_nonblocking();
    }
    return _spi_sem->take(timeout);
}

//------------------------------------------------------------------------------
void Sd2Card::chipSelectHigh(void) {
    _spi->cs_release();
    //_spi_sem->give();
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void) {
    //if( !_sem_take(5))
    //    hal.scheduler->panic(PSTR("PANIC: DataFlash could not take null"));
    _spi->cs_assert();
}

//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
    // end read if in partialBlockRead mode
    readEnd();
    
    // select card
    chipSelectLow();
    
    // wait up to 300 ms if busy
    waitNotBusy(300);
    
    // send command
    spiSend(cmd | 0x40);
    
    // send argument
    for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);
    
    // send CRC
    uint8_t crc = 0XFF;
    if (cmd == CMD0) crc = 0X95;  // correct crc for CMD0 with arg 0
    if (cmd == CMD8) crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
    spiSend(crc);
    
    // wait for response
    for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++);
    return status_;
}
//------------------------------------------------------------------------------
/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t Sd2Card::cardSize(void) {
    csd_t csd;
    if (!readCSD(&csd)) return 0;
    if (csd.v1.csd_ver == 0) {
        uint8_t read_bl_len = csd.v1.read_bl_len;
        uint16_t c_size = (csd.v1.c_size_high << 10)
        | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
        uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
        | csd.v1.c_size_mult_low;
        return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
    } else if (csd.v2.csd_ver == 1) {
        uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
        | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
        return (c_size + 1) << 10;
    } else {
        error(SD_CARD_ERROR_BAD_CSD);
        return 0;
    }
}

//------------------------------------------------------------------------------
/** Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 *
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
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
    if (cardCommand(CMD32, firstBlock)
        || cardCommand(CMD33, lastBlock)
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
    
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/** Determine if card supports single block erase.
 *
 * \return The value one, true, is returned if single block erase is supported.
 * The value zero, false, is returned if single block erase is not supported.
 */
uint8_t Sd2Card::eraseSingleBlockEnable(void) {
    csd_t csd;
    return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}
//------------------------------------------------------------------------------
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  The reason for failure
 * can be determined by calling errorCode() and errorData().
 */
uint8_t Sd2Card::init(uint8_t sckRateID) {
    writeCRC_ = errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;

    uint16_t t0 = (uint16_t)hal.scheduler->millis();
    uint32_t arg;
    
    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);
    if (_spi == NULL) {
        hal.scheduler->panic(
                             PSTR("PANIC: DataFlash SPIDeviceDriver not found"));
        return status_; /* never reached */
    }
    
    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        hal.scheduler->panic(
                             PSTR("PANIC: DataFlash SPIDeviceDriver semaphore is null"));
        return status_; /* never reached */
    }
    
    // must supply min of 74 clock cycles with CS high.
    for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);
    
      chipSelectLow();
    
    // command to go idle in SPI mode
    while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
        if (((uint16_t)hal.scheduler->millis() - t0) > SD_INIT_TIMEOUT) {
            error(SD_CARD_ERROR_CMD0);
            goto fail;
        }
    }
    // check SD version
    if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
        type(SD_CARD_TYPE_SD1);
    } else {
        // only need last byte of r7 response
        for (uint8_t i = 0; i < 4; i++) status_ = spiRec();
        if (status_ != 0XAA) {
            error(SD_CARD_ERROR_CMD8);
            goto fail;
        }
        type(SD_CARD_TYPE_SD2);
    }
    // initialize card and send host supports SDHC if SD2
    arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;
    
    while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
        // check for timeout
        if (((uint16_t)hal.scheduler->millis() - t0) > SD_INIT_TIMEOUT) {
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
        if ((spiRec() & 0XC0) == 0XC0) type(SD_CARD_TYPE_SDHC);
        // discard rest of ocr - contains allowed voltage range
        for (uint8_t i = 0; i < 3; i++) spiRec();
    }
    chipSelectHigh();
    return setSckRate(sckRateID);
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/**
 * Enable or disable partial block reads.
 *
 * Enabling partial block reads improves performance by allowing a block
 * to be read over the SPI bus as several sub-blocks.  Errors may occur
 * if the time between reads is too long since the SD card may timeout.
 * The SPI SS line will be held low until the entire block is read or
 * readEnd() is called.
 *
 * Use this for applications like the Adafruit Wave Shield.
 *
 * \param[in] value The value TRUE (non-zero) or FALSE (zero).)
 */
void Sd2Card::partialBlockRead(uint8_t value) {
    readEnd();
    partialBlockRead_ = value;
}
//------------------------------------------------------------------------------
/**
 * Read a 512 byte block from an SD card device.
 *
 * \param[in] block Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.
 
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
    return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from an SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readData(uint32_t block, uint16_t offset, uint16_t count, uint8_t* dst) {
    uint16_t n;
    if (count == 0) return true;
    if ((count + offset) > 512) {
        goto fail;
    }
    if (!inBlock_ || block != block_ || offset < offset_) {
        block_ = block;
        // use address if not SDHC card
        if (type()!= SD_CARD_TYPE_SDHC) block <<= 9;
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
    
    // skip data before offset
    for (;offset_ < offset; offset_++) {
        spiRec();
    }
    // transfer data
    for (uint16_t i = 0; i < count; i++) {
        dst[i] = spiRec();
    }
    
    offset_ += count;
    if (!partialBlockRead_ || offset_ >= 512) {
        // read rest of data, checksum and set chip select high
        readEnd();
    }
    return true;
    
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void Sd2Card::readEnd(void) {
    if (inBlock_) {
        // skip data and crc
#ifdef OPTIMIZE_HARDWARE_SPI
        // optimize skip for hardware
        SPDR = 0XFF;
        while (offset_++ < 513) {
            while (!(SPSR & (1 << SPIF)));
            SPDR = 0XFF;
        }
        // wait for last crc byte
        while (!(SPSR & (1 << SPIF)));
#else  // OPTIMIZE_HARDWARE_SPI
        while (offset_++ < 514) spiRec();
#endif  // OPTIMIZE_HARDWARE_SPI
        chipSelectHigh();
        inBlock_ = 0;
    }
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t Sd2Card::readRegister(uint8_t cmd, void* buf) {
    uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
    if (cardCommand(cmd, 0)) {
        error(SD_CARD_ERROR_READ_REG);
        goto fail;
    }
    if (!waitStartBlock()) goto fail;
    // transfer data
    for (uint16_t i = 0; i < 16; i++) dst[i] = spiRec();
    spiRec();  // get first crc byte
    spiRec();  // get second crc byte
    chipSelectHigh();
    return true;
    
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/**
 * Set the SPI clock rate.
 *
 * \param[in] sckRateID A value in the range [0, 6].
 *
 * The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 * SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 * for \a scsRateID = 6.
 *
 * \return The value one, true, is returned for success and the value zero,
 * false, is returned for an invalid value of \a sckRateID.
 */
uint8_t Sd2Card::setSckRate(uint8_t sckRateID) {
    if (sckRateID > 6) {
        error(SD_CARD_ERROR_SCK_RATE);
        return false;
    }
    _spi->set_speed(sckRateID);
    return true;
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(uint16_t timeoutMillis) {
    uint16_t t0 = hal.scheduler->millis();
    do {
        if (spiRec() == 0XFF) return true;
    }
    while (((uint16_t)hal.scheduler->millis() - t0) < timeoutMillis);
    return false;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t Sd2Card::waitStartBlock(void) {
    uint16_t t0 = hal.scheduler->millis();
    while ((status_ = spiRec()) == 0XFF) {
        if (((uint16_t)hal.scheduler->millis() - t0) > SD_READ_TIMEOUT) {
            error(SD_CARD_ERROR_READ_TIMEOUT);
            goto fail;
        }
    }
    if (status_ != DATA_START_BLOCK) {
        error(SD_CARD_ERROR_READ);
        goto fail;
    }
    return true;
    
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
#if SD_PROTECT_BLOCK_ZERO
    // don't allow write to first block
    if (blockNumber == 0) {
        error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
        goto fail;
    }
#endif  // SD_PROTECT_BLOCK_ZERO
    
    // use address if not SDHC card
    if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
    if (cardCommand(CMD24, blockNumber)) {
        error(SD_CARD_ERROR_CMD24);
        goto fail;
    }
    if (!writeData(DATA_START_BLOCK, src)) goto fail;
    
    // wait for flash programming to complete
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
        error(SD_CARD_ERROR_WRITE_TIMEOUT);
        goto fail;
    }
    // response is r2 so get and check two bytes for nonzero
    if (cardCommand(CMD13, 0) || spiRec()) {
        error(SD_CARD_ERROR_WRITE_PROGRAMMING);
        goto fail;
    }
    chipSelectHigh();
    return true;
    
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/** Write one data block in a multiple block write sequence */
uint8_t Sd2Card::writeData(const uint8_t* src) {
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
uint8_t Sd2Card::writeData(uint8_t token, const uint8_t* src) {
    
    // CRC16 checksum is supposed to be ignored in SPI mode (unless
    // explicitly enabled) and a dummy value is normally written.
    // A few funny cards (e.g. Eye-Fi X2) expect a valid CRC anyway.
    // Call setCRC(true) to enable CRC16 checksum on block writes.
    // This has a noticeable impact on write speed. :(
    int16_t crc;
    if(writeCRC_) {
        int16_t i, x;
        // CRC16 code via Scott Dattalo www.dattalo.com
        for(crc=i=0; i<512; i++) {
            x   = ((crc >> 8) ^ src[i]) & 0xff;
            x  ^= x >> 4;
            crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
        }
    } else {
        crc = 0xffff; // Dummy CRC value
    }
    
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++) {
        spiSend(src[i]);
    }
    
    spiSend(crc >> 8); // Might be dummy value, that's OK
    spiSend(crc);
    
    status_ = spiRec();
    if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
        error(SD_CARD_ERROR_WRITE);
        chipSelectHigh();
        return false;
    }
    return true;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and writeStop()
 * for optimized multiple block writes.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
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
    if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
    if (cardCommand(CMD25, blockNumber)) {
        error(SD_CARD_ERROR_CMD25);
        goto fail;
    }
    return true;
    
fail:
    chipSelectHigh();
    return false;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStop(void) {
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
    spiSend(STOP_TRAN_TOKEN);
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
    chipSelectHigh();
    return true;
    
fail:
    error(SD_CARD_ERROR_STOP_TRAN);
    chipSelectHigh();
    return false;
}

void Sd2Card::enableCRC(uint8_t mode) {
    writeCRC_ = mode;
}

