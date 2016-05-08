/* DW1000 library */

#include "SPI.h"
#include "DW1000.h"
#include "digitalWriteFast.h"

/****************************************************************************/
/* CONSTRUCTOR */

DW1000::DW1000(int ss, int rst) {
	_ss = ss;	// global _ss slave select pin
	_rst = rst;	// global _rst reset pin

	pinMode(_ss, OUTPUT);
	digitalWrite(_ss, HIGH);
	pinMode(_rst, OUTPUT);
	digitalWrite(_rst, HIGH);

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV8);
}

/****************************************************************************/
/* DW1000 API */

void DW1000::resetChip() {
	digitalWrite(_rst, LOW);
	delay(5);
	digitalWrite(_rst, HIGH);
	delay(5);
}

void DW1000::softReset() {
	long buffer = readReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, PMSC_CTRL0_LEN);
	writeReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, (buffer | PMSC_CTRL0_SYSCLKS_19M), PMSC_CTRL0_LEN);
	writeReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, (buffer & 0xEFFFFFFF), PMSC_CTRL0_LEN);
	writeReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, (buffer | 0xF0000000), PMSC_CTRL0_LEN);
	writeReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, (buffer | PMSC_CTRL0_SYSCLKS_AUTO), PMSC_CTRL0_LEN);
}

void DW1000::printAnything(byte cmd, int subindex, uint16_t offset, int n) {
	long result = readReg(cmd, subindex, offset, n);
	Serial.print(result, HEX);
}

void DW1000::printAnythingln(byte cmd, int subindex, uint16_t offset, int n) {
	long result = readReg(cmd, subindex, offset, n);
	Serial.println(result, HEX);
}

void DW1000::writePANAddress() {
	writeReg(PANADR_ID, NO_SUB, NO_OFFSET, 0x12345678, PANADR_LEN);
}

/* My settings (recommended values from DWM1000 User Manual)
 * Channel:				3
 * PRF:					16 MHz
 * Data rate:			6.8 Mbps
 * Preamble code:		5
 * Preamble length:		128
 * PAC size:			64 (changed from 8)
 * SFD length:			64 (changed from 8 (standard))
 */
void DW1000::initialise() {
	// Reset chip
	digitalWrite(_rst, LOW);
	delay(5);
	digitalWrite(_rst, HIGH);
	delay(5);

	// Channel, preamble, bitrate selection
	writeReg(CHAN_CTRL_ID, NO_SUB, NO_OFFSET, 0x29460033, CHAN_CTRL_LEN);
	writeReg(TX_FCTRL_ID, NO_SUB, NO_OFFSET, 0x0015400C, TX_FCTRL_LEN);
	writeReg(ACK_RESP_T_ID, NO_SUB, NO_OFFSET, 0x00000000, ACK_RESP_T_LEN); // changed
	writeReg(SYS_CFG_ID, NO_SUB, NO_OFFSET, 0x00001200, SYS_CFG_LEN);
	writeReg(TX_POWER_ID, NO_SUB, NO_OFFSET, 0x0E080222, TX_POWER_LEN);

	// Default values that should be modified
	writeReg(AGC_CTRL_ID, SUB, AGC_TUNE1_OFFSET, (AGC_TUNE1_16M & AGC_TUNE1_MASK), AGC_TUNE1_LEN);
	writeReg(AGC_CTRL_ID, SUB, AGC_TUNE2_OFFSET, (AGC_TUNE2_VAL & AGC_TUNE2_MASK), AGC_TUNE2_LEN);
	writeReg(AGC_CTRL_ID, SUB, AGC_TUNE3_OFFSET, (AGC_TUNE3_VAL & AGC_TUNE3_MASK), AGC_TUNE3_LEN);

	writeReg(DRX_CONF_ID, SUB, DRX_TUNE0b_OFFSET, 0x0002, DRX_TUNE0b_LEN);
	writeReg(DRX_CONF_ID, SUB, DRX_TUNE1a_OFFSET, 0x0087, DRX_TUNE1a_LEN);
	writeReg(DRX_CONF_ID, SUB, DRX_TUNE1b_OFFSET, 0x0020, DRX_TUNE1b_LEN);
	writeReg(DRX_CONF_ID, SUB, DRX_TUNE2_OFFSET, 0x311A002D, DRX_TUNE2_LEN);
	writeReg(DRX_CONF_ID, SUB, DRX_TUNE4H_OFFSET, 0x0028, DRX_TUNE4H_LEN);

	writeReg(RF_CONF_ID, SUB, RF_RXCTRLH_OFFSET, 0xD8, 1);
	writeReg(RF_CONF_ID, SUB, RF_TXCTRL_OFFSET, RF_TXCTRL_CH3, RF_TXCTRL_LEN);

	writeReg(TX_CAL_ID, SUB, TC_PGDELAY_OFFSET, TC_PGDELAY_CH3, TC_PGDELAY_LEN);

	writeReg(FS_CTRL_ID, SUB, FS_PLLCFG_OFFSET, FS_PLLCFG_CH3, FS_PLLCFG_LEN);
	writeReg(FS_CTRL_ID, SUB, FS_PLLTUNE_OFFSET, FS_PLLTUNE_CH3, FS_PLLTUNE_LEN);

	writeReg(LDE_IF_ID, SUB, LDE_CFG1_OFFSET, 0x6D, LDE_CFG1_LEN);
	writeReg(LDE_IF_ID, SUB, LDE_CFG2_OFFSET, 0x1607, LDE_CFG2_LEN);
	writeReg(LDE_IF_ID, SUB, LDE_REPC_OFFSET, (0x451E), LDE_REPC_LEN);

	// Ensure CPLOCK and CPLL_LL flags are working correctly
	writeReg(EXT_SYNC_ID, SUB, EC_CTRL_OFFSET, 0x4, EC_CTRL_LEN);

	// writeReg(AON_ID, SUB, AON_WCFG_OFFSET, 0, AON_WCFG_LEN);

	// Load LDE microcode from ROM to RAM
	writeReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, 0x0301, 2);
	writeReg(OTP_IF_ID, SUB, OTP_CTRL, OTP_CTRL_LDELOAD, 2);
	delayMicroseconds(150);
	long temp = readReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, 2);
	writeReg(PMSC_ID, SUB, PMSC_CTRL0_OFFSET, 0x0200, 2);

	// long temp = readReg(PMSC_ID, SUB, PMSC_CTRL1_OFFSET, PMSC_CTRL1_LEN);
	// writeReg(PMSC_ID, SUB, PMSC_CTRL1_OFFSET, (temp & 0xFFFDFFFF), PMSC_CTRL1_LEN);
	// writeReg(OTP_IF_ID, SUB, OTP_CTRL, 0x8000, OTP_IF_LEN);
	// delayMicroseconds(150);
	// writeReg(PMSC_ID, SUB, PMSC_CTRL1_OFFSET, (temp | PMSC_CTRL1_LDERUNE), PMSC_CTRL1_LEN);
}

void DW1000::loadLDE() {
	long temp = readReg(PMSC_ID, SUB, PMSC_CTRL1_OFFSET, PMSC_CTRL1_LEN);
	writeReg(PMSC_ID, SUB, PMSC_CTRL1_OFFSET, (temp & 0xFFFDFFFF), PMSC_CTRL1_LEN);
	writeReg(OTP_IF_ID, SUB, OTP_CTRL, 0x8000, OTP_IF_LEN);
	delayMicroseconds(150);
	writeReg(PMSC_ID, SUB, PMSC_CTRL1_OFFSET, (temp | PMSC_CTRL1_LDERUNE), PMSC_CTRL1_LEN);
}

uint64_t DW1000::readDeviceIdentifier() {
	uint64_t result = readReg(DEV_ID_ID, NO_SUB, NO_OFFSET, DEV_ID_LEN);
	return result;
}

uint64_t DW1000::readSystemConfiguration() {
	_syscfg = readReg(SYS_CFG_ID, NO_SUB, NO_OFFSET, SYS_CFG_LEN);
	return _syscfg;
}

uint64_t DW1000::readSystemStatus() {
	_sysstat = readReg(SYS_STATUS_ID, NO_SUB, NO_OFFSET, SYS_STATUS_LEN);
	return _sysstat;
}

// uint64_t temp = readReg(RX_TIME_ID, SUB, RX_TIME_RX_STAMP_OFFSET, RX_TIME_RX_STAMP_LEN);
double DW1000::readRxTimestamp() {
	uint64_t low = readReg(RX_TIME_ID, SUB, RX_TIME_RX_STAMP_OFFSET, 4);
	uint64_t high = readReg(RX_TIME_ID, SUB, RX_TIME_RX_STAMP_OFFSET+4, 1);

	uint64_t rxtime = low | (high << 32);
	double result = rxtime * (1/(128*499.2e6));

	return result;
}

int DW1000::fpsp_f1() {
	return readReg(RX_TIME_ID, SUB, RX_TIME_FP_AMPL1_OFFSET, 2);
}

int DW1000::fpsp_f2() {
	return ((readReg(RX_FQUAL_ID, NO_SUB, NO_OFFSET, 4) & RX_EQUAL_FP_AMPL2_MASK) >> RX_EQUAL_FP_AMPL2_SHIFT);
}

int DW1000::fpsp_f3() {
	return ((readReg(RX_FQUAL_ID, NO_SUB, NO_OFFSET, RX_FQUAL_LEN) & RX_EQUAL_PP_AMPL3_MASK) >> RX_EQUAL_PP_AMPL3_SHIFT);
}

int DW1000::fpsp_n() {
	return ((readReg(RX_FINFO_ID, NO_SUB, NO_OFFSET, RX_FINFO_LEN) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT);
}

int DW1000::rx_c() {
	return ((readReg(RX_FQUAL_ID, NO_SUB, NO_OFFSET, RX_FQUAL_LEN) & RX_EQUAL_CIR_MXG_MASK) >> RX_EQUAL_CIR_MXG_SHIFT);
}

double DW1000::readFirstPathSignalPower(int PRF) {
	uint16_t f1 = readReg(RX_TIME_ID, SUB, RX_TIME_FP_AMPL1_OFFSET, 2);
	uint16_t f2 = (readReg(RX_FQUAL_ID, NO_SUB, NO_OFFSET, 4) & RX_EQUAL_FP_AMPL2_MASK) >> RX_EQUAL_FP_AMPL2_SHIFT;
	uint16_t f3 = (readReg(RX_FQUAL_ID, NO_SUB, NO_OFFSET, RX_FQUAL_LEN) & RX_EQUAL_PP_AMPL3_MASK) >> RX_EQUAL_PP_AMPL3_SHIFT;
	uint32_t n = (readReg(RX_FINFO_ID, NO_SUB, NO_OFFSET, RX_FINFO_LEN) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
	double A = 115.72;

	double result = (10*log10(((f1^2)+(f2^2)+(f3^2))/(n^2))) - 115.72;
	return result;
}

double DW1000::readRxLevel(int PRF) {
	uint16_t c = (readReg(RX_FQUAL_ID, NO_SUB, NO_OFFSET, RX_FQUAL_LEN) & RX_EQUAL_CIR_MXG_MASK) >> RX_EQUAL_CIR_MXG_SHIFT;
	uint32_t n = (readReg(RX_FINFO_ID, NO_SUB, NO_OFFSET, RX_FINFO_LEN) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
	double A = 115.72;

	double result = (10*log10((c*(2^17))/(n^2))) - 115.72;
	return result;
}

uint64_t DW1000::readICTempAndVoltage() {
	writeReg(RF_CONF_ID, SUB, 0x11, 0x80, 1);
	writeReg(RF_CONF_ID, SUB, 0x12, 0x0A, 1);
	writeReg(RF_CONF_ID, SUB, 0x12, 0x0F, 1);
	writeReg(TX_CAL_ID, SUB, TC_SARL_SAR_C, 0x00, 1);
	writeReg(TX_CAL_ID, SUB, TC_SARL_SAR_C, 0x01, 1);	// Set SAR enable
	
	_ICtemp = readReg(TX_CAL_ID, SUB, TC_SARL_SAR_LVBAT_OFFSET, 1);
	_ICvoltage = readReg(TX_CAL_ID, SUB, TC_SARL_SAR_LTEMP_OFFSET, 1);
	writeReg(TX_CAL_ID, SUB, TC_SARL_SAR_C, 0x00, 1);	// Clears SAR enable

	return ((_ICtemp << 8) | _ICvoltage);
}

void DW1000::readOTP(int addr) {
	writeReg(OTP_IF_ID, SUB, OTP_ADDR, addr, OTP_ADDR_LEN);
	writeReg(OTP_IF_ID, SUB, OTP_CTRL, 0x03, OTP_CTRL_LEN);
	writeReg(OTP_IF_ID, SUB, OTP_CTRL, 0x00, OTP_CTRL_LEN);

	long result = readReg(OTP_IF_ID, SUB, OTP_RDAT, OTP_RDAT_LEN);
	Serial.println(result, HEX);
}

void DW1000::clearSystemStatus(long bit) {
	writeReg(SYS_STATUS_ID, NO_SUB, NO_OFFSET, (0xFFFFFFFF & bit), SYS_STATUS_LEN);
}

void DW1000::setSystemConfig(uint64_t buffer) {
	writeReg(SYS_CFG_ID, NO_SUB, NO_OFFSET, buffer, SYS_CFG_LEN);
}

void DW1000::toggleGPIO_MODE() {
	writeReg(GPIO_CTRL_ID, SUB, GPIO_MODE_OFFSET, 0x00001400, GPIO_MODE_LEN);
	writeReg(PMSC_ID, SUB, PMSC_LEDC_OFFSET, 0x00000120, PMSC_LEDC_LEN);
}

void DW1000::setTxFrameControl(long buffer) {
	writeReg(TX_FCTRL_ID, NO_SUB, NO_OFFSET, buffer, TX_FCTRL_LEN);
}

void DW1000::writeTxBuffer(byte buffer[], int n) {
    //currently this call only deal with n<127 short message
    byte temp[n];
    for (uint8_t i=0;i<n;i++){
    	temp[i] = buffer[n-1-i];
    }
    uint8_t len = n+2;
    uint32_t fctrl = readReg(TX_FCTRL_ID, NO_SUB, NO_OFFSET, TX_FCTRL_LEN);
    writeReg(TX_FCTRL_ID, NO_SUB, NO_OFFSET, (fctrl | 0x7F & len), TX_FCTRL_LEN);
    // write data to data buffer
    writeBytes(TX_BUFFER_ID, NO_SUB, temp, n);

}

uint64_t DW1000::readRxBuffer(uint16_t offset, int n) {
	uint64_t rxBuffer = readReg(RX_BUFFER_ID, NO_SUB, offset, n);
	return rxBuffer;
}

void DW1000::startTx() {
	writeReg(SYS_CTRL_ID, NO_SUB, NO_OFFSET, SYS_CTRL_TXSTRT, SYS_CTRL_LEN);
}

void DW1000::startRx() {
	writeReg(SYS_CTRL_ID, NO_SUB, NO_OFFSET, SYS_CTRL_RXENAB, SYS_CTRL_LEN);
}

void DW1000::startTxRx() {
	writeReg(SYS_CTRL_ID, NO_SUB, NO_OFFSET, (SYS_CTRL_TXSTRT | SYS_CTRL_RXENAB), SYS_CTRL_LEN);
}

void DW1000::stopTxRx() {
	writeReg(SYS_CTRL_ID, NO_SUB, NO_OFFSET, 0x40, SYS_CTRL_LEN);
}


void DW1000::setPanID(uint16_t val) {
    //read 0x03
    uint32_t address = readReg(PANADR_ID, NO_SUB, NO_OFFSET, 4);

    address = ((uint32_t)val << 16) | (0xFFFF & address);
    Serial.print("Address set to: ");
    Serial.println(address, HEX);
    writeReg(PANADR_ID, NO_SUB, NO_OFFSET, address, PANADR_LEN);

}

void DW1000::setShortID(uint16_t val) {
    //read 0x03
    uint32_t address = readReg(PANADR_ID, NO_SUB, NO_OFFSET, 4);
    address = (0xffff0000 & address) | (0xFFFF & (uint32_t)val);
    writeReg(PANADR_ID, NO_SUB, NO_OFFSET, address, PANADR_LEN);
}


/****************************************************************************/
/* HELPER FUNCTIONS */

uint64_t DW1000::readReg(byte cmd, int subindex, uint16_t offset, int n) {
	int header [3];
	int headerLen = 1;
	byte data[n];
	uint64_t result = 0;
	int i;

	/* Filter results more than 4 octets */
	if (n > 8) {
		return 0;									// TODO: return error
	}

	/* Generate header */
    if (!subindex) {
    	header[0] = READ | cmd;						// 0x80 OR with command if no sub-index
    }
    else {
    	header[0] = READ_SUB | cmd;					// 0x40 OR with command if sub-index is present
    	if (offset < 128) {							// Check if extended sub-index is needed
    		header[1] = offset;						
    		headerLen = 2;
    	}
    	else {
    		int sub1 = 0x7F & offset;				// 0x7F OR with offset if extended sub-index is used 
    		int sub2 = (0x7F80 & offset) >> 7;		// Remaining offset shifted right 7 bits
    		header[1] = 0x80 | sub1;
    		header[2] = sub2;
    		headerLen = 3;
    	}
    }

	/* SPI transaction */
	digitalWrite(_ss, LOW);
	for (i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]);
	}
	for (i = 0; i < n; i++) {
		data[i] = SPI.transfer(0x00);
	}
	digitalWrite(_ss, HIGH);

	/* Invert bytes because SPI transactions reads LSB first, data[] is arranged from MSB to LSB */
	for (i = n-1; i >= 0; i--) {
		result = (result << 8) | data[i];
	}

	return result;
}

void DW1000::writeReg(byte cmd, int subindex, uint16_t offset, uint64_t buffer, int n) {
	int header[3];									// SPI transaction header
	int headerLen = 1;								// SPI transaction header length
	int i;											// Counter

	/* Split data buffer */
	byte data[n];									// Array
    uint64_t mask = 0xFF;							// Mask for bitwise operation (eg: first iter.- 0000 0000 0000 00FF)
    for (i = 0; i < n; i++) {
        data[i] = (buffer & mask) >> (i * 8);
        mask = mask << 8;
    }

    /* Generate header */
    if (!subindex) {
    	header[0] = WRITE | cmd;					// 0x80 OR with command if no sub-index
    }
    else {
    	header[0] = WRITE_SUB | cmd;				// 0xC0 OR with command if sub-index is present
    	if (offset < 128) {							// Check if extended sub-index is needed
    		header[1] = offset;						
    		headerLen = 2;
    	}
    	else {
    		int sub1 = 0x7F & offset;				// 0x7F OR with offset if extended sub-index is used 
    		int sub2 = (0x7F80 & offset) >> 7;		// Remaining offset shifted right 7 bits
    		header[1] = 0x80 | sub1;
    		header[2] = sub2;
    		headerLen = 3;
    	}
    }

    /* SPI transaction */
    digitalWriteFast(_ss, LOW);
    for (i = 0; i < headerLen; i++) {
    	SPI.transfer(header[i]);
    }
    for (i = 0; i < n; i++) {
    	SPI.transfer(data[i]);
    }
    digitalWriteFast(_ss, HIGH);
}


void DW1000::writeBytes(byte cmd, word offset, byte data[], unsigned int n) {
    byte header[3];
    int headerLen = 1;
    int i;
    // TODO proper error handling: address out of bounds
    if(offset == NO_SUB) {
        header[0] = WRITE | cmd;
    } else {
        header[0] = WRITE_SUB | cmd;
        if(offset < 128) {
            header[1] = (byte)offset;
            headerLen++;
        } else {
            header[1] = RW_SUB_EXT | (byte)offset;
            header[2] = (byte)(offset >> 7);
            headerLen+=2;
        }
    }
    //SPI.beginTransaction(*_currentSPI); 
    digitalWriteFast(_ss, LOW);
    for(i = 0; i < headerLen; i++) {
        SPI.transfer(header[i]);
    }
    for(i = 0; i < n; i++) {
        SPI.transfer(data[i]);
    }
    delayMicroseconds(5);
    digitalWriteFast(_ss,HIGH);
    //SPI.endTransaction();
}
