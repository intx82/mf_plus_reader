/*
* MFRC522_T.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
* NOTE: Please also check the comments in MFRC522_T.h - they provide useful hints and background information.
* Released into the public domain.
*/
#include "MFRC522.hpp"


const uint8_t MFRC522_T::rnda[] = {0,1,2,3,4,5,6,7,8,9,0xa,0xb,0xc,0xd,0xe,0xf};
/**
 * Writes a uint8_t to the specified register in the MFRC522_T chip.
 * The interface is described in the datasheet section 8.1.2.
 * @param The register to write to. One of the PCD_Register enums.
 * @param The value to write.
 */
void MFRC522_T::PCD_WriteRegister(uint8_t reg,uint8_t value)
	{
	SPI->ena(1);		// Select slave
	SPI->transfer(reg & 0x7E);				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	SPI->transfer(value);
	SPI->ena(0);		// Release slave again
	} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522_T chip.
 * The interface is described in the datasheet section 8.1.2.
 * @param The register to write to. One of the PCD_Register enums.
 * @param The number of bytes to write to the register
 * @param The values to write. uint8_t array.
 */
void MFRC522_T::PCD_WriteRegister(uint8_t reg,uint8_t count,uint8_t *values)
	{
	SPI->ena(1);
	SPI->transfer(reg & 0x7E);				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (uint8_t index = 0; index < count; index++)
		{
		SPI->transfer(values[index]);
		}
	SPI->ena(0);		// Release slave again
	// Stop using the SPI bus
	} // End PCD_WriteRegister()

/**
 * Reads a uint8_t from the specified register in the MFRC522_T chip.
 * The interface is described in the datasheet section 8.1.2.
 * @param The register to read from. One of the PCD_Register enums.
 */
uint8_t MFRC522_T::PCD_ReadRegister(uint8_t reg)
	{
	uint8_t value;
	SPI->ena(1);			// Select slave
	SPI->transfer(0x80 | (reg & 0x7E));			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	value = SPI->transfer(0);					// Read the value back. Send 0 to stop reading.
	SPI->ena(0);			// Release slave again
	// Stop using the SPI bus
	return value;
	} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522_T chip.
 * The interface is described in the datasheet section 8.1.2.
 * @param The register to read from. One of the PCD_Register enums.
 * @param The number of bytes to read
 * @param uint8_t array to store the values in.
 * @param Only bit positions rxAlign..7 in values[0] are updated.
 */
void MFRC522_T::PCD_ReadRegister(	uint8_t reg,uint8_t count,uint8_t *values,uint8_t rxAlign)
	{
	if (count == 0)
		{
		return;
		}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
	uint8_t address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	// Set the settings to work with SPI bus
	SPI->ena(1);		// Select slave
	count--;								// One read is performed outside of the loop
	SPI->transfer(address);					// Tell MFRC522_T which address we want to read
	while (index < count)
		{
		if (index == 0 && rxAlign)  		// Only update bit positions rxAlign..7 in values[0]
			{
			// Create bit mask for bit positions rxAlign..7
			uint8_t mask = 0;
			for (uint8_t i = rxAlign; i <= 7; i++)
				{
				mask |= (1 << i);
				}
			// Read value and tell that we want to read the same address again.
			uint8_t value = SPI->transfer(address);
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] & ~mask) | (value & mask);
			}
		else   // Normal case
			{
			values[index] = SPI->transfer(address);	// Read value and tell that we want to read the same address again.
			}
		index++;
		}
	values[index] = SPI->transfer(0);			// Read the final uint8_t. Send 0 to stop reading.
	SPI->ena(0);			// Release slave again
	// Stop using the SPI bus
	} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 * @param The register to update. One of the PCD_Register enums.
 * @param The bits to set.
 */
void MFRC522_T::PCD_SetRegisterBitMask(uint8_t reg,uint8_t mask)
	{
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
	} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 * @param The register to update. One of the PCD_Register enums.
 * @param  The bits to clear.
 */
void MFRC522_T::PCD_ClearRegisterBitMask(uint8_t reg,uint8_t mask)
	{
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
	} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522_T to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
		uint8_t length,	///< In: The number of bytes to transfer.
		uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
											   )
	{
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
	uint16_t i = 5000;
	uint8_t n;
	while (1)
		{
		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		if (n & 0x04)  						// CRCIRq bit set - calculation done
			{
			break;
			}
		if (--i == 0)  						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522_T might be down.
			{
			return STATUS_TIMEOUT;
			}
		}
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL);
	result[1] = PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
	} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522_T
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522_T chip.
 */
MFRC522_T::status_t MFRC522_T::PCD_Init()
	{
	// Set the chipSelectPin as digital output, do not select the slave yet
	SPI->init();
	//SPI->reset();

	delay(50);
	if(PCD_Reset()==MFRC522_T::STATUS_TIMEOUT)
		{
		return MFRC522_T::STATUS_TIMEOUT;
		}

	PCD_WriteRegister(TModeReg, 0x80);
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
	PCD_WriteRegister(TReloadRegH, 0x05);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0x00);

	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	delay(50);
	return MFRC522_T::STATUS_OK;
	} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522_T chip and waits for it to be ready again.
 */
MFRC522_T::status_t MFRC522_T::PCD_Reset()
	{
	PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522_T might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
	delay(75);
	// Wait for the PowerDown bit in CommandReg to be cleared
	uint32_t start_time = delay_cnt;
	while (PCD_ReadRegister(CommandReg) & (1<<4))
		{
		if(delay_cnt>(start_time+245))
			{
			return MFRC522_T::STATUS_TIMEOUT;
			}
		} // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	return MFRC522_T::STATUS_OK;
	} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC522_T::PCD_AntennaOn()
	{
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03)
		{
		PCD_WriteRegister(TxControlReg, value | 0x03);
		}
	} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC522_T::PCD_AntennaOff()
	{
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
	} // End PCD_AntennaOff()

/**
 * Get the current MFRC522_T Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522_T.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t MFRC522_T::PCD_GetAntennaGain()
	{
	return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
	} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522_T Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522_T.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void MFRC522_T::PCD_SetAntennaGain(uint8_t mask)
	{
	if (PCD_GetAntennaGain() != mask)  						// only bother if there is a change
		{
		PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
		}
	} // End PCD_SetAntennaGain()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
		uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
		uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
		uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
		uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
		uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
		bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
												 )
	{
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
	} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522_T FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
		uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
		uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
		uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
		uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
		uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
		uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
		uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
		bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
													  )
	{
	uint8_t n, _validBits;
	unsigned int i;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive)
		{
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
		}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86�s.
	i = 20000;
	while (1)
		{
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq)  					// One of the interrupts that signal success has been set.
			{
			break;
			}
		if (n & 0x01)  						// Timer interrupt - nothing received in 25ms
			{
			return STATUS_TIMEOUT;
			}
		if (--i == 0)  						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522_T might be down.
			{
			return STATUS_TIMEOUT;
			}
		}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13)  	 // BufferOvfl ParityErr ProtocolErr
		{
		errorRegValue++;
		return STATUS_ERROR;
		}

	// If the caller wants data back, get it from the MFRC522_T.
	if (backData && backLen)
		{
		n = PCD_ReadRegister(FIFOLevelReg);			// Number of bytes in the FIFO
		if (n > *backLen)
			{
			return STATUS_NO_ROOM;
			}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits)
			{
			*validBits = _validBits;
			}
		}

	// Tell about collisions
	if (errorRegValue & 0x08)  		// CollErr
		{
		return STATUS_COLLISION;
		}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC)
		{
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4)
			{
			return STATUS_MIFARE_NACK;
			}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*backLen < 2 || _validBits != 0)
			{
			return STATUS_CRC_WRONG;
			}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		MFRC522_T::status_t status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK)
			{
			return status;
			}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
			{
			return STATUS_CRC_WRONG;
			}
		}

	return STATUS_OK;
	} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
		uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											)
	{
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
	} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
		uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										   )
	{
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
	} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
		uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
		uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
												)
	{
	uint8_t validBits;
	MFRC522_T::status_t status;
	tcl_pcb_block_num = false;
	if (bufferATQA == NULL || *bufferSize < 2)  	// The ATQA response is 2 bytes long.
		{
		return STATUS_NO_ROOM;
		}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
	if (status != STATUS_OK)
		{
		return status;
		}
	if (*bufferSize != 2 || validBits != 0)  		// ATQA must be exactly 16 bits.
		{
		return STATUS_ERROR;
		}
	return STATUS_OK;
	} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PICC_Select(	uid_t *uid,			///< Pointer to uid_t struct. Normally output, but can also be used to supply a known UID.
		uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										  )
	{
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	MFRC522_T::status_t result;
	uint8_t count;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		uint8_t 3: UID-data
	//		uint8_t 4: UID-data
	//		uint8_t 5: UID-data
	//		uint8_t 6: BCC					Block Check Character - XOR of bytes 2-5
	//		uint8_t 7: CRC_A
	//		uint8_t 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80)
		{
		return STATUS_INVALID;
		}

	// Prepare MFRC522_T
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete)
		{
		// Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
		switch (cascadeLevel)
			{
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
			}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0)
			{
			currentLevelKnownBits = 0;
			}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag)
			{
			buffer[index++] = PICC_CMD_CT;
			}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy)
			{
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes)
				{
				bytesToCopy = maxBytes;
				}
			for (count = 0; count < bytesToCopy; count++)
				{
				buffer[index++] = uid->uidByte[uidIndex + count];
				}
			}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag)
			{
			currentLevelKnownBits += 8;
			}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone)
			{
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32)   // All UID bits in this Cascade Level are known. This is a SELECT.
				{
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK)
					{
					return result;
					}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
				}
			else   // This is an ANTICOLLISION.
				{
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
				}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION)   // More than one PICC in the field => collision.
				{
				uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20)   // CollPosNotValid
					{
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
					}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0)
					{
					collisionPos = 32;
					}
				if (collisionPos <= currentLevelKnownBits)   // No progress - should not happen
					{
					return STATUS_INTERNAL_ERROR;
					}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
				buffer[index]	|= (1 << count);
				}
			else if (result != STATUS_OK)
				{
				return result;
				}
			else   // STATUS_OK
				{
				if (currentLevelKnownBits >= 32)   // This was a SELECT.
					{
					selectDone = true; // No more anticollision
					// We continue below outside the while.
					}
				else   // This was an ANTICOLLISION.
					{
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
					}
				}
			} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++)
			{
			uid->uidByte[uidIndex + count] = buffer[index++];
			}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0)   // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
			{
			return STATUS_ERROR;
			}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK)
			{
			return result;
			}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
			{
			return STATUS_CRC_WRONG;
			}
		if (responseBuffer[0] & 0x04)   // Cascade bit set - UID not complete yes
			{
			cascadeLevel++;
			}
		else
			{
			uidComplete = true;
			uid->sak = responseBuffer[0];
			}
		} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
	} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PICC_HaltA()
	{
	MFRC522_T::status_t result;
	uint8_t buffer[4];
	tcl_pcb_block_num = false;
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK)
		{
		return result;
		}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0);
	if (result == STATUS_TIMEOUT)
		{
		return STATUS_OK;
		}
	if (result == STATUS_OK)   // That is ironically NOT ok in this case ;-)
		{
		return STATUS_ERROR;
		}
	return result;
	} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522_T MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522_T datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
MFRC522_T::status_t MFRC522_T::PCD_Authenticate(uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
		uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
		MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
		uid_t *uid			///< Pointer to uid_t struct. The first 4 bytes of the UID is used.
											   )
	{
	uint8_t waitIRq = 0x10;		// IdleIRq

	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++)  	// 6 key bytes
		{
		sendData[2+i] = key->keyByte[i];
		}
	for (uint8_t i = 0; i < 4; i++)  				// The first 4 bytes of the UID
		{
		sendData[8+i] = uid->uidByte[i];
		}

	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData));
	} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void MFRC522_T::PCD_StopCrypto1()
	{
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
	} // End PCD_StopCrypto1()

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Read(	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
		uint8_t *buffer,		///< The buffer to store the data in
		uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										  )
	{
	MFRC522_T::status_t result;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18)
		{
		return STATUS_NO_ROOM;
		}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK)
		{
		return result;
		}

	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
	} // End MIFARE_Read()

/**
 * Writes 16 bytes to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Write(	uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
		uint8_t *buffer,	///< The 16 bytes to write to the PICC
		uint8_t bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
										   )
	{
	MFRC522_T::status_t result;

	// Sanity check
	if (buffer == NULL || bufferSize < 16)
		{
		return STATUS_INVALID;
		}

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		{
		return result;
		}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(buffer, bufferSize); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		{
		return result;
		}

	return STATUS_OK;
	} // End MIFARE_Write()

/**
 * Writes a 4 uint8_t page to the active MIFARE Ultralight PICC.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Ultralight_Write(	uint8_t page, 		///< The page (2-15) to write to.
		uint8_t *buffer,	///< The 4 bytes to write to the PICC
		uint8_t bufferSize	///< Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
													  )
	{
	MFRC522_T::status_t result;

	// Sanity check
	if (buffer == NULL || bufferSize < 4)
		{
		return STATUS_INVALID;
		}

	// Build commmand buffer
	uint8_t cmdBuffer[6];
	cmdBuffer[0] = PICC_CMD_UL_WRITE;
	cmdBuffer[1] = page;
	memcpy(&cmdBuffer[2], buffer, 4);

	// Perform the write
	result = PCD_MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		{
		return result;
		}
	return STATUS_OK;
	} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Decrement(	uint8_t blockAddr, ///< The block (0-0xff) number.
		long delta		///< This number is subtracted from the value of block blockAddr.
											   )
	{
	return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
	} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Increment(	uint8_t blockAddr, ///< The block (0-0xff) number.
		long delta		///< This number is added to the value of block blockAddr.
											   )
	{
	return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
	} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Restore(	uint8_t blockAddr ///< The block (0-0xff) number.
											 )
	{
	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
	// Doing only a single step does not work, so I chose to transfer 0L in step two.
	return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
	} // End MIFARE_Restore()

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_TwoStepHelper(	uint8_t command,	///< The command to use
		uint8_t blockAddr,	///< The block (0-0xff) number.
		long data		///< The data to transfer in step 2
												   )
	{
	MFRC522_T::status_t result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.

	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		{
		return result;
		}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(	(uint8_t *)&data, 4, true); // Adds CRC_A and accept timeout as success.
	if (result != STATUS_OK)
		{
		return result;
		}

	return STATUS_OK;
	} // End MIFARE_TwoStepHelper()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_Transfer(	uint8_t blockAddr ///< The block (0-0xff) number.
											  )
	{
	MFRC522_T::status_t result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.

	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		{
		return result;
		}
	return STATUS_OK;
	} // End MIFARE_Transfer()

/**
 * Helper routine to read the current value from a Value Block.
 *
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function.
 *
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[out]  value       Current value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
  */
MFRC522_T::status_t MFRC522_T::MIFARE_GetValue(uint8_t blockAddr, long *value)
	{
	MFRC522_T::status_t status;
	uint8_t buffer[18];
	uint8_t size = sizeof(buffer);

	// Read the block
	status = MIFARE_Read(blockAddr, buffer, &size);
	if (status == STATUS_OK)
		{
		// Extract the value
		*value = (long(buffer[3])<<24) | (long(buffer[2])<<16) | (long(buffer[1])<<8) | long(buffer[0]);
		}
	return status;
	} // End MIFARE_GetValue()

/**
 * Helper routine to write a specific value into a Value Block.
 *
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function.
 *
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[in]   value       New value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::MIFARE_SetValue(uint8_t blockAddr, long value)
	{
	uint8_t buffer[18];

	// Translate the long into 4 bytes; repeated 2x in value block
	buffer[0] = buffer[ 8] = (value & 0xFF);
	buffer[1] = buffer[ 9] = (value & 0xFF00) >> 8;
	buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
	buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
	// Inverse 4 bytes also found in value block
	buffer[4] = ~buffer[0];
	buffer[5] = ~buffer[1];
	buffer[6] = ~buffer[2];
	buffer[7] = ~buffer[3];
	// Address 2x with inverse address 2x
	buffer[12] = buffer[14] = blockAddr;
	buffer[13] = buffer[15] = ~blockAddr;

	// Write the whole data block
	return MIFARE_Write(blockAddr, buffer, 16);
	} // End MIFARE_SetValue()

/**
 * Authenticate with a NTAG216.
 *
 * Only for NTAG216. First implemented by Gargantuanman.
 *
 * @param[in]   passWord   password.
 * @param[in]   pACK       result success???.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PCD_NTAG216_AUTH(uint8_t* passWord, uint8_t pACK[]) //Authenticate with 32bit password
	{
	MFRC522_T::status_t result;
	uint8_t				cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

	cmdBuffer[0] = 0x1B; //Comando de autentificacion

	for (uint8_t i = 0; i<4; i++)
		cmdBuffer[i+1] = passWord[i];

	result = PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);

	if (result!=STATUS_OK)
		{
		return result;
		}

	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq		= 0x30;	// RxIRq and IdleIRq
	//uint8_t cmdBufferSize	= sizeof(cmdBuffer);
	uint8_t validBits		= 0;
	uint8_t rxlength		= 5;
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, 7, cmdBuffer, &rxlength, &validBits);

	pACK[0] = cmdBuffer[0];
	pACK[1] = cmdBuffer[1];

	if (result!=STATUS_OK)
		{
		return result;
		}

	return STATUS_OK;
	} // End PCD_NTAG216_AUTH()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_T::status_t MFRC522_T::PCD_MIFARE_Transceive(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
		uint8_t sendLen,		///< Number of bytes in sendData.
		bool acceptTimeout	///< True => A timeout is also success
													)
	{
	MFRC522_T::status_t result;
	uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen > 16)
		{
		return STATUS_INVALID;
		}

	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != STATUS_OK)
		{
		return result;
		}
	sendLen += 2;

	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	uint8_t cmdBufferSize = sizeof(cmdBuffer);
	uint8_t validBits = 0;
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits);
	if (acceptTimeout && result == STATUS_TIMEOUT)
		{
		return STATUS_OK;
		}
	if (result != STATUS_OK)
		{
		return result;
		}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4)
		{
		return STATUS_ERROR;
		}
	if (cmdBuffer[0] != MF_ACK)
		{
		return STATUS_MIFARE_NACK;
		}
	return STATUS_OK;
	} // End PCD_MIFARE_Transceive()


/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
MFRC522_T::PICC_Type MFRC522_T::PICC_GetType(uint8_t sak		///< The SAK uint8_t returned from PICC_Select().
											)
	{
	// http://www.nxp.com/documents/application_note/AN10833.pdf
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak)
		{
		case 0x04:
			return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:
			return PICC_TYPE_MIFARE_MINI;
		case 0x08:
			return PICC_TYPE_MIFARE_1K;
		case 0x18:
			return PICC_TYPE_MIFARE_4K;
		case 0x00:
			return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:
			return PICC_TYPE_MIFARE_PLUS;
		case 0x01:
			return PICC_TYPE_TNP3XXX;
		case 0x20:
			return PICC_TYPE_ISO_14443_4;
		case 0x40:
			return PICC_TYPE_ISO_18092;
		default:
			return PICC_TYPE_UNKNOWN;
		}
	} // End PICC_GetType()



/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
 */
void MFRC522_T::MIFARE_SetAccessBits(	uint8_t *accessBitBuffer,	///< Pointer to uint8_t 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
										uint8_t g0,				///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
										uint8_t g1,				///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
										uint8_t g2,				///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
										uint8_t g3					///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
									)
	{
	uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
	uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
	uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);

	accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
	accessBitBuffer[1] =          c1 << 4 | (~c3 & 0xF);
	accessBitBuffer[2] =          c3 << 4 | c2;
	} // End MIFARE_SetAccessBits()



/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
* Считывает номер карты, если она есть в поле
* @param Указатель на буфер куда будут положен номер карты
* @return Состояние STATUS_OK - если номер считан, и STATUS_TIMEOUT если карты нет
*/
MFRC522_T::status_t MFRC522_T::MIFARE_read_serial(uint8_t* o)
	{
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	MFRC522_T::status_t result = PICC_RequestA(bufferATQA, &bufferSize);
	if (result == STATUS_OK || result == STATUS_COLLISION)
		{
		result = PICC_Select(&uid);
		if(result == STATUS_OK)
			{
			memcpy(o,uid.uidByte,4);
			}
		}
	return result;
	}
/**
* Отправляет команду формата T=CL I-Block
*/
MFRC522_T::status_t MFRC522_T::tcl_transceive_ib(uint8_t* in, uint8_t in_len, uint8_t* out, uint8_t* out_len, uint8_t cid)
	{
	if(in_len > 60)
		{
		return STATUS_NO_ROOM;
		}
	status_t res;
	uint8_t cmd_len = 2+in_len;
	uint8_t cmd[64] = {(tcl_pcb_block_num)?0x0b:0x0a,cid};  /**< PCD (I-BLOCK) + CID + INF + CRC */
	tcl_pcb_block_num = !tcl_pcb_block_num;
	uint8_t resp[64] = {};
	uint8_t resp_len = sizeof(resp);
	memcpy(&cmd[2],in,in_len);
	if ((PCD_ReadRegister(TxModeReg)&0x80) != 0x80)
		{
		res = PCD_CalculateCRC(cmd,cmd_len,&cmd[cmd_len]);
		if (res != STATUS_OK)
			{
			return res;
			}
		cmd_len += 2;
		}

	res = PCD_TransceiveData(cmd,cmd_len,resp,&resp_len);
	if(res!=STATUS_OK)
		{
		return res;
		}

	if((resp_len-4) > (*out_len))
		{
		return STATUS_NO_ROOM;
		}

	(*out_len) = resp_len-4;
	memcpy(out,&resp[2],resp_len-4); //  -TCL; -CRC
	return STATUS_OK;
	}
/**
* Отправляет комманду RATS
*/
MFRC522_T::status_t MFRC522_T::RATS(uint8_t* out, uint8_t* out_len, uint8_t cid)
	{
	uint8_t rats_cmd[4] = {PICC_CMD_RATS,0x50|(cid&0x0f),0xbc,0xa5};
	uint8_t rats_cmd_len = ((PCD_ReadRegister(TxModeReg)&0x80) != 0x80)?4:2;
	return PCD_TransceiveData(rats_cmd,rats_cmd_len,out,out_len);
	}

/**
* Отправляет комманду mfp_write_perso в T=CL протоколе
*/
MFRC522_T::status_t MFRC522_T::mfp_write_perso(uint8_t* resp, uint8_t* resp_len, uint16_t block,uint8_t* key, uint8_t key_len)
	{
	uint8_t mfp_write_perso_cmd[1+2+16] = {PICC_MFP_mfp_write_perso_CMD,block&0xff,(block>>8)&0xff};
	memcpy(&mfp_write_perso_cmd[1+2],key,key_len);
	return tcl_transceive_ib(mfp_write_perso_cmd,sizeof(mfp_write_perso_cmd),resp,resp_len);
	}


/**
* Отправляет комманду mfp_commit_perso в T=CL протоколе
*/
MFRC522_T::status_t MFRC522_T::mfp_commit_perso(uint8_t* resp, uint8_t* resp_len)
	{
	uint8_t mfp_commit_perso_cmd = PICC_MFP_mfp_commit_perso_CMD;
	return tcl_transceive_ib(&mfp_commit_perso_cmd,sizeof(mfp_commit_perso_cmd),resp,resp_len);
	}

/**
* Сбрасывает аутентификацию для блока
*/
MFRC522_T::status_t MFRC522_T::mfp_reset_auth(uint8_t* resp, uint8_t* resp_len,uint16_t block)
	{
	uint8_t cmd[1+2] = {PICC_MFP_mfp_reset_auth,block&0xff,(block>>8)&0xff};
	return tcl_transceive_ib(cmd,sizeof(cmd),resp,resp_len);
	}

MFRC522_T::status_t MFRC522_T::mfp_first_auth(uint16_t block, uint8_t* key, uint8_t key_len)
	{
	status_t res;
	uint8_t _rndb[17] = {};
	uint8_t rndb_len = sizeof(_rndb);
	uint8_t cmd[] = {PICC_MFP_mfp_first_auth_A,block&0xff,(block>>8)&0xff,0};
	res =  tcl_transceive_ib(cmd,sizeof(cmd),_rndb,&rndb_len);
	if((res != STATUS_OK)||(_rndb[0] != 0x90))
		{
		return res;
		}

	/**
	* Получили E(Kx, _rndb)
	*/
	memset(aes->iv_dec,0,16);
	aes->aes128_dec(key, &_rndb[1],AES_T::CBC);
	uint8_t sec_cmd[33] = {PICC_MFP_AUTH_B};
	memcpy(&sec_cmd[1],rnda,sizeof(rnda)); /**< Установили номер команду и RNDA, хотя оно и должно быть случайным */
	mfp_auth_b_cmd_t* sec_cmd_ptr= (mfp_auth_b_cmd_t*)sec_cmd;

	/**
	* Сдвигаем на один байт влево и получаем _rndb`
	*/
	/**
	* 	rndb_s[15] = r[1];
	memcpy(rndb_s,&r[2],15);
	*/
	sec_cmd_ptr->rndb_s[15] = _rndb[1];
	memcpy(sec_cmd_ptr->rndb_s,&_rndb[2],15); /**< RNB` = rotate_left(RNB,1); */
	/**
	* Теперь шифруем RNDA + _rndb`, сначала А, потом Б с наложенным на него А (CBC режим)
	*	E(Kx, RndA || RndB‟)
	*/
	memset(aes->iv_enc,0,16);
	aes->aes128_enc(key,(uint8_t*)&sec_cmd[1],AES_T::CBC);
	aes->aes128_enc(key,(uint8_t*)&sec_cmd[17],AES_T::CBC);

	/**
	* Где-то там проверяется RNDA
	* там RNDA сдвигается и получается RNDA`;
	* В ответ должен получить нечто, E(Kx, TI || RndA‟ || PICCcap2 || PCDcap2) -> 32 байта
	* либо resp[0] = 6;
	*/

	mfp_mfp_first_auth_resp_b_t rr = {};
	uint8_t rr_len = sizeof(rr);
	res =  tcl_transceive_ib(sec_cmd,sizeof(sec_cmd),(uint8_t*)&rr,&rr_len);

	if((res != STATUS_OK)||(rr.status != MFP_STATUS_OK ))
		{
		if(rr.status == MFP_AUTH_ERROR)
			{
			return STATUS_AUTH_ERROR;
			}
		return res;
		}

	memset(aes->iv_dec,0,16);
	aes->aes128_dec(key, &rr.raw[0],AES_T::CBC);
	aes->aes128_dec(key, &rr.raw[16],AES_T::CBC);
	//if rnda_s == rnda

	memcpy(ti,rr.ti,sizeof(ti));
	memcpy(picc_cap,rr.picc_cap,sizeof(picc_cap));
	memcpy(pcd_cap,rr.pcd_cap,sizeof(pcd_cap));
	w_ctr = r_ctr = 0;
	memcpy(rndb,&_rndb[1],sizeof(rndb));
	gen_session_key(key);
	gen_mac_key(key);

	return res;
	}

/**
* http://www.gorferay.com/aes-128-key-diversification-example/
* Generation of Session Key for Encryption
* Session Key for Encryption KENC
* Расчет сессионного ключа -> [0] = 0x11; [1..5] = rnda[7..11] ^ rndb[7..11]; [6..10] = rndb[0..4]; [11..15] = rnda[0..4]; !вверх ногами

* sess[0..4] = rnda[11..15]
* sess[5..10] = rndb[11..15]
* sess[11..15] = rnda[4..8]^rndb[4..8];
* session = aes128(k,session);
*/
void MFRC522_T::gen_session_key(uint8_t* key)
	{
	session_key[15] = 0x11;
	for(uint8_t i = 0; i<5; i++)
		{
		session_key[i+10] = rnda[i+4] ^ rndb[i+4];
		session_key[i+5] = rndb[i+11];
		session_key[i] = rnda[i+11];
		}
	aes->aes128_enc(key,session_key);
	}
/**
* Generation of Session Key for calculating Message Authentication Code
* Session Key for MAC KMAC
* [0] = 0x22; [1..5] = rnda[11..15] ^ rndb[11..15]; [6..10] = rndb[4..8]; [11..15] = rnda[4..8]
*/
void MFRC522_T::gen_mac_key(uint8_t* key)
	{
	mac_key[15] = 0x22;
	for(uint8_t i = 0; i<5; i++)
		{
		mac_key[i+10] = rnda[i] ^ rndb[i];
		mac_key[i+5] = rndb[i+7];
		mac_key[i] = rnda[i+7];
		}
	aes->aes128_enc(key,mac_key);
	}
/**
* Последующая авторизация
*/
MFRC522_T::status_t MFRC522_T::mfp_following_auth(uint16_t block, uint8_t* key, uint8_t key_len)
	{
	status_t res;
	uint8_t _rndb[17] = {};
	uint8_t rndb_len = sizeof(_rndb);
	uint8_t cmd[] = {PICC_MFP_mfp_following_auth,block&0xff,(block>>8)&0xff};
	res =  tcl_transceive_ib(cmd,sizeof(cmd),_rndb,&rndb_len);
	if((res != STATUS_OK)||(_rndb[0] != 0x90))
		{
		return res;
		}

	/**
	* Получили E(Kx, _rndb)
	* IV = Concatenation of TI, 3x R_Ctr and 3x W_Ctr
	* IV = {ti[0..3],r_cnt[0..1],w_cnt[0..1],r_cnt[0..1],w_cnt[0..1],r_cnt[0..1],w_cnt[0..1]}
	*
	*/
	aes->iv_dec[15] = ti[3];
	aes->iv_dec[14] = ti[2];
	aes->iv_dec[13] = ti[1];
	aes->iv_dec[12] = ti[0];
	aes->iv_dec[11] = aes->iv_dec[7] =  aes->iv_dec[3] = (r_ctr&0xff);
	aes->iv_dec[10] = aes->iv_dec[6] =  aes->iv_dec[2] = (r_ctr>>8);
	aes->iv_dec[9] = aes->iv_dec[5] =  aes->iv_dec[1] = (w_ctr&0xff);
	aes->iv_dec[8] = aes->iv_dec[4] =  aes->iv_dec[0] = (w_ctr>>8);

	aes->aes128_dec(key, &_rndb[1],AES_T::CBC);

	uint8_t sec_cmd[33] = {PICC_MFP_AUTH_B};
	memcpy(&sec_cmd[1],rnda,sizeof(rnda)); /**< Установили номер команду и RNDA, хотя оно и должно быть случайным */
	mfp_auth_b_cmd_t* sec_cmd_ptr= (mfp_auth_b_cmd_t*)sec_cmd;


	/**
	* Сдвигаем на один байт влево и получаем _rndb`
	*/

	sec_cmd_ptr->rndb_s[15] = _rndb[1];
	memcpy(sec_cmd_ptr->rndb_s,&_rndb[2],15); /**< RNB` = rotate_left(RNB,1); */
	/**
	* Теперь шифруем RNDA + _rndb`, сначала А, потом Б с наложенным на него А (CBC режим)
	*	E(Kx, RndA || RndB‟)
	* IV = Same as IV for message transfer. (?)
	*/
	aes->iv_enc[15] = ti[3];
	aes->iv_enc[14] = ti[2];
	aes->iv_enc[13] = ti[1];
	aes->iv_enc[12] = ti[0];
	aes->iv_enc[11] = aes->iv_enc[7] =  aes->iv_enc[3] = (r_ctr&0xff);
	aes->iv_enc[10] = aes->iv_enc[6] =  aes->iv_enc[2] = (r_ctr>>8);
	aes->iv_enc[9] = aes->iv_enc[5] =  aes->iv_enc[1] = (w_ctr&0xff);
	aes->iv_enc[8] = aes->iv_enc[4] =  aes->iv_enc[0] = (w_ctr>>8);

	aes->aes128_enc(key,(uint8_t*)&sec_cmd[1],AES_T::CBC);
	aes->aes128_enc(key,(uint8_t*)&sec_cmd[17],AES_T::CBC);

	/**
	* Где-то там проверяется RNDA
	* там RNDA сдвигается и получается RNDA`
	* В ответ должен получить нечто, E(Kx, TI || RndA‟ || PICCcap2 || PCDcap2) -> 32 байта
	* либо resp[0] = 6;
	*/

	mfp_mfp_first_auth_resp_b_t rr = {};
	uint8_t rr_len = sizeof(rr);
	res =  tcl_transceive_ib(sec_cmd,sizeof(sec_cmd),(uint8_t*)&rr,&rr_len);

	if((res != STATUS_OK)||(rr.status != MFP_STATUS_OK ))
		{
		if(rr.status == MFP_AUTH_ERROR)
			{
			return STATUS_AUTH_ERROR;
			}
		return res;
		}
	memcpy(rndb,&_rndb[1],sizeof(rndb));
	gen_session_key(key);
	gen_mac_key(key);

	return res;
	}

/*
 * Rol
 * Description: Rotate Byte left. (Move MSByte over to LSByte position, and
 *              shift up other bytes)
 *              Ex: Rol {0,1,2,3,4} = {1,2,3,4,0}
 *
 * Arguments:   data = pointer to data block [modified]
 *              len  = length of data block
 * Return:      None
 *
 * Operation:   Copy first byte; shift all other bytes down one index, and place
 *              first byte in now empty last slot.
 *
 * Assumptions: MSByte is at index 0
 *
 * Revision History:
 *   Jan. 03, 2012      Nnoduka Eruchalu     Initial Revision
 *   May  03, 2013      Nnoduka Eruchalu     Updated Comments
 */
void MFRC522_T::rol(uint8_t *data, size_t len)
	{
	size_t i; /* index into data */
	uint8_t first = data[0];
	for (i = 0; i < len - 1; i++)
		{
		data[i] = data[i + 1];
		}
	data[len - 1] = first;
	}

/*
 * Lsl
 * Description: Logical Bit Shift Left. Shift MSB out, and place a 0 at LSB
 *              position
 *              Ex: LSL {5,1,2,3,4} = {1,2,3,4,0}
 *
 * Arguments:   data = pointer to data block [modified]
 *              len  = length of data block
 *
 * Operation:   For each byte position, shift out highest bit with
 *              (data[n] << 1) and add in the highest bit from next lower byte
 *              with (data[n+1] >> 7)
 *              The addition is done with a bitwise OR (|)
 *              For the Least significant byte however, there is no next lower
 *              byte so this is handled last and simply set to data[len-1] <<= 1
 *
 * Assumptions: MSByte is at index 0
 *
 * Revision History:
 *   Jan. 03, 2012      Nnoduka Eruchalu     Initial Revision
 *   May  03, 2013      Nnoduka Eruchalu     Updated Comments
 */
void MFRC522_T::lsl(uint8_t *data, size_t len)
	{
	size_t n; /* index into data */
	for (n = 0; n < len - 1; n++)
		{
		data[n] = (data[n] << 1) | (data[n + 1] >> 7);
		}
	data[len - 1] <<= 1;
	}

/**
* Генерирует под-ключи
* Steps: 1.  Let L = CIPHK(0b).
* 2. If MSB1(L) = 0, then K1 = L << 1;  Else K1 = (L << 1) ^ Rb; see Sec. 5.3 for the definition of Rb.
* 3. If MSB1(K1) = 0, then K2 = K1 << 1;  Else K2 = (K1 << 1) ^ Rb.  7
* 4.  Return K1, K2
*/
void MFRC522_T::mfp_gen_subkeys(uint8_t *key, uint8_t *k1, uint8_t *k2)
	{
	uint8_t L[16] = {0};
	/**
	* 1. Let L = CIPHK(0b).
	*/
	aes->aes128_enc(key, L);
	/**
	* 2. If MSB1(L) = 0, then K1 = L << 1;  Else K1 = (L << 1) ^ Rb; see Sec. 5.3 for the definition of Rb.
	*/
	memcpy(k1, L, sizeof(L));
	lsl(k1);
	if ((L[0] & 0x80) == 0x80)
		{
		k1[15] ^= 0x87;
		}
	/**
	* 3. If MSB1(K1) = 0, then K2 = K1 << 1;  Else K2 = (K1 << 1) ^ Rb.  7
	*/
	memcpy(k2, k1, sizeof(L));
	lsl(k2);
	if ((k1[0] & 0x80) == 0x80)
		{
		k2[15] ^= 0x87;
		}
	}
/**
* Prerequisites: block cipher CIPH with block size b; key K; MAC length parameter Tlen.
* Input:  message M of bit length Mlen.
* Output: MAC T of bit length Tlen.
* Suggested Notation: CMAC(K, M, Tlen) or, if Tlen is understood from the context, CMAC(K, M).
* Steps:
* 1.  Apply the subkey generation process in Sec. 6.1 to K to produce K1 and K2. (mfp_gen_subkeys())
* 2.  If Mlen = 0, let n = 1; else, let n = ªMlen/bº.
* 3.  Let M1, M2, ... , Mn-1, Mn* denote the unique sequence of bit strings such that M = M1 || M2 || ... || Mn-1 || Mn*, where M1, M2,..., Mn-1 are complete blocks.2
* 4.  If Mn* is a complete block, let Mn = K1  Mn*; else, let Mn = K2  (Mn*||10j), where j = nb-Mlen-1.
* 5.  Let C0 = 0b.
* 6.  For i = 1 to n, let Ci= CIPHK(Ci-1  Mi).
* 7.  Let T = MSBTlen(Cn).
* 8.  Return T.
*/
void MFRC522_T::mfp_mac_gen(uint8_t *msg, uint16_t msg_len, uint8_t *key, uint8_t *mac)
	{
	uint8_t k1[16] = {};
	uint8_t k2[16] = {};
	mfp_gen_subkeys(key, k1, k2);
	uint16_t n = (msg_len) / 16; /**< Количество блоков */

	if ((!msg_len) || (msg_len % 16))
		{
		/**
		* Блок не полный
		*/
		memcpy(mac, &msg[n * 16], msg_len % 16);
		memset(&mac[msg_len % 16], 0, 16 - (msg_len % 16));
		mac[msg_len % 16] = 0x80;
		for (uint8_t i = 0; i < 16; i++)
			{
			mac[i] = mac[i] ^ k2[i];
			}
		}
	else
		{
		/**
		* Блок полный
		*/
		for (uint8_t i = 0; i < 16; i++)
			{
			mac[i] = msg[((n - 1) * 16) + i] ^ k1[i];
			}
		n--;
		}
	uint8_t iv[16] = {};
	for (uint16_t i = 0; i < n; i++)
		{
		uint8_t tmp[16] = {};
		memcpy(tmp, &msg[i * 16], sizeof(tmp));
		aes->aes128_enc(key, tmp, AES_T::CBC,iv);
		}
	aes->aes128_enc(key, mac, AES_T::CBC,iv);
	}
/**
* считывает блок с Plus карты, предпологается что авторизация уже пройдена
* В оригинале надо еще проверять CMAC, но мне влом
*
* @param Номер блока
* @param Данные для возврата
* @param Размер данных
*/
MFRC522_T::status_t MFRC522_T::mfp_read_block(uint8_t block, uint8_t* data, uint8_t* d_len)
	{
	uint8_t mac_cmd[] = {PICC_MFP_READ_PLAIN, r_ctr&0xff, r_ctr>>8,ti[0],ti[1],ti[2],ti[3],block&0xff,(block>>8)&0xff,1};
	uint8_t mac[16] = {};
	mfp_mac_gen(mac_cmd,sizeof(mac_cmd),mac_key,mac);
	uint8_t cmd[] = {PICC_MFP_READ_PLAIN,block&0xff,(block>>8)&0xff,1,mac[1],mac[3],mac[5],mac[7],mac[9],mac[11],mac[13],mac[15]};

	uint8_t resp[25] = {};
	uint8_t resp_len = sizeof(resp);
	status_t res =  tcl_transceive_ib(cmd,sizeof(cmd),resp,&resp_len);
	if((res != STATUS_OK)||(resp[0] != 0x90))
		{
		return res;
		}
	memcpy(data,&resp[1],16);
	r_ctr++;
	return res;
	}

/**
* Записывает блок на карту
*
* Назад возвращается CMAC который было бы не плохо проверять, CMAC считается от 0x90 + w_ctr + ti
* @param Номер блока
* @param Данные на запись
* @param Размер
*/
MFRC522_T::status_t MFRC522_T::mfp_write_block(uint8_t block, uint8_t* data, uint8_t d_len)
	{
	if(d_len>16)
		{
		return STATUS_NO_ROOM;
		}
	uint8_t d[16] = {0};
	uint8_t mac_cmd[25] = {PICC_MFP_WRITE_PLAIN, w_ctr&0xff, w_ctr>>8,ti[0],ti[1],ti[2],ti[3],block&0xff,(block>>8)&0xff};
	memcpy(&mac_cmd[9],data,d_len);
	memcpy(d,data,d_len);

	uint8_t mac[16] = {};
	mfp_mac_gen(mac_cmd,sizeof(mac_cmd),mac_key,mac);
	uint8_t cmd[] = {PICC_MFP_WRITE_PLAIN,block&0xff,(block>>8)&0xff,
			d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[8],d[9],d[10],d[11],d[12],d[13],d[14],d[15],
			mac[1],mac[3],mac[5],mac[7],mac[9],mac[11],mac[13],mac[15]};

	uint8_t resp[9] = {};
	uint8_t resp_len = sizeof(resp);
	status_t res =  tcl_transceive_ib(cmd,sizeof(cmd),resp,&resp_len);
	if((res != STATUS_OK)||(resp[0] != 0x90))
		{
		return res;
		}
	w_ctr++;
	return res;
	}
/**
* Задержка в d вызовов delay_cb()
* @param Время задеркжи, исчесляется в количестве вызовов delay_cb в сек.
* По-умолчанию 1 вызов = 1/490 сек.
*/
void MFRC522_T::delay(uint8_t d)
	{
	delay_cnt = 0;
	while(delay_cnt<=d);
	}

/**
* Функция обслуживания задержек
* По-умолчанию 1 вызов = 1/490 сек.
*/
void MFRC522_T::delay_cb()
	{
	delay_cnt++;
	}
