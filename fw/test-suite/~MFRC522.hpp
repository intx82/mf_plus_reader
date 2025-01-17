#ifndef MFRC522_T_hpp
#define MFRC522_T_hpp

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "spi.hpp"
#include "aes.hpp"

#define NULL 0
#pragma anon_unions

class MFRC522_T
	{
	public:
		// MFRC522_T registers. Described in chapter 9 of the datasheet.
		// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
		enum PCD_Register
			{
			// Page 0: Command and status
			//						  0x00			// reserved for future use
			CommandReg				= 0x01 << 1,	// starts and stops command execution
			ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
			DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
			ComIrqReg				= 0x04 << 1,	// interrupt request bits
			DivIrqReg				= 0x05 << 1,	// interrupt request bits
			ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed
			Status1Reg				= 0x07 << 1,	// communication status bits
			Status2Reg				= 0x08 << 1,	// receiver and transmitter status bits
			FIFODataReg				= 0x09 << 1,	// input and output of 64 byte FIFO buffer
			FIFOLevelReg			= 0x0A << 1,	// number of bytes stored in the FIFO buffer
			WaterLevelReg			= 0x0B << 1,	// level for FIFO underflow and overflow warning
			ControlReg				= 0x0C << 1,	// miscellaneous control registers
			BitFramingReg			= 0x0D << 1,	// adjustments for bit-oriented frames
			CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
			//						  0x0F			// reserved for future use

			// Page 1: Command
			// 						  0x10			// reserved for future use
			ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving
			TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
			RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
			TxControlReg			= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
			TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
			TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
			RxSelReg				= 0x17 << 1,	// selects internal receiver settings
			RxThresholdReg			= 0x18 << 1,	// selects thresholds for the bit decoder
			DemodReg				= 0x19 << 1,	// defines demodulator settings
			// 						  0x1A			// reserved for future use
			// 						  0x1B			// reserved for future use
			MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
			MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
			// 						  0x1E			// reserved for future use
			SerialSpeedReg			= 0x1F << 1,	// selects the speed of the serial UART interface

			// Page 2: Configuration
			// 						  0x20			// reserved for future use
			CRCResultRegH			= 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
			CRCResultRegL			= 0x22 << 1,
			// 						  0x23			// reserved for future use
			ModWidthReg				= 0x24 << 1,	// controls the ModWidth setting?
			// 						  0x25			// reserved for future use
			RFCfgReg				= 0x26 << 1,	// configures the receiver gain
			GsNReg					= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
			CWGsPReg				= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
			ModGsPReg				= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
			TModeReg				= 0x2A << 1,	// defines settings for the internal timer
			TPrescalerReg			= 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
			TReloadRegH				= 0x2C << 1,	// defines the 16-bit timer reload value
			TReloadRegL				= 0x2D << 1,
			TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
			TCounterValueRegL		= 0x2F << 1,

			// Page 3: Test Registers
			// 						  0x30			// reserved for future use
			TestSel1Reg				= 0x31 << 1,	// general test signal configuration
			TestSel2Reg				= 0x32 << 1,	// general test signal configuration
			TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
			TestPinValueReg			= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
			TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
			AutoTestReg				= 0x36 << 1,	// controls the digital self test
			VersionReg				= 0x37 << 1,	// shows the software version
			AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
			TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
			TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
			TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels
									  // 						  0x3C			// reserved for production tests
									  // 						  0x3D			// reserved for production tests
									  // 						  0x3E			// reserved for production tests
									  // 						  0x3F			// reserved for production tests
			};

		// MFRC522_T commands. Described in chapter 10 of the datasheet.
		enum PCD_Command
			{
			PCD_Idle				= 0x00,		// no action, cancels current command execution
			PCD_Mem					= 0x01,		// stores 25 bytes into the internal buffer
			PCD_GenerateRandomID	= 0x02,		// generates a 10-byte random ID number
			PCD_CalcCRC				= 0x03,		// activates the CRC coprocessor or performs a self test
			PCD_Transmit			= 0x04,		// transmits data from the FIFO buffer
			PCD_NoCmdChange			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
			PCD_Receive				= 0x08,		// activates the receiver circuits
			PCD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
			PCD_MFAuthent 			= 0x0E,		// performs the MIFARE standard authentication as a reader
			PCD_SoftReset			= 0x0F		// resets the MFRC522_T
			};

		// MFRC522_T RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
		// Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/MFRC522_T.pdf
		enum PCD_RxGain
			{
			RxGain_18dB				= 0x00 << 4,	// 000b - 18 dB, minimum
			RxGain_23dB				= 0x01 << 4,	// 001b - 23 dB
			RxGain_18dB_2			= 0x02 << 4,	// 010b - 18 dB, it seems 010b is a duplicate for 000b
			RxGain_23dB_2			= 0x03 << 4,	// 011b - 23 dB, it seems 011b is a duplicate for 001b
			RxGain_33dB				= 0x04 << 4,	// 100b - 33 dB, average, and typical default
			RxGain_38dB				= 0x05 << 4,	// 101b - 38 dB
			RxGain_43dB				= 0x06 << 4,	// 110b - 43 dB
			RxGain_48dB				= 0x07 << 4,	// 111b - 48 dB, maximum
			RxGain_min				= 0x00 << 4,	// 000b - 18 dB, minimum, convenience for RxGain_18dB
			RxGain_avg				= 0x04 << 4,	// 100b - 33 dB, average, convenience for RxGain_33dB
			RxGain_max				= 0x07 << 4		// 111b - 48 dB, maximum, convenience for RxGain_48dB
			};

		// Commands sent to the PICC.
		enum PICC_Command
			{
			// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
			PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
			PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
			PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
			PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
			PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
			PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
			PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
			// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
			// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
			// The read/write commands can also be used for MIFARE Ultralight.
			PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
			PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
			PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
			PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
			PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
			PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
			PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
			PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
			// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
			// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
			PICC_CMD_UL_WRITE		= 0xA2,		// Writes one 4 byte page to the PICC.
			PICC_CMD_RATS = 0xe0, /**< RATS */
			};

		// MIFARE constants that does not fit anywhere else
		enum MIFARE_Misc
			{
			MF_ACK					= 0xA,		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
			MF_KEY_SIZE				= 6			// A Mifare Crypto1 key is 6 bytes.
			};

		// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
		// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
		enum PICC_Type
			{
			PICC_TYPE_UNKNOWN		,
			PICC_TYPE_ISO_14443_4	,	// PICC compliant with ISO/IEC 14443-4
			PICC_TYPE_ISO_18092		, 	// PICC compliant with ISO/IEC 18092 (NFC)
			PICC_TYPE_MIFARE_MINI	,	// MIFARE Classic protocol, 320 bytes
			PICC_TYPE_MIFARE_1K		,	// MIFARE Classic protocol, 1KB
			PICC_TYPE_MIFARE_4K		,	// MIFARE Classic protocol, 4KB
			PICC_TYPE_MIFARE_UL		,	// MIFARE Ultralight or Ultralight C
			PICC_TYPE_MIFARE_PLUS	,	// MIFARE Plus
			PICC_TYPE_TNP3XXX		,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
			PICC_TYPE_NOT_COMPLETE	= 0xff	// SAK indicates UID is not complete.
			};

		// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
		// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
		enum status_t
			{
			STATUS_OK				,	// Success
			STATUS_ERROR			,	// Error in communication
			STATUS_COLLISION		,	// Collission detected
			STATUS_TIMEOUT			,	// Timeout in communication.
			STATUS_NO_ROOM			,	// A buffer is not big enough.
			STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
			STATUS_INVALID			,	// Invalid argument.
			STATUS_CRC_WRONG		,	// The CRC_A does not match
			STATUS_AUTH_ERROR		,	/**< Ошибка авторизации */
			STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
			};

		// A struct used for passing the UID of a PICC.
		typedef struct
			{
			uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
			uint8_t		uidByte[10];
			uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
			} uid_t;

		// A struct used for passing a MIFARE Crypto1 key
		typedef struct
			{
			uint8_t		keyByte[MF_KEY_SIZE];
			} MIFARE_Key;

		// Member variables
		uid_t uid;								// Used by PICC_ReadCardSerial().

		// Size of the MFRC522_T FIFO
		static const uint8_t FIFO_SIZE = 64;		// The FIFO is 64 bytes.

		/////////////////////////////////////////////////////////////////////////////////////
		// Basic interface functions for communicating with the MFRC522_T
		/////////////////////////////////////////////////////////////////////////////////////
		void PCD_WriteRegister(uint8_t reg, uint8_t value);
		void PCD_WriteRegister(uint8_t reg, uint8_t count, uint8_t *values);
		uint8_t PCD_ReadRegister(uint8_t reg);
		void PCD_ReadRegister(uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign = 0);
		void setBitMask(unsigned char reg, unsigned char mask);
		void PCD_SetRegisterBitMask(uint8_t reg, uint8_t mask);
		void PCD_ClearRegisterBitMask(uint8_t reg, uint8_t mask);
		status_t PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result);

		/////////////////////////////////////////////////////////////////////////////////////
		// Functions for manipulating the MFRC522_T
		/////////////////////////////////////////////////////////////////////////////////////
		status_t  PCD_Init();
		status_t PCD_Reset();
		void PCD_AntennaOn();
		void PCD_AntennaOff();
		uint8_t PCD_GetAntennaGain();
		void PCD_SetAntennaGain(uint8_t mask);

		/////////////////////////////////////////////////////////////////////////////////////
		// Functions for communicating with PICCs
		/////////////////////////////////////////////////////////////////////////////////////
		status_t PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits = NULL, uint8_t rxAlign = 0, bool checkCRC = false);
		status_t PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData = NULL, uint8_t *backLen = NULL, uint8_t *validBits = NULL, uint8_t rxAlign = 0, bool checkCRC = false);
		status_t PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
		status_t PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize);
		status_t PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize);
		status_t PICC_Select(uid_t *uid, uint8_t validBits = 0);
		status_t PICC_HaltA();

		/////////////////////////////////////////////////////////////////////////////////////
		// Functions for communicating with MIFARE PICCs
		/////////////////////////////////////////////////////////////////////////////////////
		status_t PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, uid_t *uid);
		void PCD_StopCrypto1();
		status_t MIFARE_Read(uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize);
		status_t MIFARE_Write(uint8_t blockAddr, uint8_t *buffer, uint8_t bufferSize);
		status_t MIFARE_Ultralight_Write(uint8_t page, uint8_t *buffer, uint8_t bufferSize);
		status_t MIFARE_Decrement(uint8_t blockAddr, long delta);
		status_t MIFARE_Increment(uint8_t blockAddr, long delta);
		status_t MIFARE_Restore(uint8_t blockAddr);
		status_t MIFARE_Transfer(uint8_t blockAddr);
		status_t MIFARE_GetValue(uint8_t blockAddr, long *value);
		status_t MIFARE_SetValue(uint8_t blockAddr, long value);
		status_t PCD_NTAG216_AUTH(uint8_t *passWord, uint8_t pACK[]);

		/////////////////////////////////////////////////////////////////////////////////////
		// Support functions
		/////////////////////////////////////////////////////////////////////////////////////
		status_t PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, bool acceptTimeout = false);
		// old function used too much memory, now name moved to flash; if you need char, copy from flash to memory
		//const char *GetStatusCodeName(uint8_t code);

		static PICC_Type PICC_GetType(uint8_t sak);
		// old function used too much memory, now name moved to flash; if you need char, copy from flash to memory
		//const char *PICC_GetTypeName(uint8_t type);


		// Support functions for debuging
		void PCD_DumpVersionToSerial();
		void PICC_DumpToSerial(uid_t *uid);
		void PICC_DumpDetailsToSerial(uid_t *uid);
		void PICC_DumpMifareClassicToSerial(uid_t *uid, PICC_Type piccType, MIFARE_Key *key);
		void PICC_DumpMifareClassicSectorToSerial(uid_t *uid, MIFARE_Key *key, uint8_t sector);
		void PICC_DumpMifareUltralightToSerial();

		// Advanced functions for MIFARE
		void MIFARE_SetAccessBits(uint8_t *accessBitBuffer, uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3);
		bool MIFARE_OpenUidBackdoor(bool logErrors);
		bool MIFARE_SetUid(uint8_t *newUid, uint8_t uidSize, bool logErrors);
		bool MIFARE_UnbrickUidSector(bool logErrors);

		enum picc_tcl_cmd_t
			{
			PICC_MFP_READ_PLAIN = 0x33,
			PICC_MFP_mfp_first_auth_A = 0x70,
			PICC_MFP_AUTH_B = 0x72,
			PICC_MFP_mfp_following_auth = 0x76,
			PICC_MFP_mfp_reset_auth = 0x78,
			PICC_MFP_WRITE_PLAIN = 0xa3,
			PICC_MFP_mfp_write_perso_CMD = 0xa8,
			PICC_MFP_mfp_commit_perso_CMD = 0xaa
			};

		enum mfp_status_t
			{
			MFP_STATUS_OK = 0x90,
			MFP_AUTH_ERROR = 0x06,
			MFP_WRONG_SECTOR = 0x09,
			MFP_FORMAT_ERROR = 0x0c,
			MFP_GENERIC_ERROR = 0x0f,
			};

		/**
		* Структура запроса на часть Б первоначальной аутентификации
		*/
		struct mfp_auth_b_cmd_t
			{
			uint8_t cmd;
			uint8_t rnda[16];
			uint8_t rndb_s[16];
			};
		/**
		* Структура ответа на на часть Б первоначальной аутентификации (после расшифровки)
		*/
		struct mfp_mfp_first_auth_resp_b_t
			{
			mfp_status_t status;
			union 
				{
				struct 
					{
					uint8_t ti[4];
					uint8_t rnda_s[16];
					uint8_t picc_cap[6];
					uint8_t pcd_cap[6];
					};
				uint8_t raw[32];
				};
			};
		
		uint8_t ti[4]; /**< Номер транзакции, выдается после авторизации */
		uint8_t picc_cap[6]; /**< Возможности карты (ставится при инициализации карты (mfp_write_perso b000))*/
		uint8_t pcd_cap[6]; /**< Возможности передатчика */
		uint16_t r_ctr; /**< Счетчик выполненных команд чтения */
		uint16_t w_ctr; /**< Счетчик выполненных команды записи */
		static const uint8_t rnda[]; /**< По стандарту RNDA должно быть рандомайзным, и каждый раз разным. но.. */
		uint8_t rndb[16]; /**< Случаыйное число от PICC */
		uint8_t session_key[16]; /**< Смешиванием RNDA с RNDB дает сессионный ключ */
		uint8_t mac_key[16]; /**< Смешивая немного по другому получаем ключ MAC */

		/**
		* Функция передачи с TC=L I-Block (information)
		* @param Данные для передачи в карту
		* @param Длина этих данных
		* @param Данные отданные картой (без обертки T=CL и CRC)
		* @param Размер принятых данных от карты и максимальный размер который можно принять
		* @param CID карты, по-умолчанию 0
		*/
		status_t tcl_transceive_ib(uint8_t* in, uint8_t in_len, uint8_t* out, uint8_t* out_len, uint8_t cid = 0);
		/**
		* Отправляет RATS
		* @param Ответ ATS
		* @param Максимальный размер данных, сюда же будет возвращен размер ATS
		* @param Устанавливает CID карты, по-умолчанию 0
		*/
		status_t RATS(uint8_t* out, uint8_t* out_len, uint8_t cid = 0);

		/**
		* Команда mfp_write_perso
		* Записывает ключ на карту
		* Формат комманды 0xa8 [Номер блока] [Ключ]
		* @param Поле для ответа (должен быть 1 байт) 0x90 - ОК, все остальное жопа
		* @param Максимальный размер ответа, он же размер ответа после запроса
		* @param Номер блока
		* @param Ключ 16 байт AES128
		* @param Размер ключа, должен быть 16 байт, но мало-ли..
		*/
		status_t mfp_write_perso(uint8_t* resp,uint8_t* resp_len, uint16_t block,uint8_t* key, uint8_t key_len = 16);

		/**
		* Команда Commit Perso
		* Устанавливает ключи и номер уровня доступа намертво
		* @param Поле для ответа (должен быть 1 байт) 0x90 - ОК, все остальное жопа
		* @param Максимальный размер ответа, он же размер ответа после запроса
		*/
		status_t mfp_commit_perso(uint8_t* resp, uint8_t* resp_len);

		/**
		* Команда Reset Auth
		* Устанавливает ключи и номер уровня доступа намертво
		* @param Поле для ответа (должен быть 1 байт) 0x90 - ОК, все остальное жопа
		* @param Максимальный размер ответа, он же размер ответа после запроса
		* @param Номер блока
		*/
		status_t mfp_reset_auth(uint8_t* resp, uint8_t* resp_len,uint16_t block);

		/**
		* Считывает номер карты, если она есть в поле
		* @param Указатель на буфер куда будут положен номер карты
		* @return Состояние STATUS_OK - если номер считан, и STATUS_TIMEOUT если карты нет
		*/
		status_t MIFARE_read_serial(uint8_t* o);

		/**
		* Аутентификация, первый раз.
		* @param Номер блока
		* @param Ключ
		* @param Размер ключа (по-умолчанию 16)
		*/
		status_t mfp_first_auth(uint16_t block, uint8_t* key, uint8_t key_len = 16);
		/**
		* Последующая авторизация
		* @param Номер блока
		* @param Ключ
		* @param Размер ключа (по-умолчанию 16)
		*/
		status_t mfp_following_auth(uint16_t block, uint8_t* key, uint8_t key_len = 16);
		/**
		* Генерирует сессионный ключ на основне RNDA RNDB из аутентификации
		*
		* Generation of Session Key for Encryption
		* Session Key for Encryption KENC
		* Расчет сессионного ключа -> [0] = 0x11; [1..5] = rnda[7..11] ^ rndb[7..11]; [6..10] = rndb[0..4]; [11..15] = rnda[0..4];
		* session = aes128(k,session);
		* @param Ключ для блока ( к котрому производилась аутентификация)
		*
		*/
		void gen_session_key(uint8_t* key);
		
		/**
		* Генерирует MAC ключ на основе RNDA RNDB
		* Generation of Session Key for calculating Message Authentication Code
		* Session Key for MAC KMAC
		* [0] = 0x22; [1..5] = rnda[11..15] ^ rndb[11..15]; [6..10] = rndb[4..8]; [11..15] = rnda[4..8]
		* @param Ключ для блока ( к котрому производилась аутентификация)
		*/
		void gen_mac_key(uint8_t* key);
		
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
		void rol(uint8_t *data, size_t len = 16);

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
		void lsl(uint8_t *data, size_t len = 16);
		/**
		* Генерирует под-ключи 
		* Steps: 1.  Let L = CIPHK(0b). 
		* 2. If MSB1(L) = 0, then K1 = L << 1;  Else K1 = (L << 1) ^ Rb; see Sec. 5.3 for the definition of Rb. 
		* 3. If MSB1(K1) = 0, then K2 = K1 << 1;  Else K2 = (K1 << 1) ^ Rb.  7 
		* 4.  Return K1, K2
		*/
		void mfp_gen_subkeys(uint8_t *key, uint8_t *k1, uint8_t *k2);
		/**
		* Генерирует MAC к сообщению
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
		void mfp_mac_gen(uint8_t *msg, uint16_t msg_len, uint8_t *key, uint8_t *mac);

		/**
		* считывает блок с Plus карты, предпологается что авторизация уже пройдена
		* В оригинале надо еще проверять CMAC, но мне влом
		*
		* @param Номер блока
		* @param Данные для возврата
		* @param Размер данных
		*/
		status_t mfp_read_block(uint8_t block_num, uint8_t* data,uint8_t* d_len);
		
		/**
		* Записывает блок на карту
		*
		* Назад возвращается CMAC который было бы не плохо проверять, CMAC считается от 0x90 + w_ctr + ti
		* @param Номер блока
		* @param Данные на запись
		* @param Размер
		*/
		status_t mfp_write_block(uint8_t block, uint8_t* data, uint8_t d_len);
		/**
		* Задержка в d вызовов delay_cb()
		* @param Время задеркжи, исчесляется в количестве вызовов delay_cb в сек.
		* По-умолчанию 1 вызов = 1/490 сек.
		*/
		void delay_cb();
		/**
		* Функция обслуживания задержек
		* По-умолчанию 1 вызов = 1/490 сек.
		*/
		void delay(uint8_t d);
		SPI_T* SPI;
		AES_T* aes;
		MFRC522_T(SPI_T* s, AES_T* a)
			{
			SPI = s;
			aes = a;
			tcl_pcb_block_num = false;

			}
			
	private:
		volatile uint32_t delay_cnt;
		status_t MIFARE_TwoStepHelper(uint8_t command, uint8_t blockAddr, long data);
		void xor_data(uint8_t* a, uint8_t* b, uint8_t a_len = 16, uint8_t b_len = 16);
		bool tcl_pcb_block_num;

	};

#endif
