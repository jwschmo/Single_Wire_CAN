/*
  Test.h - Test library for Wiring - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// include core Wiring API
#include "WProgram.h"

// include this library's description file
#include "CANInterface.h"

// include description files for other libraries used (if any)
#include "HardwareSerial.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances


CANInterface CAN;



// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries


/* ---------------------------------------------------------------------------- */
void CANInterface::CAN_Int(void)
{
	uint8_t status = read_status(SPI_RX_STATUS);
	uint8_t addr;
	uint8_t t;
	tCAN BuffTemp;
	unsigned long TSTemp;

	unread_message++;
	if (bit_is_set(status,6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
	}
	else if (bit_is_set(status,7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
	}
	//*TSEnd = micros();
	TSTemp = micros();

	digitalWrite(CAN_CS,LOW);
	SPI.transfer(addr);

	// read id
	//BuffEnd->id  = (uint16_t) SPI.transfer(0xFF) << 3;
	//BuffEnd->id |=            SPI.transfer(0xFF) >> 5;

	BuffTemp.id  = (uint16_t) SPI.transfer(0xFF) << 3;
	BuffTemp.id |=            SPI.transfer(0xFF) >> 5;


	SPI.transfer(0xFF);
	SPI.transfer(0xFF);

	// read DLC
	uint8_t length = SPI.transfer(0xff) & 0x0f;

	//BuffEnd->header.length = length;
	//BuffEnd->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;

	BuffTemp.header.length = length;
	BuffTemp.header.rtr = (bit_is_set(status, 3)) ? 1 : 0;


	// read data
	for (t=0;t<length;t++) {

		//BuffEnd->data[t] = SPI.transfer(0xFF);
		BuffTemp.data[t] = SPI.transfer(0xFF);
	}
	digitalWrite(CAN_CS,HIGH);

	// clear interrupt flag
	if (bit_is_set(status, 6)) {
		bit_modify(CANINTF, (1<<RX0IF), 0);
	}
	else {
		bit_modify(CANINTF, (1<<RX1IF), 0);
	}


	//checks to see if ID is masked before adding to the buffer.
	if(!is_mask(BuffTemp.id))
	{
	*BuffEnd = BuffTemp;
	*TSEnd = TSTemp;

	//increment buffers by 1
	BuffEnd++;
	if(BuffEnd > &CANbuffer[49])
	{
		BuffEnd = CANbuffer;
	}
	TSEnd++;
	if(TSEnd > &TS[49])
	{
		TSEnd = TS;
	}

	// check for buffer overflow
	if(unread_message > 50)
	{
		unread_message = 50;
		BuffStart++;
		TSStart++;
		if(BuffStart > &CANbuffer[49])
			{
				BuffStart = CANbuffer;
			}
		if(TSStart > &TS[49])
			{
				TSStart = TS;
			}
	}
	}




}

/* ---------------------------------------------------------------------------- */

uint8_t CANInterface::get_INT_message(tCAN *message, unsigned long *timestamp)
{
message = BuffStart;
timestamp = TSStart;


if(unread_message > 0)
{
	unread_message--;
	BuffStart++;
	TSStart++;
	if(BuffStart > &CANbuffer[49])
		{
			BuffStart = CANbuffer;
		}
	if(TSStart > &TS[49])
		{
			TSStart = TS;
		}
	return true;
}
else if(unread_message <= 0)
{
	return false;
}


}

/* ---------------------------------------------------------------------------- */


void CANInterface::write_register( uint8_t address, uint8_t data )
{
	digitalWrite(CAN_CS,LOW);

	SPI.transfer(SPI_WRITE);
	SPI.transfer(address);
	SPI.transfer(data);

	digitalWrite(CAN_CS,HIGH);
}


/* ---------------------------------------------------------------------------- */

uint8_t CANInterface::read_register(uint8_t address)
{
	uint8_t data;

	digitalWrite(CAN_CS,LOW);

	SPI.transfer(SPI_READ);
	SPI.transfer(address);

	data = SPI.transfer(0xFF);

	digitalWrite(CAN_CS,HIGH);

	return data;
}

/* ---------------------------------------------------------------------------- */

// check if there are any new messages waiting

uint8_t CANInterface::check_message(void) {
	return (!digitalRead(MCP2515_INT));
}

/* ---------------------------------------------------------------------------- */


void CANInterface::bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
	digitalWrite(CAN_CS,LOW);

	SPI.transfer(SPI_BIT_MODIFY);
	SPI.transfer(address);
	SPI.transfer(mask);
	SPI.transfer(data);

	digitalWrite(CAN_CS,HIGH);
}

/* ---------------------------------------------------------------------------- */


uint8_t CANInterface::read_status(uint8_t type)
{
	uint8_t data;

	digitalWrite(CAN_CS,LOW);

	SPI.transfer(type);
	data = SPI.transfer(0xFF);

	digitalWrite(CAN_CS,HIGH);

	return data;
}

/* ---------------------------------------------------------------------------- */


bool CANInterface::Init(int speed)
{
BuffStart = CANbuffer;
BuffEnd = CANbuffer;
unread_message = 0;

switch(speed) {

	case 11: /* Baudrate of 33.3 kbps*/

		for(int i=0; i< 2048;i++)
		{
			CANMask[i] = 0;
		}

		//attachInterrupt(0,CAN_Int,RISING);

		pinMode(CAN_CS, OUTPUT);
		digitalWrite(CAN_CS, LOW);  /*Enables CAN SPI*/
		SPI.transfer(SPI_RESET);
		digitalWrite(CAN_CS, HIGH);	/*Disables CAN SPI*/

		delay(10);

		digitalWrite(CAN_CS,LOW);
		SPI.transfer(SPI_WRITE);
		SPI.transfer(CNF3);

		SPI.transfer((1<<PHSEG21)|(1<<PHSEG22));  //Bitrate 250kbps at 16 MHz.  	write to CNF3
		SPI.transfer((1<<BTLMODE)|(1<<PHSEG10)|(1<<PHSEG2)|(1<<PHSEG12)|(1<<PHSEG0));	//	write to CNF2 fix to go back to 500 kbps
		SPI.transfer((1<<SJW1)|speed);							//	write to CNF1


		//activate interrupts
		SPI.transfer((1<<RX1IE)|(1<<RX0IE));

		digitalWrite(CAN_CS, HIGH);

		if (read_register(CNF1) != ((1<<SJW1)|speed))
			{
				digitalWrite(LED2,HIGH);

				return false;
			}
		// deactivate the RXnBF Pins (High Impedance State)
		write_register(BFPCTRL, 0);

		// set TXnRTS as inputs
		write_register(TXRTSCTRL, 0);

		// turn off filters => receive any message
		write_register(RXB0CTRL, (1<<RXM1)|(1<<RXM0));
		write_register(RXB1CTRL, (1<<RXM1)|(1<<RXM0));

		// reset device to normal mode
		write_register(CANCTRL, 0);
		break;


	default:

	for(int i=0; i< 2048;i++)
	{
		CANMask[i] = 0;
	}

	//attachInterrupt(0,CAN_Int,RISING);

	pinMode(CAN_CS, OUTPUT);
	digitalWrite(CAN_CS, LOW);  /*Enables CAN SPI*/
	SPI.transfer(SPI_RESET);
	digitalWrite(CAN_CS, HIGH);	/*Disables CAN SPI*/

	delay(10);

	digitalWrite(CAN_CS,LOW);
	SPI.transfer(SPI_WRITE);
	SPI.transfer(CNF3);

	SPI.transfer((1<<PHSEG21));  //Bitrate 250kbps at 16 MHz.  	write to CNF3
	SPI.transfer((1<<BTLMODE)|(1<<PHSEG11)|(1<<PHSEG2)); 				//	write to CNF2 fix to go back to 500 kbps
	SPI.transfer((1<<SJW1)|speed);							//	write to CNF1


	//activate interrupts
	SPI.transfer((1<<RX1IE)|(1<<RX0IE));

	digitalWrite(CAN_CS, HIGH);

	if (read_register(CNF1) != ((1<<SJW1)|speed))
		{
			digitalWrite(LED2,HIGH);

			return false;
		}
	// deactivate the RXnBF Pins (High Impedance State)
	write_register(BFPCTRL, 0);

	// set TXnRTS as inputs
	write_register(TXRTSCTRL, 0);

	// turn off filters => receive any message
	write_register(RXB0CTRL, (1<<RXM1)|(1<<RXM0));
	write_register(RXB1CTRL, (1<<RXM1)|(1<<RXM0));

	// reset device to normal mode
	write_register(CANCTRL, 0);
	break;
	}

	return true;


}

/* ---------------------------------------------------------------------------- */

// check if there is a free buffer to send messages

uint8_t CANInterface::check_free_buffer(void)
{
	uint8_t status = read_status(SPI_READ_STATUS);

	if ((status & 0x54) == 0x54) {
		// all buffers used
		return false;
	}

	return true;
}

/* ---------------------------------------------------------------------------- */


uint8_t CANInterface::get_message(tCAN *message, unsigned long *timestamp)
{
	// read status
	uint8_t status = read_status(SPI_RX_STATUS);
	uint8_t addr;
	uint8_t t;
	if (bit_is_set(status,6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
	}
	else if (bit_is_set(status,7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
	}
	else {
		// Error: no message available
		return 0;
	}
	*timestamp = micros();

	digitalWrite(CAN_CS,LOW);
	SPI.transfer(addr);

	// read id
	message->id  = (uint16_t) SPI.transfer(0xFF) << 3;
	message->id |=            SPI.transfer(0xFF) >> 5;

	SPI.transfer(0xFF);
	SPI.transfer(0xFF);

	// read DLC
	uint8_t length = SPI.transfer(0xff) & 0x0f;

	message->header.length = length;
	message->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;

	// read data
	for (t=0;t<length;t++) {
		message->data[t] = SPI.transfer(0xFF);
	}
	digitalWrite(CAN_CS,HIGH);

	// clear interrupt flag
	if (bit_is_set(status, 6)) {
		bit_modify(CANINTF, (1<<RX0IF), 0);
	}
	else {
		bit_modify(CANINTF, (1<<RX1IF), 0);
	}

	return (status & 0x07) + 1;
}

/* ---------------------------------------------------------------------------- */


uint8_t CANInterface::send_message(tCAN *message)
{
	uint8_t status = read_status(SPI_READ_STATUS);

	/* Statusbyte:
	 *
	 * Bit	Function
	 *  2	TXB0CNTRL.TXREQ
	 *  4	TXB1CNTRL.TXREQ
	 *  6	TXB2CNTRL.TXREQ
	 */
	uint8_t address;
	uint8_t t;
//	SET(LED2_HIGH);
	if (bit_is_clear(status, 2)) {
		address = 0x00;
	}
	else if (bit_is_clear(status, 4)) {
		address = 0x02;
	}
	else if (bit_is_clear(status, 6)) {
		address = 0x04;
	}
	else {
		// all buffer used => could not send message
		return 0;
	}

	digitalWrite(CAN_CS,LOW);
	SPI.transfer(SPI_WRITE_TX | address);

	SPI.transfer(message->id >> 3);
    SPI.transfer(message->id << 5);

	SPI.transfer(0);
	SPI.transfer(0);

	uint8_t length = message->header.length & 0x0f;

	if (message->header.rtr) {
		// a rtr-frame has a length, but contains no data
		SPI.transfer((1<<RTR) | length);
	}
	else {
		// set message length
		SPI.transfer(length);

		// data
		for (t=0;t<length;t++) {
			SPI.transfer(message->data[t]);
		}
	}
	digitalWrite(CAN_CS,HIGH);

	delayMicroseconds(1);

	// send message
	digitalWrite(CAN_CS,LOW);
	address = (address == 0) ? 1 : address; /* This kinda coding just pisses me off don't do this....*/
	SPI.transfer(SPI_RTS | address);
	digitalWrite(CAN_CS,HIGH);

	return address;
}

//sets the mask for the CAN ID
//Address is masked if value is 1
//Address is not masked if value is 0
bool CANInterface::mask_address(uint16_t id,bool mask)
{
	if(id < 0x800)
	{
		CANMask[id] = mask;
		return 0;
	}
	else
	{
		return 1;
	}
}

//Retrieves the current mask for the ID
bool CANInterface::is_mask(uint16_t id)
{
	if(id < 0x800)
	{
		return CANMask[id];
	}
	else
	{
		return 1;
	}
}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library



