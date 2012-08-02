/*
  Test.h - Test library for Wiring - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef CANInterface_h
#define CANInterface_h

// include types & constants of Wiring core API
#include "WConstants.h"
#include "mcp2515_defs.h"
#include "SPI.h"
#include "global.h"
#include <inttypes.h>

#define LED2 8
#define LED3 7

#define	MCP2515_INT 	2

#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps
#define	CANSPEED_95		14		// CAN speed at 95.238 kbps
#define CANSPEED_33		11		// CAN speed at 33.3 kbps  # of TQ's 20, BRP 9+1


#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE			0x11

#define PID_REQUEST         0x7DF
#define PID_REPLY			0x7E8

#define CAN_CS				53

typedef struct CANmessage
{
	uint16_t id;
	struct {
		int8_t rtr : 1;
		uint8_t length : 4;
	} header;
	uint8_t data[8];
} tCAN;

// library interface description
class CANInterface
{
  // user-accessible "public" interface
  public:
    //CANInterface(void);
    bool Init(int);
    void CAN_Int(void);
    void doSomething(void);
    void write_register( uint8_t address, uint8_t data );
    uint8_t read_register(uint8_t address);
    uint8_t check_message(void) ;
    void bit_modify(uint8_t address, uint8_t mask, uint8_t data);
    uint8_t read_status(uint8_t type);
    uint8_t check_free_buffer(void);
    uint8_t get_message(tCAN *message, unsigned long *timestamp);
    uint8_t get_INT_message(tCAN *message, unsigned long *timestamp);
    uint8_t send_message(tCAN *message);
    bool mask_address(uint16_t id, bool mask);
    bool is_mask(uint16_t id);

  // library-accessible "private" interface
  private:
    int value;
    //this really should be a structure, but I'm lazy
    int8_t unread_message;
    CANmessage CANbuffer[50];
    CANmessage *BuffStart;
    CANmessage *BuffEnd;
    unsigned long TS[50];
    unsigned long *TSStart;
    unsigned long *TSEnd;
    //end of should be structure
    bool CANMask[2048];
};

extern CANInterface CAN;

#endif

