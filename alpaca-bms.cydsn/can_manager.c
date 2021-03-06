#include "can_manager.h"

#include "cell_interface.h"

volatile uint8_t can_buffer[8];
extern BAT_PACK_t bat_pack;
extern volatile uint8_t CAN_DEBUG;

/* Data Frame format for Voltage and Temperature
The datatype consists of three bytes:
1. Identifying Number (eg cell #1)
2. upper byte of data
3. lower byte of data
*/



void can_send_temp(uint8_t high_temp0,
    uint8_t high_temp1,
    uint8_t high_temp2,
    uint8_t high_temp3,
    uint8_t high_temp4,
    uint8_t high_temp5,
    uint8_t high_tempNode,
    uint8_t high_temp)
{
    can_buffer[0] = (high_temp0/10)<<4 | (high_temp0%10);
    can_buffer[1] = (high_temp1/10)<<4 | (high_temp1%10);
    can_buffer[2] = (high_temp2/10)<<4 | (high_temp2%10);
    can_buffer[3] = (high_temp3/10)<<4 | (high_temp3%10);
    can_buffer[4] = (high_temp4/10)<<4 | (high_temp4%10);
    can_buffer[5] = (high_temp5/10)<<4 | (high_temp5%10);
    can_buffer[6] = 0xff & high_tempNode;
    can_buffer[7] = (high_temp/10)<<4 | (high_temp%10);
            
	CAN_1_SendMsgtemp();
    CyDelay(5);

        

} // can_send_temp()



void can_send_volt(
    uint16_t min_voltage,
    uint16_t max_voltage,
    uint32_t pack_voltage)
{
    //max and min voltage means the voltage of single cell
        can_buffer[0] = HI8(min_voltage);
        can_buffer[1] = LO8(min_voltage);;

        can_buffer[2] = HI8(max_voltage);
        can_buffer[3] = LO8(max_voltage);

        can_buffer[4] = 0xFF & (pack_voltage >> 24);
        can_buffer[5] = 0xFF & (pack_voltage >> 16);
        can_buffer[6] = 0xFF & (pack_voltage >> 8);
        can_buffer[7] = 0xFF & (pack_voltage);


        CAN_1_SendMsgvolt();
        CyDelay(1);

} // can_send_volt()



void can_send_current()
{
    int16_t battery_current = bat_pack.current;
	can_buffer[0] = (battery_current<0 ? 1:0);
	can_buffer[1] = 0xFF & (uint16)(abs(battery_current)>>8); // upper byte
    can_buffer[2] = 0xFF & abs(battery_current); // lower byte
    can_buffer[3] = 0x00; 
    can_buffer[4] = 0x00; 
    can_buffer[5] = (battery_current<0 ? 1:0); 
    can_buffer[6] = 0xff & (((abs(battery_current)/1000%10)<<4) | (0x0f & (abs(battery_current)/100%10))); 
    can_buffer[7] = 0xff & (((uint8)abs(battery_current)/10%10) << 4 | abs(battery_current)%10); 
	CAN_1_SendMsgcurrent();
} // can_send_current()


void can_send_status(uint8_t name,
                    uint8_t SOC_P,
                    BMS_STATUS status,
                    uint8_t stack,
                    uint8_t cell,
                    uint16_t value16){
//8 SOC Percent
//8 AH used since full charge
//16 BMS Status bits (error flags)
//16 Number of charge cycles
//16 Pack balance (delta) mV
    can_buffer[0] = name;
    can_buffer[1] = (uint8_t)(SOC_P/10)<<4 | (uint8_t)(SOC_P%10);
    can_buffer[2] = HI8(status);
    can_buffer[3] = LO8(status);
    can_buffer[4] = stack & 0xFF;
    can_buffer[5] = (cell) & 0xFF;
    can_buffer[6] = HI8(value16);
    can_buffer[7] = LO8(value16);

    CAN_1_SendMsgstatus();
}

                    

                    
uint8_t can_test_send()
{
    uint16_t ID=0x205;
    uint8_t msg[8];
    //msg[0~3] is left
    msg[0]=0x0f;
    msg[1]=0xf0;
    msg[2]=0x0f;
    msg[3]=0xf0;
    //msg[4~7] is right
    msg[4]=0x9a;
    msg[5]=0xbc;
    msg[6]=0xde;
    msg[7]=0xff;
    
    
    
	uint8_t i, state;
	CAN_1_TX_MSG message;
	CAN_1_DATA_BYTES_MSG payload;

	message.id = ID; // edit for testing
	message.rtr = 0;
	message.ide = 0;
	message.dlc = 0x08;
	message.irq = 1;
	message.msg = &payload;

	for(i=0;i<8;i++)
		payload.byte[i] = msg[i];

	state = CAN_1_SendMsg(&message);

/*
	if(state != CYRET_SUCCESS)
	{
		LED_Write(1);
		return CAN_1_FAIL;
	}

	LED_Write(0);
	return CYRET_SUCCESS;
*/
	
	return state;
} // can_test_send()


void can_init()
{
	CAN_1_GlobalIntEnable(); // CAN Initialization
	CAN_1_Init();
	CAN_1_Start();
} // can_init()
