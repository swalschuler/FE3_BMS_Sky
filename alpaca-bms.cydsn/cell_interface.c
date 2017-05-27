/* LICENSE
	cell_interface.h and cell_interface.c are derivatives of source from Linear
	Technology Corp.(LTC)
*/

/************************************

***********************************************************/



#include "cell_interface.h"
#include "current_sense.h"
#include "LTC68041.h"
#include "math.h"

#include <stdlib.h>

uint8_t fatal_err;

// FE3 new structure
BAT_CELL_t bat_cell[N_OF_CELL];
BAT_TEMP_t bat_temp[N_OF_TEMP];
volatile BAT_ERR_t bat_err;
BAT_NODE_t bat_node[N_OF_NODE];
BAT_STACK_t bat_stack[N_OF_STACK];
BAT_PACK_t bat_pack;

extern volatile uint8_t CAN_DEBUG;
volatile uint8_t bad_therm=0;
volatile BAT_ERR_t bat_err_array[100];
volatile uint8_t bat_err_index;
volatile uint8_t bat_err_index_loop;
uint32_t deltaTime;
uint32_t lastTime;


/**
 * @initialize. In case need to setup anything, write them in it.
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void  bms_init(){
    //setup SS pin
    SS_SetDriveMode(SS_DM_RES_UP);
    LTC68_Start();
    LTC6804_initialize();
    LTC6804_wrcfg(TOTAL_IC,tx_cfg);
}


/**
 * @initialize the mypack struct. 
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void mypack_init(){
    uint8_t stack=0;
    uint8_t cell=0;
    uint8_t node=0;
    uint8_t temp=0;
    bat_err_index = 0;
    bat_err_index_loop = 0;

    // FE3 new data structure
    // initialize cells and temps
    // with ordered to help debugging
    cell = 0;
    for (cell = 0; cell<N_OF_CELL;cell++){
        bat_cell[cell].voltage = cell;
        bat_cell[cell].bad_counter = 0;
    }
    temp = 0;
    for (temp = 0; temp < N_OF_TEMP ; temp++){
        bat_temp[temp].temp_c = (uint8_t)temp;
        bat_temp[temp].temp_raw = (uint16_t)temp;
        bat_temp[temp].bad_counter = 0;
        bat_temp[temp].bad_type = 0;
        
        if ((temp%10) < 5){
            bat_temp[temp].type = THERM_BOARD;
        }
        else{
            bat_temp[temp].type = THERM_CELL;
        }
    }

    // register node
    node = 0;
    for (node = 0; node < N_OF_NODE; node++){
        cell = 0;
        temp = 0;
        for (cell = 0; cell < 14; cell++){    
            bat_node[node].cells[cell] = &(bat_cell[node*14+cell]);
        }
        for (temp = 0; temp < 10; temp++){
            bat_node[node].temps[temp] = &(bat_temp[node*10+temp]);
        }
        bat_node[node].over_temp = 0;
        bat_node[node].under_temp = 0;
        bat_node[node].over_voltage = 0;
        bat_node[node].under_voltage = 0;
    }
    // register stack
    stack = 0;
    for (stack = 0;stack < N_OF_STACK; stack++){
        bat_stack[stack].nodes[0] = &(bat_node[stack*2]);
        bat_stack[stack].nodes[1] = &(bat_node[stack*2+1]);
        bat_stack[stack].voltage = stack;
    }
    // register pac
    stack = 0;
    for (stack = 0; stack< 3; stack++){
        bat_pack.stacks[stack] = &(bat_stack[stack]);
    }
    for (node = 0; node<6; node++){
        bat_pack.nodes[node] = &(bat_node[node]);
    }
    
    // get SOC
    uint32_t temp_SOC = SOC_Store_ReadByte(0x00)<<24;
    temp_SOC |= SOC_Store_ReadByte(0x01)<<16;
    temp_SOC |= SOC_Store_ReadByte(0x02)<<8;
    temp_SOC |= SOC_Store_ReadByte(0x03);
    
    if(temp_SOC<SOC_SOC_LOW || temp_SOC > SOC_FULL){
        temp_SOC = SOC_NOMIAL;
    }
    bat_pack.current_charge = temp_SOC;
    
    //give the pack non-0 value help debuging CAN
    bat_pack.voltage = 12;
    bat_pack.current = 34;
    bat_pack.fuse_fault = 0;
    bat_pack.status = 0; 
    bat_pack.health = NORMAL;
    bat_pack.SOC_cali_flag =0;
    bat_pack.HI_temp_c = 0;
    bat_pack.HI_temp_raw = 0;
    bat_pack.HI_voltage = 0;
    bat_pack.LO_voltage = 0;
    bat_pack.time_stamp = 0;
    bat_pack.SOC_percent = 0;
    
    
}




/**
 * @wake all BMSs up, waiting for commands
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void  wake_up(){
    wakeup_sleep();
}

void check_cfg(){
    //DEBUG_UART_PutString("Enter Check_CFG\n");
    int i=0;
    wakeup_sleep();
    LTC6804_rdcfg(TOTAL_IC,rx_cfg);
    //LCD_Position(1u,0u);
    for (i=0;i<8;i++){
        if (rx_cfg[i] != tx_cfg[i]){
              fatal_err = COM_FAILURE;
            return;
        }
    }
}



void check_chips(){
    
}// check_chips()



uint8_t get_cell_volt(){
    LTC68_ClearFIFO();
   // DEBUG_UART_PutString("Enter GET_CELL_VOLT\n");
    int error;
    wakeup_sleep();
    LTC6804_adcv();
    CyDelay(10);
    wakeup_sleep();
    error = LTC6804_rdcv(0, TOTAL_IC,cell_codes); // Set to read back all cell voltage registers
    if (error == -1)
    {
        #ifdef DEBUG_LCD
            LCD_Position(0u,10u);
            LCD_PrintString("ERROR");
        #endif
       return 1;
    }
    
    //get information
    update_volt(cell_codes);
    
    
    //check error
    check_volt();

    
    return 0;
}// get_cell_volt()


uint8_t get_cell_temp(){
    int error;
    wakeup_sleep();
    LTC6804_adax();
    CyDelay(3);  
    wakeup_sleep();
    uint8_t redundant=3;
    while (redundant>0){   
        //read three time at most
        error = LTC6804_rdaux(0,TOTAL_IC,aux_codes); // Set to read back all aux registers
        //DEBUG
        error=0;
        if (error == 0)
        {
            break;
        }
        redundant-=1;
    }
    
    if (redundant <= 0){
        #ifdef DEBUG_LCD
        LCD_Position(0u,10u);
        LCD_PrintString("ERROR");
        #endif
        bat_pack.health = FAULT;
        return 1;
    }

    //get information
    update_temp(aux_codes);

    //check error
    check_temp();
   
   
    #ifdef DEBUG_LCD
        LCD_Position(1u,10u);
        print_cells(aux_codes[0][0]);
        LCD_Position(0u,10u);
        LCD_PrintString("OK");
    #endif
    return 0;
}// get_cell_temp()



uint8_t check_cells(){ 
    // not in use
    //using ADOW
  uint16_t cell_pu[TOTAL_IC][12];
  uint16_t cell_pd[TOTAL_IC][12];
  int error;
  uint8_t i_IC=0;
  uint8_t i_cell=0;

  wakeup_sleep();

  LTC6804_adow(ADOW_PUP_UP);
  error = LTC6804_rdcv(0, TOTAL_IC,cell_pu); // Set to read back all cell voltage registers

  wakeup_sleep();

  LTC6804_adow(ADOW_PUP_DOWN);
  error = LTC6804_rdcv(0, TOTAL_IC,cell_pd); // Set to read back all cell voltage registers

  if (error==-1){
    return 1;
    }

  for (i_IC=0;i_IC<TOTAL_IC;i_IC++){
    for (i_cell=0;i_cell<12;i_cell++){
      if ((((int16_t)cell_pu[i_IC][i_cell+1]-(int16_t)cell_pd[i_IC][i_cell+1]) < -400) && (CELL_ENABLE&(0x1<<i_cell))){
        fatal_err |= CELL_VOLT_UNDER;
        //LCD_Position(1u,0u);
        //LCD_PrintString("big ");
        return 1;
      }
      if (cell_pu[i_IC][0]==0){
        fatal_err |= CELL_VOLT_UNDER;
        //LCD_Position(1u,0u);
        //LCD_PrintString("eq 0");
        return 1;
      }
      if (cell_pd[i_IC][11]==0){
        fatal_err |= CELL_VOLT_UNDER;
        return 1;
      }
    }

  }
    return 0;

}// check_cells()


void update_volt(uint16_t cell_codes[TOTAL_IC][12]){
    uint8_t cell=0;
    uint8_t raw_cell=0;
    uint8_t node = 0;
    uint8_t ic=0;
    uint8_t stack=0;
    uint32_t temp_volt;

    // 2/9/2017 Added voltage offset for the first and last cell of each node

    for (ic=0;ic<TOTAL_IC;ic++){
        for (raw_cell=0;raw_cell<12;raw_cell++){
            if ((CELL_ENABLE & (0x1<<raw_cell))){
                int temp = cell % (N_OF_CELL / N_OF_NODE);       
                if (temp == 0 || temp == 13)
                    cell_codes[ic][raw_cell] += VOLTAGE_READING_OFFSET*10;
                cell++;            
            }
        }
    } 
    cell = 0;
    /*
    int i = 0;
    for (i = 0; i < 84; i++){
        int temp = i % (N_OF_CELL / N_OF_NODE);
        if (temp == 0 || temp == 13)
            bat_cell[i].voltage += VOLTAGE_READING_OFFSET;
    }
    */
    
    //log in voltage data
    for (ic=0;ic<TOTAL_IC;ic++){
        for (raw_cell=0;raw_cell<12;raw_cell++){
            if ((CELL_ENABLE & (0x1<<raw_cell))){
                bat_cell[cell].voltage = cell_codes[ic][raw_cell]/10;  //only record in mV not in 0.1mV
                cell++;
            }
        }
    }

    // voltage_compensation();
    
    // FE3 update 3 stacks voltage
    node = 0;
    cell = 0;
    stack = 0;
    temp_volt = 0;
    for (stack = 0; stack<3; stack++){
        temp_volt = 0;
        for (node = 0; node<2; node++){
            for (cell = 0; cell<14; cell++){
                temp_volt += (uint32_t)(bat_stack[stack].nodes[node]->cells[cell]->voltage);
            }
        }
        bat_stack[stack].voltage = temp_volt;
    }

    // FE3 update pack voltage
    stack = 0;
    temp_volt = 0;
    for (stack = 0; stack<3; stack++){
        temp_volt += bat_pack.stacks[stack]->voltage;
    }
    bat_pack.voltage = temp_volt/3;

}

void check_volt(){
    uint8_t cell = 0;
    uint8_t node = 0;
    uint16_t voltage16 = 0;

    // update each cell
    for (cell = 0; cell<N_OF_CELL; cell++){
        voltage16 = bat_cell[cell].voltage;
        if (voltage16 > (uint16_t)OVER_VOLTAGE){
            bat_cell[cell].bad_counter++;
            bat_cell[cell].bad_type = 1;
        }else if (voltage16 < (uint16_t)UNDER_VOLTAGE){
            bat_cell[cell].bad_counter++;
            bat_cell[cell].bad_type = 0;
        }else{
            if (bat_cell[cell].bad_counter>0){
                bat_cell[cell].bad_counter--;
            }           
        }
    }

    // update node
    for (node = 0; node< N_OF_NODE; node++){
        for (cell = 0; cell<14; cell++){
            if (bat_node[node].cells[cell]->bad_counter > ERROR_VOLTAGE_LIMIT){
                if (bat_node[node].cells[cell]->bad_type == 0){
                    bat_node[node].under_voltage |= (1u<<cell);
                }else{
                    bat_node[node].over_voltage |= (1u<<cell);
                }
            }
        }
    }

    // update pack of cell voltage error
    for (node = 0; node < N_OF_NODE; node++){
        if (bat_pack.nodes[node]->over_voltage != 0){
            bat_pack.status |= CELL_VOLT_OVER;
            bat_err_add(CELL_VOLT_OVER, bat_node[node].over_voltage, node);
        }

        if (bat_pack.nodes[node]->under_voltage != 0){
            bat_pack.status  |= CELL_VOLT_UNDER;
            bat_err_add(CELL_VOLT_UNDER, bat_node[node].under_voltage, node);
        }
    }
    
    // find the max voltage and mini vltage
    uint16_t max_voltage = bat_cell[0].voltage;
    uint16_t min_voltage = bat_cell[0].voltage;
    cell=0;
    for(cell=0;cell<N_OF_CELL;cell++){
        if (max_voltage<bat_cell[cell].voltage){
            max_voltage = bat_cell[cell].voltage;
        }
        if(min_voltage>bat_cell[cell].voltage){
            min_voltage = bat_cell[cell].voltage;
        }
    }
    
    bat_pack.HI_voltage = max_voltage;
    bat_pack.LO_voltage = min_voltage;
    
        
}

                


void update_temp(uint16_t aux_codes[TOTAL_IC][6]){
    uint8_t ic=0;
    uint16_t temp;
    uint8_t i=0;



    // FE3 new data structure
    // update node
    temp = 0;
    for (ic = 0; ic < 12; ic++){
        for (i = 0; i < 5; i++){
            bat_temp[temp].temp_raw = aux_codes[ic][i];
            bat_temp[temp].temp_ref = aux_codes[ic][5];
            bat_temp[temp].temp_c = (uint8_t)temp_transfer(aux_codes[ic][i], aux_codes[ic][5]);

            temp++;
        }
    }
    
    

}


void check_temp(){
    uint8_t temp=0;
    uint8_t node=0;
    uint16_t temp_c=0;
    
    // ignore the known broken thermistors
    //    if (node == 3 || node == 17 || node == 27){
    
/*  Tony's Old ones
    bat_temp[3].temp_raw = bat_temp[2].temp_raw;
    bat_temp[3].temp_c = bat_temp[2].temp_c;
    
    bat_temp[17].temp_raw = bat_temp[16].temp_raw;
    bat_temp[17].temp_c = bat_temp[16].temp_c;
    
    bat_temp[27].temp_raw = bat_temp[26].temp_raw;
    bat_temp[27].temp_c = bat_temp[26].temp_c;
    
    bat_temp[51].temp_raw = bat_temp[50].temp_raw;
    bat_temp[51].temp_c = bat_temp[50].temp_c;
    
    bat_temp[57].temp_raw = bat_temp[56].temp_raw;
    bat_temp[57].temp_c = bat_temp[56].temp_c;
*/    
    
    /*
    // Sirius's button cell test bench ignore
    bat_temp[7].temp_raw = bat_temp[6].temp_raw;
    bat_temp[7].temp_c = bat_temp[6].temp_c;
    
    bat_temp[11].temp_raw = bat_temp[10].temp_raw;
    bat_temp[11].temp_c = bat_temp[10].temp_c;
    
    bat_temp[17].temp_raw = bat_temp[10].temp_raw;
    bat_temp[17].temp_c = bat_temp[10].temp_c;
    
    bat_temp[23].temp_raw = bat_temp[22].temp_raw;
    bat_temp[23].temp_c = bat_temp[22].temp_c;
    */
    
    // 2/4/2016 Measurement on actual battery pack
    // 0,7,20,21,27,30,33 are dead
    bat_temp[0].temp_raw = bat_temp[1].temp_raw;
    bat_temp[0].temp_c = bat_temp[1].temp_c;
    
    bat_temp[7].temp_raw = bat_temp[6].temp_raw;
    bat_temp[7].temp_c = bat_temp[6].temp_c;
    
    bat_temp[20].temp_raw = bat_temp[22].temp_raw;
    bat_temp[20].temp_c = bat_temp[22].temp_c;
    
    bat_temp[21].temp_raw = bat_temp[22].temp_raw;
    bat_temp[21].temp_c = bat_temp[22].temp_c;
    
    bat_temp[27].temp_raw = bat_temp[28].temp_raw;
    bat_temp[27].temp_c = bat_temp[28].temp_c;

    bat_temp[30].temp_raw = bat_temp[31].temp_raw;
    bat_temp[30].temp_c = bat_temp[31].temp_c;
    
    bat_temp[33].temp_raw = bat_temp[34].temp_raw;
    bat_temp[33].temp_c = bat_temp[34].temp_c;
    
    // check temp
    for (node = 0; node<N_OF_TEMP; node++){
        temp_c = bat_temp[node].temp_c;
        if (temp_c > (uint8_t)CRITICAL_TEMP_H){
            //if over temperature
            bat_temp[node].bad_counter++;
            bat_temp[node].bad_type = 1;
        }else if (temp_c < (uint8_t)CRITICAL_TEMP_L){
            // if under temperature
            bat_temp[node].bad_counter++;
            bat_temp[node].bad_type = 0;
        }else{
            //if there is no error
            if (bat_temp[node].bad_counter>0){
                bat_temp[node].bad_counter--;
            }           
        }
    }

    // update node
    for (node = 0; node< N_OF_NODE; node++){
        for (temp = 0; temp<10; temp++){
            if (bat_node[node].temps[temp]->bad_counter > ERROR_TEMPERATURE_LIMIT){
                if (bat_node[node].temps[temp]->bad_type == 0){
                    bat_node[node].under_temp |= (1u<<temp);
                }else{
                    bat_node[node].over_temp |= (1u<<temp);
                }
            }
        }
    }
    
    // update temperature highest to each node
    node = 0;
    uint8_t temp_temp=0;
    uint8_t i=0;
    for (node=0;node<N_OF_NODE;node++){
        temp_temp = bat_pack.nodes[node]->temps[0]->temp_c;
        bat_pack.nodes[node]->high_temp = temp_temp;
        for (i=1;i<10;i++){
            if (temp_temp < bat_pack.nodes[node]->temps[i]->temp_c){
                temp_temp = bat_pack.nodes[node]->temps[i]->temp_c;
                bat_pack.nodes[node]->high_temp = temp_temp;
            }
        }
    }

    // Update the battery_pack highest temperature
    bat_pack.HI_temp_c = bat_temp[0].temp_c;
    bat_pack.HI_temp_raw = bat_temp[0].temp_raw;
    for (i=1;i<N_OF_TEMP;i++){
        if (bat_temp[i].temp_c > bat_pack.HI_temp_c){
            bat_pack.HI_temp_c = bat_temp[i].temp_c;
            bat_pack.HI_temp_raw = bat_temp[i].temp_raw;
            bat_pack.HI_temp_node = i/10;
        }    
    }
    
    
    // update pack of temp error
    for (node = 0; node < N_OF_NODE; node++){
        if (bat_pack.nodes[node]->over_temp != 0){
            bat_pack.status |= PACK_TEMP_OVER;
            bat_err_add(PACK_TEMP_OVER, bat_node[node].over_temp, node);
        }

        if (bat_pack.nodes[node]->under_temp != 0){
            bat_pack.status  |= PACK_TEMP_UNDER;
            bat_err_add(PACK_TEMP_UNDER, bat_node[node].under_temp, node);
        }
    }
}




void check_stack_fuse()
{
	uint8_t stack=0;

	int delta_0_1, delta_1_2, delta_2_0;

	// compute delta
	delta_0_1 = (int)bat_stack[0].voltage - (int)bat_stack[1].voltage;
	delta_1_2 = (int)bat_stack[1].voltage - (int)bat_stack[2].voltage;
	delta_2_0 = (int)bat_stack[2].voltage - (int)bat_stack[0].voltage;

	// absolute value of delta
	if(delta_0_1 < 0) delta_0_1 *= -1;
	if(delta_1_2 < 0) delta_1_2 *= -1;
	if(delta_2_0 < 0) delta_2_0 *= -1;

	// Comparisons to stack limits
	if((unsigned int)delta_0_1 > STACK_VOLT_DIFF_LIMIT)
		bat_stack[0].bad_counter++;
	else
		if(bat_stack[0].bad_counter > 0)
			bat_stack[0].bad_counter--;

	if((unsigned int)delta_1_2 > STACK_VOLT_DIFF_LIMIT)
		bat_stack[1].bad_counter++;
	else
		if(bat_stack[1].bad_counter > 0)
			bat_stack[1].bad_counter--;

	if((unsigned int)delta_2_0 > STACK_VOLT_DIFF_LIMIT)
		bat_stack[2].bad_counter++;
	else
		if(bat_stack[2].bad_counter > 0)
			bat_stack[2].bad_counter--;


	stack=0;
	for (stack =0;stack<3;stack++){
		if (bat_stack[stack].bad_counter>FUSE_BAD_LIMIT){
			bat_pack.status |= STACK_FUSE_BROKEN;
			fatal_err |= STACK_FUSE_BROKEN;

		if(bat_stack[0].bad_counter > FUSE_BAD_LIMIT &&
			bat_stack[1].bad_counter > FUSE_BAD_LIMIT)
		{
			bat_pack.fuse_fault = 1;
            bat_pack.voltage = (bat_pack.stacks[0]->voltage+bat_pack.stacks[2]->voltage)/2;
		} // if fuse on stack 1 fails and compensate the pack voltage

		if(bat_stack[1].bad_counter > FUSE_BAD_LIMIT &&
			bat_stack[2].bad_counter > FUSE_BAD_LIMIT)
		{
			bat_pack.fuse_fault = 2;
            bat_pack.voltage = (bat_pack.stacks[0]->voltage+bat_pack.stacks[1]->voltage)/2;
		} // if fuse on stack 2 fails and compensate the pack voltage

		if(bat_stack[2].bad_counter > FUSE_BAD_LIMIT &&
			bat_stack[0].bad_counter > FUSE_BAD_LIMIT)
		{
			bat_pack.fuse_fault = 0;
            bat_pack.voltage = (bat_pack.stacks[1]->voltage+bat_pack.stacks[2]->voltage)/2;
		} // if fuse on stack 0 fails and compensate the pack voltage

		}  
	}
}


void bat_err_add(uint16_t err, uint8_t bad_cell, uint8_t bad_node){
    bat_pack.health = FAULT;
    uint8_t i=0;
    // check array, dont duplicate
    if (bat_err_index_loop){
        for (i=0;i<100;i++){
            if (err == bat_err_array[i].err
             || bad_cell == bat_err_array[i].bad_cell
             || bad_node == bat_err_array[i].bad_node){
                return;
            }
        }
    }else{
        for (i=0;i<bat_err_index;i++){
            if (err == bat_err_array[i].err
             || bad_cell == bat_err_array[i].bad_cell
             || bad_node == bat_err_array[i].bad_node){
                return;
            }
        }
    }


    if (bat_err_index>=100){
        bat_err_index_loop = 1;
        bat_err_index = 0;
    }else{
        bat_err_index++;
    }

    // because when this function been called, it must be a serious problem that is err

    bat_err_array[bat_err_index].err = err;
    bat_err_array[bat_err_index].bad_cell = bad_cell;
    bat_err_array[bat_err_index].bad_node = bad_node;


    return;
}


uint8_t temp_transfer(uint16_t in_raw, uint16_t ref){
    //using 1/T = 1/T0 +(1/B)(R/R0)
    //V = raw/0xffff*5
    //R is R=10K(5-V)/V;
    //translate raw reading to C temperature
    //B25=3900
    //B75=3936
    if (in_raw==65535){
        //bad reading
        return 0;
    }
    uint16_t raw = in_raw;
    raw=raw+1;
    float v = 3.0*(1.0*raw/(ref*1.0));
    float R=(1.0*10000.0*v/5)/(1-v/5.0);
    float beta = 3950.0;
    float A = 0.01764;
    float T = beta/log(R/A)-273.16;
    return (uint8_t)ceil(T);
}
//void balance_cells(){}// balance_cells()

void voltage_compensation(){
    //should compsensation to top and bottom cells
    /*
    float dV = 500;         //in 0.0001V
    float temp = 0;
    float d=0;
    uint8_t stack=0;
    float dT=0;
    
    for (stack=0;stack<3;stack++){
        //calculate voltage across interface
        if (temp_transfer(bat_stack[stack].value16) > 25){
            dT = (float)(temp_transfer(bat_stack[stack].value16) - 25);
            temp = dT*0.017+1.4;
            d = (temp/1.4)*dV;
        }else{
            dT = (float)(25-temp_transfer(bat_stack[stack].value16));
            temp = 1.4-dT*0.017;
            d = (temp/1.4)*dV;
        }
        if (d>800){
            d=800;
        }else if(d<500){
            d=500;
        }
        mypack.cell[stack][0][0].value16+=(uint16_t)d;
        mypack.cell[stack][1][6].value16+=(uint16_t)d;
        mypack.cell[stack][2][0].value16+=(uint16_t)d;
        mypack.cell[stack][3][6].value16+=(uint16_t)d;
        
    }
    
    */
    
}




// The basic idea of SOC estimation is reading current value and integrate them by time.
// Time can be estimated from delta.Time
BAT_SOC_t get_soc()
{
    BAT_SOC_t tempSOC;
    tempSOC.absolute_SOC = bat_pack.current_charge;
    tempSOC.percent_SOC = 100*tempSOC.absolute_SOC/SOC_FULL_CAP;  //percentile SOC
    return tempSOC;
} // get_soc()

void update_soc(){
    // attention!! timer is counting down!
    if (bat_pack.time_stamp==0){
        bat_pack.time_stamp = SOC_Timer_ReadCounter();
        deltaTime = 0;
    }else{
        deltaTime = (SOC_Timer_ReadCounter()<bat_pack.time_stamp ? 
            bat_pack.time_stamp - SOC_Timer_ReadCounter() :
            bat_pack.time_stamp+(65535-SOC_Timer_ReadCounter()));
        bat_pack.time_stamp = SOC_Timer_ReadCounter();
    }
    
    // calibrate SOC when voltage reach linear region.
    // if its high SOC
    if ((bat_pack.SOC_cali_flag==0) && (bat_pack.voltage >= SOC_CALI_HIGH)){
        bat_pack.current_charge = SOC_SOC_HIGH;
        bat_pack.SOC_cali_flag = 1;
    }
    // if its low SOC
    if ((bat_pack.SOC_cali_flag==0) && (bat_pack.voltage <= SOC_CALI_LOW)){
        bat_pack.current_charge = SOC_SOC_LOW;
        bat_pack.SOC_cali_flag = 1;
    }

    // reset cali falg when charge droped into nomial voltage
    if (bat_pack.voltage < (SOC_CALI_HIGH-1000) || bat_pack.voltage > (SOC_CALI_LOW+1000)){
        bat_pack.SOC_cali_flag = 0;
    }

    // if it is full?
    if (bat_pack.voltage >= SOC_FULL){
        bat_pack.status |= FULL;
    }

    float deltaCharge = 1.0*deltaTime*bat_pack.current*100.0/1000.0;   //get_current returned mA
    // deltaTime = (getTime) - lastTime;
    if (deltaCharge<0){
        //charging
        bat_pack.current_charge = bat_pack.current_charge + (uint32_t)abs(floor(deltaCharge));
    }else{
        bat_pack.current_charge = bat_pack.current_charge - (uint32_t)abs(floor(deltaCharge));
    }
    
    bat_pack.SOC_percent = (uint8_t)floor(100.0*bat_pack.current_charge/SOC_SOC_HIGH);
    
    // write current charge back to EEPROM
    _SOC_log();
  
    
    return;
}


uint8_t bat_health_check(){
    if ((bat_pack.status | COM_FAILURE) ||
    (bat_pack.status | STACK_FUSE_BROKEN) ||
    (bat_pack.status | ISO_FAULT) ||
    (bat_pack.status | IMBALANCE)){
        bat_pack.health = FAULT;
        return 1;
    }else{
        return 0;
    }   
}

void _SOC_log(){
    uint32_t temp_SOC = bat_pack.current_charge;
    SOC_Store_WriteByte((uint8_t)(0xff&temp_SOC>>24), 0x00);
    SOC_Store_WriteByte((uint8_t)(0xff&temp_SOC>>16), 0x01);
    SOC_Store_WriteByte((uint8_t)(0xff&temp_SOC>>8), 0x02);
    SOC_Store_WriteByte((uint8_t)(0xff&temp_SOC), 0x03);
    return;
}


void bat_balance(){
    uint8_t ic=0;
    uint8_t cell=0;
    uint8_t i=0;
    uint8_t temp_cfg[TOTAL_IC][6];
    uint16_t low_voltage = bat_pack.LO_voltage <= UNDER_VOLTAGE ? UNDER_VOLTAGE :bat_pack.LO_voltage;
    
    for (ic=0;ic<TOTAL_IC;ic++){
        for (i=0;i<6;i++){
            temp_cfg[ic][i] = tx_cfg[ic][i];
        }
    }
    
    for (ic=0;ic<TOTAL_IC;ic++){
        for (cell=0;cell<12;cell++){
            uint16_t diff = 0;
            if (cell_codes[ic][cell]/10 > low_voltage)
                diff = cell_codes[ic][cell]/10 - low_voltage;
                
            //if ((CELL_ENABLE & (0x1<<cell)) && ((cell_codes[ic][cell]/10)-low_voltage > BALANCE_THRESHOLD)){
            if ( diff > 0 && diff > BALANCE_THRESHOLD ){    
                // if this cell is 30mV or more higher than the lowest cell
                if (cell<8){
                    temp_cfg[ic][4] |= (0x1<<cell);
                }else{
                    temp_cfg[ic][5] |= (0x1<<(cell-8));
                }
            }
        }
    }
    
    // discharge time has been set in void LTC6804_init_cfg() already. 0x2 = 1 min
    
    LTC6804_wrcfg(TOTAL_IC, temp_cfg);
    
    
}


void DEBUG_balancing_on(){
    uint8_t ic=0;
    uint8_t cell=0;
    uint8_t i=0;
    uint8_t temp_cfg[TOTAL_IC][6];
    uint16_t low_voltage = bat_pack.LO_voltage <= UNDER_VOLTAGE ? UNDER_VOLTAGE :bat_pack.LO_voltage;
    
    for (ic=0;ic<TOTAL_IC;ic++){
        for (i=0;i<6;i++){
            temp_cfg[ic][i] = tx_cfg[ic][i];
        }
    }
    
    for (ic=0;ic<TOTAL_IC;ic++){
        for (cell=0;cell<12;cell++){
            temp_cfg[ic][4] |= 0x00;
            temp_cfg[ic][5] |= 0x21;
        }
    }
    
    LTC6804_wrcfg(TOTAL_IC, temp_cfg);
    
}