/*
This file is the function implemented on the Freescale Sensor Fussion board for the mouse control of the MANO robotic arm project. 
The following function is to map users's head movement into a bluetooth mouse profile in order to control the robotic arm. It includes the calibration function, head movement deadzone , control selection menu, and bluetooh mouse control.
*/


#include "Events.h"
#include "mqx_tasks.h"
#include "I2C.h"
#include "UART.h"
#include "FTM.h"
#include "include_all.h"
#include "string.h"
#include "Cpu.h"
#include "Events.h"
#include "mqx_tasks.h"
#include "SW1.h"
#include "SW2.h"

#include <stdio.h>
#include "LED_RED.h"
#include "LED_GREEN.h"
#include "LED_BLUE.h"
#include "I2C.h"
#include "UART.h"
#include "FTM.h"
#include "include_all.h"


// I2C and UART global buffers
uint8 I2C_Buf[I2C_BUF_LEN];
uint8 sUARTOutputBuf[UART_OUTPUT_BUFFER_SIZE];
uint8 sUARTInputBuf[UART_INPUT_BUFFER_SIZE];
uint8 iCommand[4];
uint8 floatConversion[3];

// sensor physical I2C addresses
#define MPL3115_I2C_ADDR		0x60
#define FXOS8700_I2C_ADDR		0x1E
#define FXAS21000_I2C_ADDR		0x20
#define FXAS21002_I2C_ADDR		0x20
#define MMA8652_I2C_ADDR		0x1D
#define MAG3110_I2C_ADDR		0x0E

// MPL3115 registers and constants
#define MPL3115_STATUS					0x00
#define MPL3115_OUT_P_MSB				0x01
#define MPL3115_WHO_AM_I				0x0C
#define MPL3115_CTRL_REG1       		0x26
#define MPL3115_WHO_AM_I_VALUE			0xC4

// FXOS8700 registers and constants
#define FXOS8700_OUT_X_MSB       	  	0x01
#define FXOS8700_WHO_AM_I      			0x0D
#define FXOS8700_XYZ_DATA_CFG       	0x0E
#define FXOS8700_CTRL_REG1        	 	0x2A
#define FXOS8700_CTRL_REG2        	 	0x2B
#define FXOS8700_M_CTRL_REG1         	0x5B
#define FXOS8700_M_CTRL_REG2        	0x5C
#define FXOS8700_WHO_AM_I_VALUE     	0xC7

// FXAS21000 registers and constants
#define FXAS21000_DATA_REG            	0x01
#define FXAS21000_WHO_AM_I        		0x0C
#define FXAS21000_CTRL_REG0           	0x0D
#define FXAS21000_CTRL_REG1           	0x13
#define FXAS21000_WHO_AM_I_VALUE		0xD1

// FXAS21002 registers and constants
#define FXAS21002_DATA_REG            	0x01 
#define FXAS21002_WHO_AM_I        		0x0C 
#define FXAS21002_CTRL_REG0           	0x0D
#define FXAS21002_CTRL_REG1           	0x13
#define FXAS21002_WHO_AM_I_VALUE		0xD4

// MMA8652 registers and constants
#define MMA8652_STATUS					0x00
#define MMA8652_OUT_X_MSB       	  	0x01
#define MMA8652_WHO_AM_I        		0x0D
#define MMA8652_XYZ_DATA_CFG     	  	0x0E
#define MMA8652_CTRL_REG1           	0x2A
#define MMA8652_CTRL_REG2           	0x2B
#define MMA8652_WHO_AM_I_VALUE			0x4A

// MAG3110 registers and constants
#define MAG3110_STATUS					0x00
#define MAG3110_OUT_X_MSB       	  	0x01
#define MAG3110_WHO_AM_I      			0x07
#define MAG3110_CTRL_REG1        	 	0x10
#define MAG3110_CTRL_REG2         		0x11
#define MAG3110_WHO_AM_I_VALUE     		0xC4
#define DEADZONE						0x02
// Stage machine address
#define AnaDigReadStg	0
#define	CaliStg1	2
#define	CaliStg2	3
#define	CaliStg3	4
#define	CaliStg4	5
#define	CaliStg5	6
#define	ResetStg	7
#define TerStg	15
#define	DataSendStg	33
//bite-sip-n-puff data 
#define puff	32000
#define sip		28000
#define sipRange 30000
#define puffRange 15000

//mouse action data 
static uint_16 value[AD1_CHANNEL_COUNT];
uint_32 count1=0;
uint_32 count2=0;

uint_8 stage=0;
uint_8 reset_x;
uint_8 reset_y;
uint_8 ScaleX;
uint_8 ScaleY;
uint_8 stgCount=0;

bool ScrollUp=0;
bool ScrollDown=0;
uint_8 caliValue=2;



/***************************************************************************
** This section adds in mouse functionality to the sensor fusion above.
** Sip and puff and digital buttons were added.
***************************************************************************/
/*========================================================================
** Function: buzz(uint_8 delay)
** 
** Description:
** 				Feedback for the user when go into different modes.
=========================================================================*/

void buzz(uint_8 delay)
{
	
	int cycle=0;
		for (cycle;cycle<100;cycle++)
		{
			Bit1_SetVal();
			WAIT1_Waitus(delay);
				Bit1_ClrVal();
				WAIT1_Waitus(delay);
	}
}

/*========================================================================
** Function: (Neg)scaleInt
** 
** Description:
** 			convert 16 bits sensor fusion value to 8 bits Bluetooth value
** 			variable 'reset' is the value grabbed when on the reset mode
** 			it is define the ZERO point of the mouse
=========================================================================*/
uint8 scaleInt(int16 value,uint8 reset)  
{
	return applyDeadzone(value/20.0F,reset);
}
uint8 scaleNegateInt(int16 value, uint8 reset)  // conv1
{
	return applyDeadzone(-value/20.0F,reset);
}
 
// define mouse deadzone
/*========================================================================
** Function: applyDeadzone
** 
** Description:
** 		define Deadzone
** 		when mouse X/Y value in between positive and negative DEADZONE value, it is in deadzone
** 		Mouse will not move when in deadzone
** 		when mouse X/Y value is larger than positive deadzone value or less than negative deadzone value, 
** 		mouse will move, mouse data is getting substracted by the deadzone value
** 		deadzone value is defined by DEADZONE 
**  
=========================================================================*/
float applyDeadzone(float value, float zero){
	if ((value<(zero+DEADZONE)) && (value>(zero-DEADZONE))){
		return zero;
	} else if (value<zero) {
		return value + DEADZONE;
	} else {
		return value - DEADZONE;
	}
}

/*======================================================================================
 * This portion of code is to translate the sensor fusion, bite-sip-n-puff data to a 8-bits BT mouse profile
	
 * The task got finish by 9 stages
 * AnaDigReadStg: read the analog value( pressure sensor) and digital value( bite sensor )
 * DataSend Stage: combine, modify the data and send it though bluetooth 
 * 
 * Reset Stage: Grab the center value and reset the center
 * Calibration Stage (1-5) : adjust sensitivity of the mouse 					
 * TerStg: Send Tertiary click 
 *

=======================================================================================*/

void Bluetooth_UART(LDD_TDeviceData *DeviceDataPtr)
{
	struct fquaternion fq;		// quaternion to be transmitted
	float ftmp;
	uint32 iIndex;				// output buffer counter
	int32 tmpint32;				// scratch int32
	int16 tmpint16;				// scratch int16
	int16 iPhi, iThe;
	//int16 iRho;		// integer angles to be transmitted
	int16 iDelta;				// magnetic inclination angle if available
	int16 iOmega[3];			// scaled angular velocity vector
	uint16 isystick;			// algorithm systick time
	uint8 tmpuint8;				// scratch uint8
	uint8 flags;				// byte of flags
	int16 i, j, k;				// general purpose

#ifdef UART_OFF
	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK; // Enable full STOP mode
	return;  // SERIAL COMM IS NOT RUNNING
#else
								//    LED_RED_ClrVal(NULL);
	SCB_SCR &= (~SCB_SCR_SLEEPDEEP_MASK); // Disable full STOP mode
#endif

	if 	(globals.RPCPacketOn)  
	{
	switch (stage){			
		case AnaDigReadStg: 
			stgCount=DataSendStg;
			/***Analog and Digital Read Stage ***/
			AD_finished = FALSE;	//read the analog value( pressure sensor) and digital value( bite sensor )			
			AD1_Measure(TRUE); 
			AD1_GetValue16(&value[0]); 
			//RESET STAGE
					
			if((value[0]>puff)&&(SW2_GetVal(NULL)==1)){//puff and left click
				count1++;
				if (count1>20){ 			//if more than 2 secs
					stgCount=ResetStg;	//go to reset stage
					count1=0;
						
				}
			}
			//Calibration Stage
			else if((value[0]<sip)&&(SW2_GetVal(NULL)==1)){//sip and left click 
				count1++;
				if (count1>20){			//if more than 2 secs
					stgCount=caliValue;	//go to calibration stage 
					count1=0;
				}
			} 
			else if ((value[0]>puff)&&(SW1_GetVal(NULL)==1))	//if puff and right click 
			{
				count1++;
				if (count1>10){
				stgCount=TerStg;	//send a tetiary click 
				count1=0;
								}
			}
			else{
			  count1=0;
			}						
			stage=stgCount;	  
			break;
		
		
		case TerStg:
			TerDone=0;
			stage=0;
			stgCount=0;
			//analog read 
			AD_finished = FALSE;				
			AD1_Measure(TRUE);
			AD1_GetValue16(&value[0]); 
			//send command 
			globals.iPacketNumber++;
			sBufAppendItem(sUARTOutputBuf, &iIndex, &HID_MOUSE_START[0], 3);
			sUARTOutputBuf[iIndex++] = 0x04;
			sUARTOutputBuf[iIndex++] = 0x00;
			sUARTOutputBuf[iIndex++] = 0x00;
			sUARTOutputBuf[iIndex++] = 0x00;
			UART_SendBlock(DeviceDataPtr, sUARTOutputBuf, iIndex);
			//go back to analogread 
			stgCount=0;
			break;
		    /*==========================
		    ** RESET STAGE
		   	===========================*/
		case ResetStg:
				resetDone=0; //reset 
				stage=0;
				stgCount=0;
				AD_finished = FALSE;				
				AD1_Measure(TRUE); 
				AD1_GetValue16(&value[0]); 
												//LED change to Blue-purple 		 
				LED_BLUE_ClrVal(NULL);
				LED_GREEN_SetVal(NULL);
				LED_RED_ClrVal(NULL);
				
				buzz(2);				//buzzer on 
				if(value[0]<sip){	//if sip, select new center and go back to DataSend Stage 
					 resetDone=1;	    	
				}
				if(!resetDone){
				 //grab the position value 
					reset_x=scaleInt(iPhi,0);
					reset_y=scaleNegateInt(iThe,0);// new position position
					stgCount=ResetStg;//reset stage   	
				 }
				if(resetDone){
					 stgCount=0;
				}
				stage=stgCount;
				break;	
			    /*=====================================
			    ** CALIBRATION STAGE
			    ** 5 LEVELS
			    ** The lower the buzzer number the lower
			    ** the sensitivity.
			    =======================================*/  
		case 2:
				caliDone=0;
				stage=0;
				stgCount=0;
				AD_finished = FALSE;				
				AD1_Measure(TRUE); 
				AD1_GetValue16(&value[0]); 
										// set LED blue-purple 
				LED_BLUE_ClrVal(NULL);
				LED_GREEN_SetVal(NULL);
				LED_RED_ClrVal(NULL);			
				 if(value[0]>puff){	//puff to get out calibration 
					 caliDone=1;	    	
				 }
				 if(!caliDone){		
					buzz(400);	 
					if(SW2_GetVal(NULL)==1){//if left click 
						count1++;
						if (count1>8){	// go up to next level of sensitivity 
							 caliValue=caliValue+2;
							 count1=0;
						}		
					}		 	    		
				}
				stgCount=caliValue;			 
				if(caliDone){
					 stgCount=0;
				}
				stage=stgCount;
				break;
		case 4:
				caliDone=0;		//reset 
				stage=0;
				stgCount=0;AD_finished = FALSE;				
				AD1_Measure(TRUE);
				AD1_GetValue16(&value[0]); 
										 // set LED blue-purple 
				LED_BLUE_ClrVal(NULL);
				LED_GREEN_SetVal(NULL);
				LED_RED_ClrVal(NULL);						
				if(value[0]>puff){//puff to get out calibration 
					caliDone=1;	    	
				}
			   if(!caliDone){					
					buzz(425);	 
					if(SW2_GetVal(NULL)==1){//if left click  
						count1++;
						if (count1>8){	// go up to next level of sensitivity 
							caliValue=caliValue+2;
							count1=0;
						}						 		
					}		 	    		
				}
				stgCount=caliValue;						 
				if(caliDone){
					stgCount=0;
				}
				stage=stgCount;			
				break;
		case 6:
				caliDone=0;
				stage=0;
				
				stgCount=0;
				AD_finished = FALSE;				
				AD1_Measure(TRUE); 
				AD1_GetValue16(&value[0]); 
											// set LED blue-purple 		 
				LED_BLUE_ClrVal(NULL);
				LED_GREEN_SetVal(NULL);
				LED_RED_ClrVal(NULL);									
				if(value[0]>puff){	//puff to get out calibration 
					 caliDone=1;	    	
				 }
				 if(!caliDone){								
					buzz(450);	 
					if(SW2_GetVal(NULL)==1){//if left click  
						count1++;
						if (count1>8){	// go up to next level of sensitivity 
							caliValue=caliValue+2;
							count1=0;
						}				 		
					}								 	    		
				}
				stgCount=caliValue;									 
				if(caliDone){
					stgCount=0;
				}
				stage=stgCount;						
				break;		
		case 8:
				caliDone=0;		//reset
				stage=0;
				stgCount=0;
				AD_finished = FALSE;				
				AD1_Measure(TRUE); 
				AD1_GetValue16(&value[0]); 
										// set LED blue-purple 	
				LED_BLUE_ClrVal(NULL);
				LED_GREEN_SetVal(NULL);
				LED_RED_ClrVal(NULL);										
				if(value[0]>puff){//puff to get out  calibration stage 
					caliDone=1;	    	
				}
				if(!caliDone){
														
					buzz(475);	 
					if(SW2_GetVal(NULL)==1){//if left click  								 	 
						count1++;
						if (count1>8){
							caliValue=caliValue+2; // go up to next level of sensitivity 
							count1=0;
						}									 		
					}										 	    		
				}
				stgCount=caliValue;
				if(caliDone){
						stgCount=0;
					}
					stage=stgCount;
					break;	
		case 10:
				caliDone=0;		//reset 
				stage=0;
				stgCount=0;
				AD_finished = FALSE;				
				AD1_Measure(TRUE); 
				AD1_GetValue16(&value[0]); 
										// set LED blue-purple			 
				LED_BLUE_ClrVal(NULL);
				LED_GREEN_SetVal(NULL);
				LED_RED_ClrVal(NULL);
				if(value[0]>puff){//puff to get out  calibration stage 
					caliDone=1;	    	
				}
				if(!caliDone){
					buzz(500);	 
					if(SW2_GetVal(NULL)==1){//if left click 
						count1++;
						if (count1>8){
							caliValue=2;	// go up the first level of  sensitivity 
							count1=0;
						}										 		
					}									 	    		
				}
				stgCount=caliValue;										 
				if(caliDone){
					stgCount=0;	
				}
				stage=stgCount;							
				break;		
		/*=============================================
		    ** DATA SEND STAGE
		    =============================================*/
		case DataSendStg:
				//  sending BT data command	

				globals.iPacketNumber++;// start byte: 0xFD 0x05 0x02
				sBufAppendItem(sUARTOutputBuf, &iIndex, &HID_MOUSE_START[0], 3);
				/**** button byte    ****/	    
				
				
					 if((SW1_GetVal(NULL)==1)&&(value[0]>sip)&&(value[0]<puff)){		
					   //turn on Blue LED
					   LED_BLUE_ClrVal(NULL);
					   LED_GREEN_SetVal(NULL);
					   LED_RED_SetVal(NULL);
					    sUARTOutputBuf[iIndex++] = 0x02; // right click 
					    		    	
				} 
					
					 else  if((SW2_GetVal(NULL)==1)&&(value[0]>sip)&&(value[0]<puff)){		
					  //turn on Blue LED
					  LED_BLUE_ClrVal(NULL);
					  LED_GREEN_SetVal(NULL);
					  LED_RED_SetVal(NULL);

					  sUARTOutputBuf[iIndex++] = 0x01; // left click 
				}
			  
				else{
					   LED_BLUE_SetVal(NULL);
					   sUARTOutputBuf[iIndex++] = 0x00; //no click event 
				}	
		   	   
			/*** Moving byte ( X and Y)***/
					 /*
					  * noted that value from 0x01 to 0x7E is positive
					  * value from 0x81 to 0xFF is negative
					  * 0x00 ,0x80, 0x7F are zero
					  */
			sUARTOutputBuf[iIndex++] =  (caliValue	*(scaleInt(iPhi,reset_x)-reset_x));	  
			sUARTOutputBuf[iIndex++] =  (caliValue*(scaleNegateInt(iThe,reset_y)-reset_y));   		   	    
			/***** Scrolling Byte  ****/		   	    
			
			/*======================================================================================
			** Method: SCROLL
			** Description:
			** 			Sip to scroll up
			** 			Puff to scroll down 
			** 			The scrolling speed is proportional to the value of pressure sensor 
			** 			Once pressure sensor value passes a certain value ( max or min scroll speed),
			** 			srcoll up/down stays at max/min speed 
			======================================================================================*/
			
				if((value[0]<sip)&&(value[0]>(sip-sipRange))&&(value[0]>5)){	//scroll down 			
					sUARTOutputBuf[iIndex++] = (0xFF- (sip-value[0])/500); //reduced range 
				} 
				else if (value[0]<(sip-sipRange)||(value[0]<5)||(value[0]==(sip-sipRange)))
				{
					sUARTOutputBuf[iIndex++] = 0x7E;
				}

				else if ((value[0]>puff)&&(value[0]<(puff+puffRange))){	//scroll up
					
					//sUARTOutputBuf[iIndex++] = (0xFF-((value[0]-puff)/115));//full range
					sUARTOutputBuf[iIndex++] = (((value[0]-puff)/250));//reduced range 
					
				}	
				else if ((value[0]==(puff+puffRange))||(value[0]>(puff+puffRange)))
				{
					sUARTOutputBuf[iIndex++] = 0x88;
				}

				else{
					sUARTOutputBuf[iIndex++] = 0x00;   //disable scroll 
				}			
			//send BT data 
			UART_SendBlock(DeviceDataPtr, sUARTOutputBuf, iIndex);
			stage=0;
			break;
		}	
	}

	return;
}