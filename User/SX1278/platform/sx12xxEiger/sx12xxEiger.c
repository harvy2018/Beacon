/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx12xxEiger.c
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 */
#include <stdint.h> 
#include "spi.h"
#include "sx12xxEiger.h"
#include "uartPort.h"
#include "bsp_spi_flash.h"
#include "bsp_usart.h"
#include "bsp_timer.h"
#include "sx1276-LoRa.h"
#include "OLED_I2C.h"
#include "device.h"
#include "flashMgr.h"
#include "info_source.h"
#include "GPS.h"

extern u8 Battery0[];
extern u8 Battery1[];
extern u8 Battery2[];
extern u8 Battery3[];
extern u8 Battery4[];
extern u8 BatteryC[];


u8 BT_SLEEP[]="AT+SLEEP\r\n";
u8 BT_WAKE[]="AT+WAKE\r\n";
u8 BT_AT[]="AT\r\n";

u8 BT_STATUS=0;//0:WAKE;1:SLEEP

u8 button_state[3]={0};

// System tick (1ms)
volatile uint32_t TickCounter = 0;

void BoardInit( void )
{
		
    	bsp_InitTimer();
        GPIO_Configuration(); 
		USART1_Config();
		USART2_Config();
		//USART3_Config();

		SpiInit();//lora spi	
	   // sf_InitHard();	/* 初始化SPI flash */
      //  I2C_Configuration();
	  //  OLED_Init();   
        ADC_Configuration();
	  //  TIMX_Init(CALC_TYPE_US);  
        NVIC_Configuration();    
}


void Delay (uint32_t delay)
{
    // Wait delay ms
    uint32_t startTick = TickCounter;
    while( ( TickCounter - startTick ) < delay ); 
    
}

void LongDelay (uint8_t delay)
{
    uint32_t longDelay;
    uint32_t startTick;

    longDelay = delay * 1000;

    // Wait delay s
    startTick = TickCounter;
    while( ( TickCounter - startTick ) < longDelay );   
}


void GPIO_Configuration(void)
{ 
    
		GPIO_InitTypeDef GPIO_InitStructure;
		 //禁用JTAG 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 			//激活GPIOB clock
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 			//激活GPIOB clock
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 			//激活GPIOB clock
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
        GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	

		GPIO_InitStructure.GPIO_Pin = BUZZ_PIN|LED_G_PIN|LED_B_PIN|LED_R_PIN;  	//设置GPIOA11、设置GPIOA12
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				  	//设置GPIO速度为10MHZ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;				  	//设置以上三个GPIO为输出
		GPIO_Init(GPIOB, &GPIO_InitStructure);							  	//将以上设置参数写入
    
        GPIO_InitStructure.GPIO_Pin = BT_PWR_PIN|GPS_PWR_PIN;  	       //设置GPIOA11、设置GPIOA12
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			   //设置GPIO速度为10MHZ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   //设置以上三个GPIO为输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
    

	    GPIO_InitStructure.GPIO_Pin = CHRG_PIN|BUTTON_DOWN_PIN;  	//设置设置GPIOA15、设置GPIOA8
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				  	//设置GPIO速度为10MHZ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;				  	//设置以上三个GPIO为输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);		

	
		GPIO_InitStructure.GPIO_Pin = BUTTON_UP_PIN|BUTTON_MID_PIN;  	//设置GPIOB11、GPIOB10
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				  	//设置GPIO速度为10MHZ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;				  	//设置以上三个GPIO为输出
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

		LED_ON;
        bsp_DelayMS(200);
        
        LED_OFF;
        
        
		BUZZ_OFF;
     //   GPS_PWR_OFF;
        GPS_PWR_ON;
        BT_PWR_OFF;

}

void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
	
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  // Tax uart port 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  // Tax uart port 1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		

 
}

u8 CHRG_Detect(void)
{
    u8 ret;   
    ret = GPIO_ReadInputDataBit(CHRG_PORT, CHRG_PIN);     
}



void Button_Detect(void)
{
        
    u8 button[3]={0};

 
    for(u8 i=0;i<7;i++)
    {   
         button[0]+= GPIO_ReadInputDataBit( BUTTON_UP_PORT,   BUTTON_UP_PIN );     
         button[1]+= GPIO_ReadInputDataBit( BUTTON_MID_PORT,  BUTTON_MID_PIN );
         button[2]+= GPIO_ReadInputDataBit( BUTTON_DOWN_PORT, BUTTON_DOWN_PIN );
       //  bsp_DelayMS(10);
    }
    
    for(u8 i=0;i<3;i++)
    {
        if(button[i]<=2)
           button_state[i]=1; 
        else
           button_state[i]=0;  
    }
    
}
    
/*    
void Button_Menu(u8 *ButtonState)  
{
    extern u32 SecTick;
    extern int temperaure;  
    extern u16 seq_index,RX_cnt;    
    extern u16 Voltage,HistoryRXCnt,FlashErrCnt;
    
    u8 dlen;
    u8 buf[128]={0};
    static u8 sysFlag=1;
    u8 PageNum=4;
    static u8 ButtonStatePre[3]={0};
       
    
    if(ButtonState[0]==1)  
     {
          
        OLED_CLS_B();
        sprintf(buf,"SysInfo:%d-%d",PageNum,sysFlag);
        OLED_ShowStr(0,0,buf,2); 
        memset(buf,0,sizeof(buf));
         
        if(sysFlag==1)
        {
            sysFlag++;
           

            if(BT_STATUS==0)
               OLED_ShowStr(0,4,"BT Status: ON",2); 
            else
               OLED_ShowStr(0,4,"BT Status: OFF",2);  
            
              
              if(SX1276LoRaGetRFState()==RFLR_STATE_IDLE)
              {                    
                  OLED_ShowStr(0,6,"RF Status: OFF",2);  
              }
              else
              {                    
                 OLED_ShowStr(0,6,"RF Status: ON",2);  
              }
                          
        }
        else if(sysFlag==2)
        {
            sprintf(buf,"Temp:%0.2f'",(float)temperaure/10000);
            OLED_ShowStr(0,4,buf,2); 
            sprintf(buf,"Voltage:%dmV",Voltage);             
            OLED_ShowStr(0,6,buf,2);
            sysFlag++;
        } 
        else if(sysFlag==3)
        {
            sprintf(buf,"H_RXCnt:%d",HistoryRXCnt);
            OLED_ShowStr(0,4,buf,2); 
            sprintf(buf,"C_RXCnt:%d",RX_cnt);             
            OLED_ShowStr(0,6,buf,2);
            sysFlag++;
        }
        else
        {
            sprintf(buf,"FlashErrCnt:%d",FlashErrCnt);
            OLED_ShowStr(0,4,buf,2);
            sprintf(buf,"RunTime:%dmin",SecTick/60); 
            OLED_ShowStr(0,6,buf,2); 
            sysFlag=1;            
        }
        
     }
        
        if(ButtonState[1]==1 && ButtonState[2]==0)
        {
            
            OLED_CLS_B();//清屏 
                                
          if(SX1276LoRaGetRFState()==RFLR_STATE_IDLE)
          {    
             SX1276LoRaSetRFState(RFLR_STATE_RX_INIT);
             OLED_ShowStr(0,0,"RF ON!",2);  
          }
          else
          {
             SX1276LoRaSetRFState(RFLR_STATE_IDLE);
             OLED_ShowStr(0,0,"RF OFF!",2);  
          }
 
            
        }
        
        if(ButtonState[2]==1 && ButtonState[1]==0)
        {             
            OLED_CLS_B();
            Wireless_Info_Review(seq_index);
            seq_index--;
            if(seq_index==0)
                seq_index=RX_cnt;
           
        }
        
         if(ButtonState[2]==1 && ButtonState[1]==1)
         {
               OLED_CLS_B();//清屏 BT_GETSTATUE
                        
              if(BT_STATUS==0) 
              {
                   
                  
                   uartWriteBuffer(UART_BT,BT_SLEEP,sizeof(BT_SLEEP));
                   uartSendBufferOut(UART_BT);
                  
                 // bsp_DelayMS(500);
                  
//                  dlen=uartGetAvailBufferedDataNum(UART_BT);
//                  if(dlen>=2)  
//                  {                      
//                    uartRead(UART_BT,buf,dlen);
//                    if(buf[dlen-1]=='O' && buf[dlen]=='K')
//                    {
                          OLED_ShowStr(0,0,"BT OFF!",2); 
                           BT_STATUS=1;
                   // }                   
                        
                 // }
                  
              }
              else
              {
                  
                  for(u8 t=0;t<50;t++) 
                  {
                      uartWriteBuffer(UART_BT,BT_WAKE,sizeof(BT_WAKE));
                      uartSendBufferOut(UART_BT);
                  }
                   bsp_DelayMS(500);
               
                  dlen=uartGetAvailBufferedDataNum(UART_BT);
                  if(dlen>=2)  
                  {                      
                    uartRead(UART_BT,buf,dlen);
                    if(buf[dlen-5]=='O' && buf[dlen-4]=='K')
                    {
                          OLED_ShowStr(0,0,"BT ON!",2); 
                          BT_STATUS=0;
                    }
                        
                  }
                  
                  
              }
         }
     
    
}

*/
void Button_Process(u8 *ButtonState)  
{
    extern u32 SecTick;
    extern int temperaure;  
    extern u16 seq_index,RX_cnt;    
    extern u16 Voltage,HistoryRXCnt,FlashErrCnt;
    extern u8 SOS_Flag;   
    u8 dlen;
    u8 buf[128]={0};
    static u8 GPSFlag=0;
    u8 PageNum=4;
  //  static u8 ButtonStatePre[3]={0};
    u8 RFStatus=0;
    extern tRadioDriver *Radio; 

    RFStatus=SX1276LoRaGetRFState();
    
    if(ButtonState[OK]==1 && ButtonState[BACK]==0 && ButtonState[NEXT]==0)
    {
            
           SOS_Flag=!SOS_Flag;
            BUZZ_ON;   
            bsp_DelayMS(200);
            BUZZ_OFF;  
       
    }
    else if((ButtonState[BACK]==1 || ButtonState[NEXT]==1) && (ButtonState[OK]==0) && (RFStatus!=RFLR_STATE_TX_RUNNING))
    {
        CheckIn idpack; 
        UploadIDPack(&idpack);
        Radio->SetTxPacket((u8*)&idpack, sizeof(idpack));
         LED_B_ON;
              
    }
    else if(ButtonState[BACK]==1 && ButtonState[NEXT]==1 && ButtonState[OK]==1)
    {
         if(GPSFlag==0)
         {
           GPS_PWR_OFF;
           GPSFlag=1;             
         }
         else
         {
             GPS_PWR_ON;
             GPSFlag=0;    
         }
    }         
    
}
/*
void Button_Process(u8 *ButtonState)  
{
    extern u32 SecTick;
    extern int temperaure;  
    extern u16 seq_index,RX_cnt;    
    extern u16 Voltage,HistoryRXCnt,FlashErrCnt;
    
    FlashSectionStruct *pFlash;
    u8 dlen;
    u8 buf[128]={0};
    static u8 sysFlag=1;
    u8 PageNum=4;
    static u8 ButtonStatePre[3]={0};
     
    pFlash=&flashSectionStruct[FLASH_SECTION_DATA_INFO];
       
     
    if(ButtonState[OK]==1 && ButtonState[BACK]==0 && ButtonState[NEXT]==0)
    {
        
        OLED_CLS_B();//清屏 
                            
      if(SX1276LoRaGetRFState()==RFLR_STATE_IDLE)
      {    
          RFSwitchON();
      }
      else
      {
          RFSwitchOFF();
      }

        
    }
    else if((ButtonState[BACK]==1 || ButtonState[NEXT]==1) && ButtonState[OK]==0)
    {
        OLED_CLS_B();//清屏 
        Record_Review_B(ButtonState);
    }
    else if(ButtonState[BACK]==1 && ButtonState[NEXT]==1 && ButtonState[OK]==1)
    {
        OLED_CLS_B();//清屏 
        OLED_ShowStr(0,2,"Data Erase...",2);
        uf_EraseChip(FLASH_EX);
        uf_WriteBuffer(FLASH_EX,FlashVer,pFlash->base,sizeof(FlashVer));
        OLED_ShowStr(0,2,"Data Erase Done!",2);
        bsp_DelayMS(1000);
        NVIC_SystemReset();
    }
   
}
*/
void Battery_Led_Mgr(void)
{
     u16 Voltage;
    
     extern u8 LowPwr_Flag;
    
    Voltage=getVoltage();
  
            
    if(Voltage>=Battery_WARN && Voltage<Battery_0BAR)
    {
        LowPwr_Flag=1;
    }
    else if(Voltage<Battery_WARN)
    {       
        LED_OFF;
        SX1276LoRaSetRFState(RFLR_STATE_IDLE);
        SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP);
    }
    else
        LowPwr_Flag=0;
          
  //  printf("Voltage:%d\r\n",Voltage);
}


void Battery_Mgr(void)
{
     s16 usValue,Voltage;
     static s16 Voltage_temp=0;
     extern u8 CHRG_Flag;
     extern u8 ScreenTest_Flag;
    
     usValue = GetADC();
     Voltage=((uint32_t)usValue * 3300)*14/4095/11;
    
    if((Voltage-Voltage_temp>=50) || (Voltage_temp-Voltage-Voltage_temp>=50) || CHRG_Flag==1 ||  ScreenTest_Flag==0)
    {    
        CHRG_Flag=0;
        
        if(Voltage>=Battery_4BAR)
            OLED_DrawBMP(98,0,127,2,Battery4);
        else if(Voltage>=Battery_3BAR)
            OLED_DrawBMP(98,0,127,2,Battery3);
        else if(Voltage>=Battery_2BAR)
            OLED_DrawBMP(98,0,127,2,Battery2);
        else if(Voltage>=Battery_1BAR)
            OLED_DrawBMP(98,0,127,2,Battery1);
        else if(Voltage>=Battery_0BAR)
            OLED_DrawBMP(98,0,127,2,Battery0);
        else if(Voltage>=Battery_WARN)
        {
            OLED_DrawBMP(98,0,127,2,Battery0);
            OLED_ShowStr(24,4,"Low Power,Please Charge!",2);
        }
        else
        {
            OLED_OFF();
            LED_OFF;
            SX1276LoRaSetRFState(RFLR_STATE_IDLE);
            SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        }
        
        Voltage_temp=Voltage;
    }
    
}


BeaconCoordinate *beacon_info[BeaconRelayMax];   
Watch_15 *wptr15_info[BeaconRelayMax];
Watch_WangAn *wptr_info[BeaconRelayMax];   


void RelayMgrInit(void)
{
    extern RelayMgr RMGR[];
    for(u8 i=0;i<3;i++)
      memset((u8*)&RMGR[i],0,sizeof(RelayMgr));
            
//     memcpy((Watch_15 *)RMGR[WATCH_15].relay_ptr,       wptr15, BeaconRelayMax);
//     memcpy((Watch_WangAn *)RMGR[WATCH_WA].relay_ptr,   wptr,   BeaconRelayMax);
//     memcpy((BeaconCoordinate *)RMGR[BEACON].relay_ptr, bptr,   BeaconRelayMax);
      
}

void BeaconRelayStorage(void* ptr,Watch_Type Btpye)
{
   extern RelayMgr RMGR[];  
   extern  int8_t RxPacketSnrEstimate;
   extern float RxPacketRssiValue;
    

   switch (Btpye)
   {
       case WATCH_15:
       {
            Watch_15 *wptr15=(Watch_15 *)ptr;
           
            for(u8 i=0;i<RMGR[Btpye].BeaconRelayCnt;i++)
            {
               if(!memcmp((u8*)&(wptr15->Mask),(u8*)&(wptr15_info[i]->Mask),9))//检索到已存在的信标ID
               {
                 //   if(memcmp((u8*)wptr15_info[i],(u8*)wptr15,sizeof(Watch_15)))
                    {                                              
                        memcpy((u8*)wptr15_info[i],(u8*)wptr15,sizeof(Watch_15));                               
                       
                        wptr15_info[i]->Header=0x66BB;                               
                                                               
                        RMGR[Btpye].GetMatchBeacon=1;               
                        RMGR[Btpye].RealySendMgr[i]=1;
                    }
               }
               
            }                                  

            if(RMGR[Btpye].GetMatchBeacon==0)//未匹配到已有信标ID，说明是新的信标ID
            {                                   
               if(RMGR[Btpye].BeaconRelayCnt<BeaconRelayMax)
               {
                   wptr15_info[RMGR[Btpye].BeaconRelayCnt]=(Watch_15 *)malloc(sizeof(Watch_15));   
                   
                   memcpy((u8*)wptr15_info[RMGR[Btpye].BeaconRelayCnt],(u8*)wptr15,sizeof(Watch_15)); 
                   
                   wptr15_info[RMGR[Btpye].BeaconRelayCnt]->Header=0x66BB;                               
                   
                   RMGR[Btpye].RealySendMgr[RMGR[Btpye].BeaconRelayCnt]=1;                                  
                   RMGR[Btpye].BeaconRelayCnt++;
                   
               }
               
            }
             printf("Watch_15: E=%f,N=%f ",(float)(__REV(wptr15->longitude))/10000000,(float)(__REV(wptr15->latitude))/10000000);                      
             printf("SNR:%ddB, RSSI:%0.2fdBm\r\n",RxPacketSnrEstimate,RxPacketRssiValue);
                                                     
            RMGR[Btpye].GetMatchBeacon=0;  
           
       }      
       break;
       
       case WATCH_WA:
       {
            Watch_WangAn * wptr=(Watch_WangAn *)ptr;
           
            for(u8 i=0;i<RMGR[Btpye].BeaconRelayCnt;i++)
            {
               if(!memcmp((u8*)&(wptr->ID),(u8*)&(wptr_info[i]->ID),4))//检索到已存在的信标ID
               {
                  //  if(memcmp((u8*)wptr_info[i],(u8*)wptr,sizeof(Watch_WangAn)))
                    {                                              
                        memcpy((u8*)wptr_info[i],(u8*)wptr,sizeof(Watch_WangAn));                               
                       
                        wptr_info[i]->Header=0xBB;                               
                        wptr_info[i]->Tail=0xBB;
                                          
                        RMGR[Btpye].GetMatchBeacon=1;               
                        RMGR[Btpye].RealySendMgr[i]=1;
                    }
               }
               
            }                                  

            if(RMGR[Btpye].GetMatchBeacon==0)//未匹配到已有信标ID，说明是新的信标ID
            {                                   
               if(RMGR[Btpye].BeaconRelayCnt<BeaconRelayMax)
               {
                   wptr_info[RMGR[Btpye].BeaconRelayCnt]=(Watch_WangAn *)malloc(sizeof(Watch_WangAn));  
                   
                   memcpy((u8*)wptr_info[RMGR[Btpye].BeaconRelayCnt],(u8*)wptr,sizeof(Watch_WangAn)); 
                   
                   wptr_info[RMGR[Btpye].BeaconRelayCnt]->Header=0xBB;                          
                   
                   wptr_info[RMGR[Btpye].BeaconRelayCnt]->Tail=0xBB;  

                   RMGR[Btpye].RealySendMgr[RMGR[Btpye].BeaconRelayCnt]=1;                   
                   RMGR[Btpye].BeaconRelayCnt++;
                   
               }
               
            }
            printf("Watch_WA ID:%X,E=%f,N=%f ",wptr->ID,(float)(wptr->latitude)/100000,(float)(wptr->longitude)/100000);
            printf("SNR:%ddB, RSSI:%0.2fdBm\r\n",RxPacketSnrEstimate,RxPacketRssiValue);
                                                     
            RMGR[Btpye].GetMatchBeacon=0;  
       
       }
       break;
       
       case BEACON:
       {
            BeaconCoordinate *bptr=(BeaconCoordinate *)ptr;
            
           for(u8 i=0;i<RMGR[Btpye].BeaconRelayCnt;i++)
            {
               if(!memcmp((u8*)(bptr->id),(u8*)(beacon_info[i]->id),12))//检索到已存在的信标ID
               {
                   // if(memcmp((u8*)beacon_info[i],(u8*)bptr,sizeof(BeaconCoordinate)))
                    {                                              
                        memcpy((u8*)beacon_info[i],(u8*)bptr,sizeof(BeaconCoordinate));                               
                        beacon_info[i]->head.Preamble=0xFE;                               
                        beacon_info[i]->tail=0xF1;
                        beacon_info[i]->cc=CalcSum((u8*)&(beacon_info[i]->head.len),beacon_info[i]->head.len-1);
                        RMGR[Btpye].GetMatchBeacon=1;               
                        RMGR[Btpye].RealySendMgr[i]=1;
                    }
               }
               
            }                                  

            if(RMGR[Btpye].GetMatchBeacon==0)//未匹配到已有信标ID，说明是新的信标ID
            {                                   
               if(RMGR[Btpye].BeaconRelayCnt<BeaconRelayMax)
               {
                   beacon_info[RMGR[Btpye].BeaconRelayCnt]=(BeaconCoordinate *)malloc(sizeof(BeaconCoordinate));
                   
                   memcpy((u8*)beacon_info[RMGR[Btpye].BeaconRelayCnt],(u8*)bptr,sizeof(BeaconCoordinate));   
                   
                   beacon_info[RMGR[Btpye].BeaconRelayCnt]->head.Preamble=0xFE;    
                   
                   beacon_info[RMGR[Btpye].BeaconRelayCnt]->tail=0xF1;

                   beacon_info[RMGR[Btpye].BeaconRelayCnt]->cc=CalcSum((u8*)&(beacon_info[RMGR[Btpye].BeaconRelayCnt]->head.len),beacon_info[RMGR[Btpye].BeaconRelayCnt]->head.len-1);                                          
                   
                   RMGR[Btpye].RealySendMgr[RMGR[Btpye].BeaconRelayCnt]=1;
                   
                   RMGR[Btpye].BeaconRelayCnt++;
                   
               }
               
            }

            printf("ID: %08X %08X\r\n",(bptr->id[0]),(bptr->id[1]));
            printf("E: %f ",(float)((bptr->longitude))/10000000);
            printf("N: %f ",(float)((bptr->latitude))/10000000);
            printf("SNR:%ddB, RSSI:%0.2fdBm\r\n",RxPacketSnrEstimate,RxPacketRssiValue);
            printf("Voltage:%dmV\r\n",bptr->voltage);                                     
            RMGR[BEACON].GetMatchBeacon=0;  
       
       }
       break;
   }
       
    
}

void  RelaySend(Watch_Type Btpye,u8 RelayCnt)  
{
   extern tRadioDriver *Radio;  
   extern RelayMgr RMGR[]; 
   
 //  LED_B_Blink;  
    
   switch (Btpye)
   {
      case WATCH_15:
      {
       
            Radio->SetTxPacket((u8*)wptr15_info[RelayCnt], sizeof(Watch_15));              
            RMGR[Btpye].RelaySendCnt=RelayCnt;
           
  
      }
      break;    
      
      case WATCH_WA:
      { 
            Radio->SetTxPacket((u8*)wptr_info[RelayCnt], sizeof(Watch_WangAn));              
            RMGR[Btpye].RelaySendCnt=RelayCnt;
          
          
      }
      break; 
      
      case BEACON:
      {
            Radio->SetTxPacket((u8*)beacon_info[RelayCnt], sizeof(BeaconCoordinate));              
            RMGR[Btpye].RelaySendCnt=RelayCnt;
           
      }
      break;    
       
   }       
  //  LED_B_ON;   
  
      
}
    
    

    
                           
                           
                           
