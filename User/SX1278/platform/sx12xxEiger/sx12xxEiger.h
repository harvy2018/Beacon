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
 * \file       sx12xxEiger.h
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 */
#ifndef __SX12XXEIGER_H__
#define __SX12XXEIGER_H__



#include <stdint.h>
#include <stdbool.h>

#if defined( STM32F4XX )
    #include "stm32f4xx.h"
#elif defined( STM32F2XX )
    #include "stm32f2xx.h"
#else
    #include "stm32f10x.h"
#endif

//#include "../../../info_source.h"


#define USE_USB                                     1

#if defined( STM32F4XX ) || defined( STM32F2XX )
#define BACKUP_REG_BOOTLOADER                       RTC_BKP_DR0      /* Booloader enter*/
#else
#define BACKUP_REG_BOOTLOADER                       BKP_DR1          /* Booloader enter*/
#endif

#define FW_VERSION                                  "2.0.B2"
#define SK_NAME                                     "SX12xxEiger"

#define Battery_4BAR    3820
#define Battery_3BAR    3700
#define Battery_2BAR    3500
#define Battery_1BAR    3400
#define Battery_0BAR    3350
#define Battery_WARN    3310

#define MAX_STORAGE_NUM  32
#define LOCAL_TIME_ZONE_SECOND_OFFSET     28800 //8小时的秒数
#define REFERENCE_YEAR_BASE     1900


#define CHRG_PORT  GPIOA

#define BUTTON_UP_PORT    GPIOB
#define BUTTON_MID_PORT   GPIOB
#define BUTTON_DOWN_PORT  GPIOA

#define LED_G_PIN   GPIO_Pin_13
#define LED_R_PIN   GPIO_Pin_14
#define LED_B_PIN   GPIO_Pin_15

#define BUZZ_PIN        GPIO_Pin_12
#define CHRG_PIN        GPIO_Pin_15
#define BT_PWR_PIN      GPIO_Pin_11
#define GPS_PWR_PIN     GPIO_Pin_12

#define BUTTON_UP_PIN   GPIO_Pin_7
#define BUTTON_MID_PIN  GPIO_Pin_6
#define BUTTON_DOWN_PIN GPIO_Pin_8


#define BUZZ_ON     GPIO_SetBits(GPIOB,GPIO_Pin_12);
#define BUZZ_OFF    GPIO_ResetBits(GPIOB,GPIO_Pin_12);

#define LED_G_ON     GPIO_SetBits(GPIOB,GPIO_Pin_13);
#define LED_G_OFF    GPIO_ResetBits(GPIOB,GPIO_Pin_13);

#define LED_R_ON     GPIO_SetBits(GPIOB,GPIO_Pin_14);
#define LED_R_OFF    GPIO_ResetBits(GPIOB,GPIO_Pin_14);

#define LED_B_ON     GPIO_SetBits(GPIOB,GPIO_Pin_15);
#define LED_B_OFF    GPIO_ResetBits(GPIOB,GPIO_Pin_15);

#define GPS_PWR_ON     GPIO_SetBits(GPIOA,GPIO_Pin_12);
#define GPS_PWR_OFF    GPIO_ResetBits(GPIOA,GPIO_Pin_12);

#define BT_PWR_ON     GPIO_SetBits(GPIOA,GPIO_Pin_11);
#define BT_PWR_OFF    GPIO_ResetBits(GPIOA,GPIO_Pin_11);

//#define LED_Blink   GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12)));
//#define LED_Blink   GPIO_PinReverse(GPIOA, GPIO_Pin_12);

#define LED_G_Blink   {LED_G_ON;bsp_DelayMS(50);LED_G_OFF;}
#define LED_R_Blink   {LED_R_ON;bsp_DelayMS(100);LED_R_OFF;}
#define LED_B_Blink   {LED_B_ON;bsp_DelayMS(50);LED_B_OFF;}
#define LED_ON        {LED_G_ON;LED_R_ON;LED_B_ON;}
#define LED_OFF       {LED_G_OFF;LED_R_OFF;LED_B_OFF;}
#define BEEP          {BUZZ_ON;bsp_DelayMS(100);BUZZ_OFF;}

/*!
 * Functions return codes definition
 */
typedef enum
{
    SX_OK,
    SX_ERROR,
    SX_BUSY,
    SX_EMPTY,
    SX_DONE,
    SX_TIMEOUT,
    SX_UNSUPPORTED,
    SX_WAIT,
    SX_CLOSE,
    SX_YES,
    SX_NO,          
}tReturnCodes;

extern volatile uint32_t TickCounter;

/**
  * @brief   Small printf for GCC/RAISONANCE
  */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#endif /* __GNUC__ */

/*!
 * Initializes board peripherals
 */
void BoardInit( void );


/*!
 * Delay code execution for "delay" ms
 */
void Delay ( uint32_t delay );

/*!
 * Delay code execution for "delay" s
 */
void LongDelay ( uint8_t delay );

/*!
 * \brief Computes a random number between min and max
 *
 * \param [IN] min range minimum value
 * \param [IN] max range maximum value
 * \retval random random value in range min..max
 */
uint32_t randr( uint32_t min, uint32_t max );
void Button_Detect();
u8 CHRG_Detect(void);
void Battery_Mgr(void);
void Button_Menu(u8 *ButtonState);
void Button_Process(u8 *ButtonState);
void Battery_Led_Mgr(void);

//void  RelaySend(Watch_Type Btpye,u8 RelayCnt);
//void BeaconRelayStorage(void* ptr,Watch_Type Btpye);
#endif // __SX12XXEIGER_H__
