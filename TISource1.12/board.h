/*****************************************************************************
*
*  board.h - FRAM board definitions
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

#ifndef BOARD_H
#define	BOARD_H

#define SPI_CS_PORT_OUT    P1OUT
#define SPI_CS_PORT_DIR    P1DIR
#define SPI_CS_PORT_SEL    P1SEL0
#define SPI_CS_PORT_SEL2   P1SEL1
#define SPI_CS_PIN         BIT3

#define SPI_IRQ_PORT_DIR   P2DIR
#define SPI_IRQ_PORT_SEL   P2SEL0
#define SPI_IRQ_PORT_SEL2  P2SEL1
#define SPI_IRQ_PIN        BIT3

#define WLAN_EN_PORT_OUT   P4OUT
#define WLAN_EN_PORT_DIR   P4DIR
#define WLAN_EN_PORT_SEL   P4SEL0
#define WLAN_EN_PORT_SEL2  P4SEL1
#define WLAN_EN_PIN        BIT1

#define UART1_PORT   P2SEL0
#define UART1_PIN    BIT0
#define UART2_PORT   P2SEL1
#define UART2_PIN    BIT1

typedef enum
{
    NO_LED,
    LED1,
    LED2,
    LED3,
    LED4,
    LED5,
    LED6,
    LED7,
    LED8
} ledEnum;

typedef enum
{
    NO_LED_IND = NO_LED,
    CC3000_ON_IND = LED1,
    CC3000_ASSOCIATED_IND = LED2,
    CC3000_IP_ALLOC_IND = LED3,
    CC3000_SERVER_INIT_IND = LED4,
    CC3000_CLIENT_CONNECTED_IND = LED5,
    CC3000_SENDING_DATA_IND = LED6,
    CC3000_UNUSED1_IND = LED7,
    CC3000_FTC_IND = LED8,
} ledNames;

void pio_init();
void StartDebounceTimer();
void StopDebounceTimer();

void initLEDs();
long IsFTCflagSet();
void SetFTCflag();
void ClearFTCflag();
void RestoreSwitch();
long switchIsPressed();
long ReadWlanInterruptPin(void);
void WlanInterruptEnable();
void WlanInterruptDisable();
void WriteWlanPin( unsigned char val );
void unsolicicted_events_timer_init(void);
void unsolicicted_events_timer_disable(void);
void initClk(void);
void DissableSwitch();
void turnLedOn(char ledNum);
void turnLedOff(char ledNum);
void toggleLed(char ledNum);

void restartMSP430();
#endif
