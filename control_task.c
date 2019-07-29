/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifdef USE_BIOS
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>


/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#if defined(SOC_AM65XX)
#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>
#endif
#endif
#endif /* #ifdef USE_BIOS */

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>

#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <ti/sysbios/knl/Swi.h>

#if defined(SOC_AM65XX)
#include <ti/drv/sciclient/sciclient.h>
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define DELAY_VALUE       (500U)   /* 500 msec */
#define I2C_MPU_6065_INSTANCE 1
#define MPU_6050_SLAVE_ADDR 0x68
#define PWR_MGMT_1 107
#define ACCEL_CONFIG 28
#define GYRO_CONFIG 27

#define SMPLRT_DIV 25
#define CONFIG 26
#define GYRO_CONFIG 27
#define ACCEL_CONFIG 28
#define FIFO_EN 35
#define ACCEL_XOUT_H 59
#define ACCEL_XOUT_L 60


/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);
void AppLoopDelay(uint32_t delayVal);
void EICAS(UArg arg0, UArg arg1);
//void RNAV(UArg arg0, UArg arg1);
//void isrFunc();
//void swiFunc();
//void swiFunc_hwi();

void swiFunc_EICAS();
void IMUSetUp();
//void swiFunc_RNAV();
//void swiFunc_EW();
//void swiFunc_RADAR();
uint8_t readSensor(I2C_Handle h, uint8_t reg);





/* Callback function */
void AppGpioCallbackFxn(void);

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
/* GPIO clock and pinmux configurations */
extern void AppGPIOInit(void);
#endif

#if defined(idkAM574x) || defined(idkAM572x)
extern void GPIOApp_UpdateBoardInfo(void);
extern void GPIOAppUpdateConfig(uint32_t *gpioBaseAddr, uint32_t *gpioPin);
#endif

#ifdef SOC_AM65XX
/* Main domain GPIO interrupt events */
#define MAIN_GPIO_INTRTR_GPIO0_BANK0_INT (0x000000C0) /* GPIO port 0 bank 0 interrupt event #, input to MAIN_GPIO_INTRTR */
#define MAIN_GPIO_INTRTR_GPIO1_BANK0_INT (0x000000C8) /* GPIO port 1 bank 0 interrupt event #, input to MAIN_GPIO_INTRTR */

/* Main domain GPIO interrupt events */
#define WKUP_GPIO_INTRTR_GPIO0_BANK0_INT (0x0000003C) /* GPIO port 0 bank 0 interrupt event #, input to WKUP_GPIO_INTRTR */


/* Main to MCU GPIO interrupt router mux output events */
#define CSL_MAIN2MCU_INTRTR_PLS_GPIOMUX_INT0_DFLT_PLS  (0x00000000)
#define CSL_MAIN2MCU_INTRTR_PLS_GPIOMUX_INT31_DFLT_PLS (0x0000001F)

void GPIO_configIntRouter(uint32_t portNum, uint32_t pinNum, uint32_t gpioIntRtrOutIntNum, GPIO_v0_HwAttrs *cfg)
{
    GPIO_IntCfg       *intCfg;
    uint32_t           bankNum;

    intCfg = cfg->intCfg;

#if defined (am65xx_evm) || defined (am65xx_idk)
    
    /* no main domain GPIO pins directly connected to LEDs on GP EVM, 
       use WKUP domain GPIO pins which connected to LEDs on base board */
    cfg->baseAddr = CSL_WKUP_GPIO0_BASE;

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */

    /* WKUP GPIO int router input interrupt is the GPIO bank interrupt */
#if defined (__aarch64__)
    intCfg[pinNum].intNum = CSL_GIC0_INTR_WKUP_GPIOMUX_INTRTR0_BUS_OUTP_0 + bankNum;
#else
    intCfg[pinNum].intNum = CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_0 + bankNum;
#endif
    intCfg[pinNum].eventId = 0;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;
    
    /* Setup interrupt router configuration for gpio port/pin */
#else /* defined (am65xx_evm) */
    /* Use main domain GPIO pins directly connected to IDK EVM */

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */
    if (portNum == 0)
    {
        /* MAIN GPIO int router input interrupt is the GPIO bank interrupt */
#if defined (__aarch64__)
        intCfg[pinNum].intNum = CSL_GIC0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_0 + bankNum;
#else
        intCfg[pinNum].intNum = CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_0 + bankNum;
#endif
    }
    else
    {
#if defined (__aarch64__)
        intCfg[pinNum].intNum = CSL_GIC0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_6 + bankNum;
#else
        intCfg[pinNum].intNum = CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_6 + bankNum;
#endif
    }
    intCfg[pinNum].eventId = 0;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;
   
#endif /* defined (am65xx_evm) */
    GPIO_log("\nIntConfig:  portNum[%d],pinNum[%d],bankNum[%d], intNum[%d]",portNum,pinNum,bankNum,intCfg[pinNum].intNum);
}



#ifdef USE_BIOS
#if defined (__aarch64__)
Void InitMmu()
{
    Mmu_MapAttrs attrs;
    Bool         retVal;
    uint32_t     mapIdx = 0;

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = 0;
    retVal = Mmu_map(0x00100000, 0x00100000, 0x00900000, &attrs); /* Main MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x00400000, 0x00400000, 0x00001000, &attrs); /* PSC0          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x01800000, 0x01800000, 0x00100000, &attrs); /* gicv3          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02400000, 0x02400000, 0x000c0000, &attrs); /* dmtimer        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
    
    mapIdx++;
    retVal = Mmu_map(0x02800000, 0x02800000, 0x00040000, &attrs); /* uart           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02000000, 0x02000000, 0x00100000, &attrs); /* main I2C       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
    
    
    mapIdx++;
    retVal = Mmu_map(0x42120000, 0x42120000, 0x00001000, &attrs); /* Wkup I2C0       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02100000, 0x02100000, 0x00080000, &attrs); /* McSPI          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x00600000, 0x00600000, 0x00002000, &attrs); /* GPIO           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x42110000, 0x42110000, 0x00001000, &attrs); /* WKUP GPIO      */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x00a00000, 0x00a00000, 0x00040000, &attrs); /* MAIN INTR_ROUTERs */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x42200000, 0x42200000, 0x00001000, &attrs); /* WKUP INTR_ROUTER */
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }
    
    mapIdx++;
    retVal = Mmu_map(0x40f00000, 0x40f00000, 0x00020000, &attrs); /* MCU MMR0 CFG   */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x40d00000, 0x40d00000, 0x00002000, &attrs); /* PLL0 CFG       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x43000000, 0x43000000, 0x00020000, &attrs); /* WKUP MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02C40000, 0x02C40000, 0x00100000, &attrs); /* pinmux ctrl    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x2A430000, 0x2A430000, 0x00001000, &attrs); /* ctrcontrol0    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x030000000, 0x030000000, 0x10000000, &attrs); /* NAVSS used by sciclient  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }


    mapIdx++;
    retVal = Mmu_map(0x42000000, 0x42000000, 0x00001000, &attrs); /* PSC WKUP */
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }

    attrs.attrIndx = 7;
    mapIdx++;
    retVal = Mmu_map(0x80000000, 0x80000000, 0x03000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x70000000, 0x70000000, 0x04000000, &attrs); /* msmc           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
        System_printf("Mmu_map idx %d returned error %d", mapIdx, retVal);
        while(1);
    }

    return;
}
#endif /* #if defined (__aarch64__) */
#endif /* #ifdef USE_BIOS */
#endif /* #if defined(SOC_AM65XX) */

/*
 *  ======== Board_initI2C ========
 */
static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);



#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

#if defined(idkAM572x) || defined(idkAM574x)
    GPIOApp_UpdateBoardInfo();
#endif

    /* Modify the default GPIO configurations if necessary */
#if defined (am65xx_evm) || defined (am65xx_idk)

    GPIO_configIntRouter(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, 0, &gpio_cfg);

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#endif
}

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;
int interrupt_ = 0;

Task_Handle taskEICAS;
Task_Handle taskRNAV;
Task_Handle taskEW;
Task_Handle taskRADAR;
Swi_Handle swi0;
int iSwitchTask = 0;

Task_Params taskEICASParams;
Task_Params taskRNAVParams;
Task_Params taskEWParams;
Task_Params taskRADARParams;

Clock_Handle Clock;
Clock_Handle Clock_EICAS;
Clock_Handle Clock_RNAV;
Clock_Handle Clock_EW;
Clock_Handle Clock_RADAR;


Semaphore_Handle sem_EICAS;
Semaphore_Handle sem_RNAV;
Semaphore_Handle sem_EW;
Semaphore_Handle sem_RADAR;


I2C_HwAttrs i2c_cfg;





/*
 *  ======== test function ========
 */
/*void gpio_test_1(UArg arg0, UArg arg1)
{
    Board_initGPIO();
    while(1){
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_HIGH);
        Osal_delay(DELAY_VALUE);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
        Osal_delay(DELAY_VALUE);

}

}


void gpio_test_2(UArg arg0, UArg arg1)
{

    while(1){
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
        Osal_delay(2*DELAY_VALUE);
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
        Osal_delay(2*DELAY_VALUE);

}

}
*/
int pos = 0;

void EICAS(UArg arg0, UArg arg1)
{
    IMUSetUp();


    while(1){
//8ms
        //GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
        //AppLoopDelay(800*DELAY_VALUE);
        //GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);

        I2C_Params i2cParams;
        I2C_Handle handle = NULL;
        I2C_Params_init(&i2cParams);
        handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);



        uint8_t Ho = 0;
                Ho = readSensor(handle, 0x75);
                UART_printf("CONFIG: %x \n",Ho);


        uint8_t CONG = 0;
        CONG = readSensor(handle, CONFIG);
        UART_printf("CONFIG: %x \n",CONG);

        uint8_t SMPLRT = 0;
        SMPLRT = readSensor(handle, SMPLRT_DIV);
        UART_printf("SMPLRT_DIV: %x \n",SMPLRT);

        uint8_t GYRO_C = 0;
        GYRO_C = readSensor(handle, GYRO_CONFIG);
        UART_printf("GYRO_CONFIG: %x \n",GYRO_C);

        uint8_t ACCEL_C = 0;
        ACCEL_C = readSensor(handle, ACCEL_CONFIG);
        UART_printf("ACCEL_CONFIG: %x \n",ACCEL_C);

        uint8_t FIFO_E = 0;
        FIFO_E = readSensor(handle, FIFO_EN);
        UART_printf("FIFO_EN: %x \n",FIFO_E);

        uint8_t ACCEL_H = 0;
        ACCEL_H = readSensor(handle, ACCEL_XOUT_H);
        UART_printf("ACCEL_XOUT_H: %x \n",ACCEL_H);

        uint8_t ACCEL_L = 0;
        ACCEL_L = readSensor(handle, ACCEL_XOUT_L);
        UART_printf("ACCEL_XOUT_L: %x \n",ACCEL_L);


        //readSensor(handle, GYRO_CONFIG, 0x10);
        I2C_close(handle);
       // UART_printf("Acc/Gyro Setup Ended... \n");


        //AppLoopDelay(DELAY_VALUE);


        Semaphore_pend(sem_EICAS, BIOS_WAIT_FOREVER);





    }

}
/*
void RNAV(UArg arg0, UArg arg1)
{

    while(1){

// 832ms
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_HIGH);
        AppLoopDelay(8000*DELAY_VALUE);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
        //AppLoopDelay(DELAY_VALUE);
        Semaphore_pend(sem_RNAV, BIOS_WAIT_FOREVER);


    }

}


void EW(UArg arg0, UArg arg1)
{

    while(1){
  //      if(interrupt_ == 1){
// 832ms
            Semaphore_pend(sem_EW, BIOS_WAIT_FOREVER);

            GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
            AppLoopDelay(1000*DELAY_VALUE);
            GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
        //AppLoopDelay(DELAY_VALUE);
    //    }

    }


}

void RADAR(UArg arg0, UArg arg1)
{

    while(1){

// 832ms
        Semaphore_pend(sem_RADAR, BIOS_WAIT_FOREVER);

//        if(interrupt_ == 1){
            GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_HIGH);
            AppLoopDelay(800*DELAY_VALUE);
            GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
        //AppLoopDelay(DELAY_VALUE);
        //UART_printStatus("\n flag =1 \n");

//        }


    }


}
*/




/*
void gpio_test_0(UArg arg0, UArg arg1)
{

    while(1){
//8ms
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_HIGH);
        AppLoopDelay(800*DELAY_VALUE);
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
        //AppLoopDelay(DELAY_VALUE);
        Semaphore_pend(semTask, BIOS_WAIT_FOREVER);


    }

}

void gpio_test_1(UArg arg0, UArg arg1)
{

    while(1){

// 832ms
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
        AppLoopDelay(8000*DELAY_VALUE);
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
        //AppLoopDelay(DELAY_VALUE);
        Semaphore_pend(semTask, BIOS_WAIT_FOREVER);


    }

}






*/


/*
void gpio_test_0(UArg arg0, UArg arg1)
{

    while(1){
        AppLoopDelay(DELAY_VALUE);
        if(iSwitchTask){
            iSwitchTask = 0;
            Task_yield();
        }

    }

}


void gpio_test_1(UArg arg0, UArg arg1)
{

    while(1){
        AppLoopDelay(DELAY_VALUE);
        if(iSwitchTask){
            iSwitchTask = 0;
            Task_yield();
        }

    }

}

*/

uint8_t readSensor(I2C_Handle h, uint8_t reg){
    uint8_t rxData = 0;
    uint8_t txData = 0;
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    memset(&txData, 0x00, sizeof(txData));
    t.slaveAddress = MPU_6050_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;
    transferStatus = I2C_transfer(h, &t);
    if(I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
    return rxData;
}

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val){
    uint8_t txData[2] = {0,0};
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    //memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = MPU_6050_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if(I2C_STS_SUCCESS != transferStatus){
           UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
       }

}

void IMUSetUp(){
    I2C_Params i2cParams;
    I2C_Handle handle;
    I2C_Params_init(&i2cParams);
    //i2cParams.bitRate = I2C_400kHz;
    UART_printf("Antes do Open \n");
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    if(handle == NULL) UART_printf("ERROR");

    UART_printf("Init Acc/Gyro Setup... \n");
    UART_printf("Device Reset... \n");
    writeSensor(handle, PWR_MGMT_1, 0x00);
    UART_printf("Accelerometer Config... \n");
    writeSensor(handle, ACCEL_CONFIG, 0x10);
    UART_printf("Gyro Config... \n");
    writeSensor(handle, GYRO_CONFIG, 0x10);
    I2C_close(handle);
    UART_printf("Acc/Gyro Setup Ended... \n");
}






//typedef struct I2C_HwAttrs_s

#ifdef USE_BIOS
/*
 *  ======== main =========
 */
int main(void)
{



 #if defined(SOC_AM65XX) || defined(SOC_J7)
    Sciclient_ConfigPrms_t  sciClientCfg;
#endif

#if defined(SOC_AM65XX) || defined(SOC_J7)
    Sciclient_configPrmsInit(&sciClientCfg);
    Sciclient_init(&sciClientCfg);
#endif
    Board_initGPIO();
    //UART_printf("Teste123 \n");



    /* Call board init functions */


    I2C_socGetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

    I2C_socSetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);
    I2C_init();


    Task_Params_init(&taskEICASParams);
    taskEICASParams.stackSize = 0x1400;
    taskEICASParams.priority = 15;
    Clock_Params  clkParams;
    Clock_Params_init(&clkParams);

    clkParams.startFlag = TRUE;
    clkParams.period = 20;
    Clock_EICAS = Clock_create(swiFunc_EICAS,1, &clkParams, NULL);

    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;

    sem_EICAS = Semaphore_create(0, &semParams, NULL);
    taskEICAS = Task_create(EICAS, &taskEICASParams, NULL);



  //  GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_CFG_OUTPUT);


   // Swi_Params swiParams;
   // Swi_Params_init(&swiParams);
   // swi0 = Swi_create(swiFunc, &swiParams, NULL);
    Swi_enable();
    /*Task_Params_init(&taskRNAVParams);
    Task_Params_init(&taskEWParams);
    Task_Params_init(&taskRADARParams);

    taskRNAVParams.stackSize = 0x1400;
    taskEICASParams.stackSize = 0x1400;
    taskRNAVParams.stackSize = 0x1400;


    taskEICASParams.priority = 15;
    taskRNAVParams.priority = 1;
    taskEWParams.priority = 2;
    taskRADARParams.priority = 5;


    Clock_Params  clkParams;
    Clock_Params_init(&clkParams);

    clkParams.startFlag = TRUE;
    clkParams.period = 1;
    Clock_EICAS = Clock_create(swiFunc_EICAS,1, &clkParams, NULL);

    clkParams.period = 20;
    clkParams.startFlag = TRUE;
    Clock_RNAV = Clock_create(swiFunc_RNAV,1, &clkParams, NULL);

    clkParams.period = 4;
    clkParams.startFlag = FALSE;
//    clkParams.startFlag = TRUE;
    Clock_EW = Clock_create(swiFunc_EW,1, &clkParams, NULL);
    Clock_stop(Clock_EW);

    clkParams.period = 2;
    clkParams.startFlag = FALSE;
 //   clkParams.startFlag = TRUE;
    Clock_RADAR = Clock_create(swiFunc_RADAR,1, &clkParams, NULL);
    Clock_stop(Clock_RADAR);

    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;

    sem_EICAS = Semaphore_create(0, &semParams, NULL);
    sem_RNAV = Semaphore_create(0, &semParams, NULL);
    sem_EW = Semaphore_create(0, &semParams, NULL);
    sem_RADAR = Semaphore_create(0, &semParams, NULL);



   // Clock_Params  clkParams;
    //Clock_Params_init(&clkParams);

    clkParams.startFlag = FALSE;
    clkParams.period = 80;


    Clock = Clock_create(swiFunc_hwi,80, &clkParams, NULL);



    taskEICAS = Task_create(EICAS, &taskEICASParams, NULL);
    taskRNAV = Task_create(RNAV, &taskRNAVParams, NULL);
    taskEW = Task_create(EW, &taskEICASParams, NULL);
    taskRADAR = Task_create(RADAR, &taskRNAVParams, NULL);


    GPIODirModeSet(SOC_GPIO_2_REGS, 8, GPIO_CFG_OUTPUT);
    //lCD data2
    GPIODirModeSet(SOC_GPIO_2_REGS, 10, GPIO_CFG_OUTPUT);
    //LCD data4

    GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 22, GPIO_CFG_OUTPUT);

    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 24, GPIO_CFG_OUTPUT);

    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);

    hwiParams.enableInt = FALSE;

    int intNum = 98;
    Hwi_create(intNum, isrFunc, &hwiParams, NULL);

    Hwi_enableInterrupt(intNum);
    GPIODirModeSet(SOC_GPIO_1_REGS,16,GPIO_CFG_INPUT);
    GPIOIntTypeSet(SOC_GPIO_1_REGS, 16,GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(SOC_GPIO_1_REGS,GPIO_INT_LINE_1,16);


    Swi_Params swiParams;
    Swi_Params_init(&swiParams);
    swi0 = Swi_create(swiFunc, &swiParams, NULL);
    Swi_enable();
    */
#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
    AppGPIOInit();
#endif

    /* Start BIOS */
    BIOS_start();
    return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal)
{
    Osal_delay(delayVal);
}

/*
 *  ======== AppLoopDelay ========
 */
void AppLoopDelay(uint32_t delayVal)
{
    volatile uint32_t i;

    for (i = 0; i < (delayVal); i++)
        ;
}


void isrFunc(){
    //UART_printStatus("\n ISR triggered.  1 \n");
   // GPIOPinWrite(SOC_GPIO_2_REGS, 10, GPIO_PIN_HIGH);

    Swi_post(swi0);
    GPIOPinIntClear(SOC_GPIO_1_REGS,GPIO_INT_LINE_1,16);
    //GPIOPinWrite(SOC_GPIO_2_REGS, 10, GPIO_PIN_LOW);
}


/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
    /* Toggle LED1 */
    GPIO_toggle(USER_LED1);
    AppLoopDelay(DELAY_VALUE);
    gpio_intr_triggered = 1;
}


void myNewSwitch(Task_Handle prev, Task_Handle next){
  //  int GPIOPin2TurnOn = 0;
  //  int GPIOPin2TurnOff = 0;

   /* if (prev == task0){

        GPIOPin2TurnOff = 21;
     //   UART_printStatus("\n Leaving Task 0 \n");

    }
    else if(prev == task1){
       // UART_printStatus("\n Leaving Task 1. \n");
        GPIOPin2TurnOff = 22;
    }

    if(next == task0){
        //UART_printStatus("\n Entering Task 0. \n");
        GPIOPin2TurnOn = 21;

    }
    else if( next == task1){
        //UART_printStatus("\n Entering Task 1. \n");
        GPIOPin2TurnOn = 22;

    }

    if(GPIOPin2TurnOn !=0){
        GPIOPinWrite(SOC_GPIO_1_REGS, GPIOPin2TurnOn, GPIO_PIN_HIGH);

    }


    if(GPIOPin2TurnOff !=0){
        GPIOPinWrite(SOC_GPIO_1_REGS, GPIOPin2TurnOff, GPIO_PIN_LOW);

    }
*/

}
void swiFunc(){
    Hwi_disableInterrupt(98);
    if(interrupt_ == 0){
     //   UART_printStatus("\n 0 \n");

        interrupt_ = 1;

        Clock_start(Clock_EW);

        Clock_start(Clock_RADAR);
    }
    else {
       // UART_printStatus("\n 1 \n");

        Clock_stop(Clock_EW);
        Clock_stop(Clock_RADAR);
        interrupt_ = 0;
    }
   Clock_start(Clock);

    //iSwitchTask = 1;
    //Semaphore_post(semTask);
}


void swiFunc_hwi(){
    //iSwitchTask = 1;
   // UART_printStatus("\n ISR triggered. \n");

//    Semaphore_post(sem_EICAS);
    Clock_stop(Clock);

    Hwi_enableInterrupt(98);
    GPIOPinIntClear(SOC_GPIO_1_REGS,GPIO_INT_LINE_1,16);



}

void swiFunc_EICAS(){

    //iSwitchTask = 1;
    Semaphore_post(sem_EICAS);
}

void swiFunc_RNAV(){
    //UART_printStatus("\n ISR triggered. \n");
    //iSwitchTask = 1;
    Semaphore_post(sem_RNAV);
}

void swiFunc_EW(){
    //iSwitchTask = 1;
    Semaphore_post(sem_EW);
}

void swiFunc_RADAR(){
    //iSwitchTask = 1;
    Semaphore_post(sem_RADAR);
}





