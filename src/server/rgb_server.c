#include "rgb_server.h"
#include "rgb.h"
#include "octo.h"

#include "stm32f103xb.h"

/* private functions */
static _Bool prvCheckDeviceID();
static void prvRgbSetPin(uint8_t sensorPosition);
static void prvRgbResetPins();
void prvRgbInit();
struct xRGB prvGetRgb(uint8_t ucPosition);
uint8_t prvGetRed(uint8_t ucPosition);
uint8_t prvGetGreen(uint8_t ucPosition);
uint8_t prvGetBlue(uint8_t ucPosition);


typedef struct xMESSAGE_I2C
{
    uint8_t ucAddress;              /* I2C slave address */
    uint8_t *pucWriteBytes;         /* Byte that should be send to the slave */
    uint8_t ucWriteBytesLength;     /* Amount of bytes to send (which can be 1 or 2) */
    uint8_t ucWriteFinished;        /* ALWAYS set this to 0 */
    uint8_t ucRead;                 /* 0 = do not read, 1 = read 1 byte from slave, 2 = read 2 bytes from slave */
    uint8_t *pucReadBytes;          /* Bytes received from the slave */
} MessageI2c;

static _Bool xI2cPeripheralBusy = 0;
static MessageI2c xI2cIsrMessage;
static MessageI2c xI2cDummyMessage;
QueueHandle_t xI2cToIsr;
QueueHandle_t xI2cFromIsr;

void I2C1_EV_IRQ_handler(void)
{
    if(I2C1->SR1 & I2C_SR1_SB)		/* (0x01) Start bit has been send */
	{
        if(!xI2cPeripheralBusy)
        {
            if(xQueuePeekFromISR(xI2cToIsr, &xI2cIsrMessage))
                xI2cPeripheralBusy = 1; 
        }
        
        if(xI2cPeripheralBusy)
        {
            if(!xI2cIsrMessage.ucWriteFinished)
            {
                I2C1->SR1;
                I2C1->DR = (xI2cIsrMessage.ucAddress << 1) | 0;                
            } else if(xI2cIsrMessage.ucWriteFinished)
            {
                I2C1->SR1;
                I2C1->DR = (xI2cIsrMessage.ucAddress << 1) | 1;
            }
        }           
 
        I2C1->SR1 &= ~(I2C_SR1_SB); /* Acknowledge the interrupt */				
        return;
	}

	if(I2C1->SR1 & I2C_SR1_ADDR)	/* (0x02) I2C slave address has been send */
	{
        if(xI2cIsrMessage.ucWriteFinished)
        {
            if(xI2cIsrMessage.ucRead == 1)
            {
                I2C1->CR1 &= ~(I2C_CR1_ACK);
                GPIOB->CRL &= ~(GPIO_CRL_MODE6_1);      /* Errata */
            } else if(xI2cIsrMessage.ucRead == 2)
            {
                GPIOB->CRL &= ~(GPIO_CRL_MODE6_1);      /* Change PB6 from I2C_CLK to I/O pin (errata 2.13.1) */
                I2C1->CR1 |= (I2C_CR1_POS);             /* Generate a NACK on the next receiving byte */
                
                I2C1->SR1;                              /* Acknowledge the interrupt */
                I2C1->SR2;  
            
                I2C1->CR1 &= ~(I2C_CR1_ACK);            /* Do not send an acknowledgement */
                GPIOB->CRL |= (GPIO_CRL_MODE6_1);       /* Change PB6 from I/O pin to I2C_CLK (errata 2.13.1) */
                return;
            }
        } else if(!xI2cIsrMessage.ucWriteFinished)
        {           
            I2C1->SR1;                  /* FIRST acknowledge the interrupt before write to data register */
            I2C1->SR2;                  /* See Figure 271 in the reference manual, transfer sequence diagram for master transmitter */

            I2C1->DR = xI2cIsrMessage.pucWriteBytes[0];
            xI2cIsrMessage.ucWriteBytesLength--;    /* One less byte remaining to send */
        }    
         
        I2C1->SR1;                  /* Acknowledge the interrupt */
        I2C1->SR2;                  
        I2C1->SR1 &= ~(I2C_SR1_ADDR);
        if(!xI2cIsrMessage.ucWriteBytesLength)
            I2C1->CR1 |= (I2C_CR1_STOP);
        return;
	}

    if(I2C1->SR1 & I2C_SR1_RXNE)	/* Receive data register not empty */
	{
        if(xI2cIsrMessage.ucRead == 1)
        {
            I2C1->SR1;                  /* Dummy read to clear the BTF (byte transfer finished) flag */
            xI2cIsrMessage.pucReadBytes[0] = I2C1->DR;  /* Must be the next action after the dummy SR1 read */
               
            while(I2C1->CR1 & I2C_CR1_STOP);
            I2C1->CR1 |= I2C_CR1_ACK;       /* Set acknowledgement after a byte is received on again */ 
            GPIOB->CRL |= (GPIO_CRL_MODE6_1);   /* Errata */

            xI2cPeripheralBusy = 0;         /* We finished this request and are free to accept a new one */
            xQueueReceiveFromISR(xI2cToIsr, &xI2cDummyMessage, NULL);   /* Delete the peeked request, basically acknowledge it */
            xQueueSendFromISR(xI2cFromIsr, &xI2cIsrMessage, NULL);      /* Send the response to the caller */

            I2C1->SR1 &= ~(I2C_SR1_RXNE);   /* Acknowledge the interrupt */
	        I2C1->DR;
            return;
        } else if (xI2cIsrMessage.ucRead == 2)
        {
            I2C1->CR1 |= (I2C_CR1_STOP);
            xI2cIsrMessage.pucReadBytes[0] = I2C1->DR;
            
            while(I2C1->CR1 & I2C_CR1_STOP);
            I2C1->CR1 &= ~(I2C_CR1_POS);
            I2C1->CR1 |= (I2C_CR1_ACK);
            
            xI2cIsrMessage.pucReadBytes[1] = I2C1->DR;
               
            xI2cPeripheralBusy = 0;
            xQueueReceiveFromISR(xI2cToIsr, &xI2cDummyMessage, NULL);
            
            xQueueSendFromISR(xI2cFromIsr, &xI2cIsrMessage, NULL);           
            
        }   
    }

	if(I2C1->SR1 & I2C_SR1_BTF)		/* (0x04) Byte transfer is finished */
	{
        if(xI2cIsrMessage.ucWriteFinished)
        {
            /* This should never be reached, but handled by RxNE interrupt */
        } else if(!xI2cIsrMessage.ucWriteFinished)
        {
            if(xI2cIsrMessage.ucWriteBytesLength)
            {
                I2C1->DR = xI2cIsrMessage.pucWriteBytes[1];     
                xI2cIsrMessage.ucWriteBytesLength--;
 
                I2C1->SR1 &= ~(I2C_SR1_BTF);/* Acknowledge the interrupt */
	            return; 
            }

            I2C1->CR1 |= (I2C_CR1_STOP);        /* Send a stop bit since we only want to write 1 byte */
            xI2cIsrMessage.ucWriteFinished = 1; /* We succesfully wrote our byte to the slave */  

            if(xI2cIsrMessage.ucRead)           
            {
                I2C1->CR1 |= (I2C_CR1_START);   /* Generate a new START bit (for the read action) */ 
            } else if(!xI2cIsrMessage.ucRead)
            {
                xI2cPeripheralBusy = 0;         /* We finished this request and are free to accept a new one */
                xQueueReceiveFromISR(xI2cToIsr, &xI2cDummyMessage, NULL);/* Delete the peeked request, basically acknowledge it */
                xQueueSendFromISR(xI2cFromIsr, &xI2cIsrMessage, NULL);  /* Send the response to the caller */
            }
        }         

        I2C1->SR1 &= ~(I2C_SR1_BTF);/* Acknowledge the interrupt */
	    return;
    }

	if(I2C1->SR1 & I2C_SR1_TXE)		/* Transmit data register empty */
	{

        I2C1->SR1 &= ~(I2C_SR1_TXE);/* Acknowledge the interrupt */ 
	    return;
    }
}
/*-----------------------------------------------------------*/ 

QueueHandle_t xToRgbServer;
QueueHandle_t xFromRgbServer;

void vTaskRgbServer( void * pvParameters )
{
RgbServerMessage_t xMessage;
RgbColours_t xColours;    

    xToRgbServer = xQueueCreate(RGB_SERVER_MESSAGE_QUEUE_SIZE, sizeof(RgbServerMessage_t));
    xFromRgbServer = xQueueCreate(RGB_SERVER_MESSAGE_QUEUE_SIZE, sizeof(RgbColours_t)); 

    xI2cToIsr = xQueueCreate(1, sizeof(MessageI2c));
    xI2cFromIsr = xQueueCreate(1, sizeof(MessageI2c));

    prvRgbInit();       /* Initialise the TCS34725 */

    while(1)
    {
        if(xQueueReceive(xToRgbServer, &xMessage, portMAX_DELAY) != pdTRUE)
            continue;

        xColours = prvGetRgb(xMessage.ePlaceholder);
   
        if(xQueueSend(xMessage.xQueueDestination, &xColours, portMAX_DELAY) != pdTRUE)
            continue;  
    }
}
/*-----------------------------------------------------------*/ 

/**********************************Private function begin **********************************/

/*
 * initialize function for TCS34725.
 */
void prvRgbInit()
{
int uI;  
MessageI2c xConfigureSensor;
MessageI2c xI2cResponse;
uint8_t ucI2cResult[2];
uint8_t ucI2cWrite[2];

    for(uI = 0; uI < SENSORCOUNT; uI++) 
    {
        prvRgbSetPin(uI);
        if(prvCheckDeviceID())
        {
            /* Enables the adc */
            /* Select enable register(0x00) */
            /* Power ON, RGBC enable, wait time disable(0x03) */
            union EnableRegister enRegister;
            enRegister.Bits.ADCEnable = 1;
            enRegister.Bits.powerOn = 1;

            ucI2cWrite[0] = (TCS34725_ENABLE | TCS34725_COMMAND_BIT);
            ucI2cWrite[1] = (enRegister.value);

            xConfigureSensor.ucAddress = 0x29;
            xConfigureSensor.pucWriteBytes = ucI2cWrite;
            xConfigureSensor.ucWriteBytesLength = 2;
            xConfigureSensor.ucWriteFinished = 0;
            xConfigureSensor.ucRead = 0;
            xConfigureSensor.pucReadBytes = ucI2cResult;

            if(xQueueSend(xI2cToIsr, &xConfigureSensor, portMAX_DELAY) != pdTRUE)
                return;
            I2C1->CR1 |= (1 << I2C_CR1_START_Pos);


            if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
                return;

            vTaskDelay(pdMS_TO_TICKS(10)); 
 
 
            /* Set intergration time */
            /* Select ALS time register(0x81) */
            /* Atime = 700 ms(0x00) */
            union RGBCTimingRegister xTimeReg;
            xTimeReg.value = 0x00;
 
            ucI2cWrite[0] = (TCS34725_RGBCTIME | TCS34725_COMMAND_BIT);
            ucI2cWrite[1] = (xTimeReg.value);

            xConfigureSensor.ucAddress = 0x29;
            xConfigureSensor.pucWriteBytes = ucI2cWrite;
            xConfigureSensor.ucWriteBytesLength = 2;
            xConfigureSensor.ucWriteFinished = 0;
            xConfigureSensor.ucRead = 0;
            xConfigureSensor.pucReadBytes = ucI2cResult;
 
            if(xQueueSend(xI2cToIsr, &xConfigureSensor, portMAX_DELAY) != pdTRUE)
                return;
            I2C1->CR1 |= (1 << I2C_CR1_START_Pos);


            if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
                return;

            vTaskDelay(pdMS_TO_TICKS(10)); 
 

            /* Set gain */
            /* Select Wait Time register(0x83) */
            /* WTIME : 2.4ms(0xFF) */
            union WaitTimeRegister xWaitReg;
            xWaitReg.value = 0xFF;
 
            ucI2cWrite[0] = (TCS34725_WAITTIME | TCS34725_COMMAND_BIT);
            ucI2cWrite[1] = (xWaitReg.value);

            xConfigureSensor.ucAddress = 0x29;
            xConfigureSensor.pucWriteBytes = ucI2cWrite;
            xConfigureSensor.ucWriteBytesLength = 2;
            xConfigureSensor.ucWriteFinished = 0;
            xConfigureSensor.ucRead = 0;
            xConfigureSensor.pucReadBytes = ucI2cResult;
 
            if(xQueueSend(xI2cToIsr, &xConfigureSensor, portMAX_DELAY) != pdTRUE)
                return;
            I2C1->CR1 |= (1 << I2C_CR1_START_Pos);


            if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
                return;

            vTaskDelay(pdMS_TO_TICKS(10)); 


            /* Select control register(0x8F) */
            /* AGAIN = 1x(0x00) */
            /* The gain register determines the sensitivity of the diodes */
            /* 00=1x, 01=4x, 10=16x, 11=60x Gain*/
            union ControlRegister xCntrlReg;
            xCntrlReg.value = 0x00;

            ucI2cWrite[0] = (TCS34725_CNTRLREG | TCS34725_COMMAND_BIT);
            ucI2cWrite[1] = (xCntrlReg.value);

            xConfigureSensor.ucAddress = 0x29;
            xConfigureSensor.pucWriteBytes = ucI2cWrite;
            xConfigureSensor.ucWriteBytesLength = 2;
            xConfigureSensor.ucWriteFinished = 0;
            xConfigureSensor.ucRead = 0;
            xConfigureSensor.pucReadBytes = ucI2cResult;
 
            if(xQueueSend(xI2cToIsr, &xConfigureSensor, portMAX_DELAY) != pdTRUE)
                return;
            I2C1->CR1 |= (1 << I2C_CR1_START_Pos);


            if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
                return;

            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
        else
        {
            TCS34725_SENSOR1_8_VALUES = 0x00ff0000;
        }
    }
}
/*-----------------------------------------------------------*/ 

/*
 * Returns the value of of each colour including clear (0-255) of the given rgb sensor.
 */
struct xRGB prvGetRgb(uint8_t ucPosition)
{
RgbColours_t xColours;
    
    prvRgbSetPin(ucPosition);
    vTaskDelay(pdMS_TO_TICKS(10));

    xColours.ulRed = prvGetRed(ucPosition);
    xColours.ulGreen = prvGetGreen(ucPosition);
    xColours.ulBlue = prvGetBlue(ucPosition);
    
    prvRgbResetPins();
    return xColours;
}
/*-----------------------------------------------------------*/ 

/*
 * Returns the value of red(0-255) of the given rgb sensor.
 */
uint8_t prvGetRed(uint8_t ucPosition)
{
MessageI2c xSensorRed;
MessageI2c xSensorClear;
MessageI2c xI2cResponse;
uint8_t ucI2cRed[2];
uint8_t ucI2cClear[2];
uint8_t ucI2cWrite[2]; 

float fRed;
uint16_t usClear;
uint16_t usRed;

    xSensorClear.ucAddress = 0x29;    
    xSensorClear.pucWriteBytes = ucI2cWrite;
    xSensorClear.ucWriteBytesLength = 1;
    xSensorClear.ucWriteFinished = 0;
    xSensorClear.ucRead = 2;
    xSensorClear.pucReadBytes = ucI2cClear;

    ucI2cWrite[0] = (TCS34725_CDATA | TCS34725_COMMAND_BIT);

    if(xQueueSend(xI2cToIsr, &xSensorClear, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0;
 
    usClear = (ucI2cClear[1] << 8) | ucI2cClear[0];

    ucI2cWrite[0] = (TCS34725_RDATA | TCS34725_COMMAND_BIT);

    xSensorRed.ucAddress = 0x29;    
    xSensorRed.pucWriteBytes = ucI2cWrite;
    xSensorRed.ucWriteBytesLength = 1;
    xSensorRed.ucWriteFinished = 0;
    xSensorRed.ucRead = 2;
    xSensorRed.pucReadBytes = ucI2cRed;

    if(xQueueSend(xI2cToIsr, &xSensorRed, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0; 

    usRed = (ucI2cRed[1] << 8) | ucI2cRed[0];     
    
    fRed = (float) ((float) usRed / (float) usClear) * 255.0;
    return (uint8_t) fRed;
}
/*-----------------------------------------------------------*/ 

/*
 * Returns the value of green(0-255) of the given rgb sensor.
 */
uint8_t prvGetGreen(uint8_t ucPosition)
{ 
MessageI2c xSensorGreen;
MessageI2c xSensorClear;
MessageI2c xI2cResponse;
uint8_t ucI2cGreen[2];
uint8_t ucI2cClear[2];
uint8_t ucI2cWrite[2]; 

float fGreen;
uint16_t usClear;
uint16_t usGreen;

    xSensorClear.ucAddress = 0x29;    
    xSensorClear.pucWriteBytes = ucI2cWrite;
    xSensorClear.ucWriteBytesLength = 1;
    xSensorClear.ucWriteFinished = 0;
    xSensorClear.ucRead = 2;
    xSensorClear.pucReadBytes = ucI2cClear;

    ucI2cWrite[0] = (TCS34725_CDATA | TCS34725_COMMAND_BIT);

    if(xQueueSend(xI2cToIsr, &xSensorClear, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0;
 
    usClear = (ucI2cClear[1] << 8) | ucI2cClear[0];

    ucI2cWrite[0] = (TCS34725_GDATA | TCS34725_COMMAND_BIT);

    xSensorGreen.ucAddress = 0x29;    
    xSensorGreen.pucWriteBytes = ucI2cWrite;
    xSensorGreen.ucWriteBytesLength = 1;
    xSensorGreen.ucWriteFinished = 0;
    xSensorGreen.ucRead = 2;
    xSensorGreen.pucReadBytes = ucI2cGreen;

    if(xQueueSend(xI2cToIsr, &xSensorGreen, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0; 

    usGreen = (ucI2cGreen[1] << 8) | ucI2cGreen[0];     
    
    fGreen = (float) ((float) usGreen / (float) usClear) * 255.0;
    return (uint8_t) fGreen;
}
/*-----------------------------------------------------------*/ 

/*
 * Returns the value of blue(0-255) of the given rgb sensor.
 */
uint8_t prvGetBlue(uint8_t ucPosition)
{
MessageI2c xSensorBlue;
MessageI2c xSensorClear;
MessageI2c xI2cResponse;
uint8_t ucI2cBlue[2];
uint8_t ucI2cClear[2];
uint8_t ucI2cWrite[2]; 

float fBlue;
uint16_t usClear;
uint16_t usBlue;

    xSensorClear.ucAddress = 0x29;    
    xSensorClear.pucWriteBytes = ucI2cWrite;
    xSensorClear.ucWriteBytesLength = 1;
    xSensorClear.ucWriteFinished = 0;
    xSensorClear.ucRead = 2;
    xSensorClear.pucReadBytes = ucI2cClear;

    ucI2cWrite[0] = (TCS34725_CDATA | TCS34725_COMMAND_BIT);

    if(xQueueSend(xI2cToIsr, &xSensorClear, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0;
 
    usClear = (ucI2cClear[1] << 8) | ucI2cClear[0];

    ucI2cWrite[0] = (TCS34725_BDATA | TCS34725_COMMAND_BIT);

    xSensorBlue.ucAddress = 0x29;    
    xSensorBlue.pucWriteBytes = ucI2cWrite;
    xSensorBlue.ucWriteBytesLength = 1;
    xSensorBlue.ucWriteFinished = 0;
    xSensorBlue.ucRead = 2;
    xSensorBlue.pucReadBytes = ucI2cBlue;

    if(xQueueSend(xI2cToIsr, &xSensorBlue, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0; 

    usBlue = (ucI2cBlue[1] << 8) | ucI2cBlue[0];     
    
    fBlue = (float) ((float) usBlue / (float) usClear) * 255.0;
    return (uint8_t) fBlue;
}
/*-----------------------------------------------------------*/ 

/*
 *  Reset pins.
 */
static void prvRgbResetPins()
{
    GPIOA->BSRR = (0x00FF0000); /* Clear GPIO A0 to and include A7 */
    GPIOB->BSRR = (0xF0000000); /* Clear GPIO B12 to and include B15 */
}
/*-----------------------------------------------------------*/ 

/*
 * Used for setting rgb sensor pin.
 */
static void prvRgbSetPin(uint8_t ucSensorPosition)
{
    /* Reset pins */
    prvRgbResetPins();
    if(!ucSensorPosition)
    {
        GPIOA->BSRR = (0x00000001);
    }
    else if(ucSensorPosition > 0 &&  ucSensorPosition < 8)
    {
        GPIOA->BSRR = (0x00000001 << ucSensorPosition);
    }
    else
    {
        GPIOB->BSRR = (0x00001000 << (ucSensorPosition - 8));
    }
}
/*-----------------------------------------------------------*/ 

/*
 * Checks if ID is equal to 0x44 so the corresponding sensor is either TCS34721 or TCS34725.
 */
static _Bool prvCheckDeviceID()
{
MessageI2c xDeviceId;
MessageI2c xI2cResponse;
uint8_t ucI2cResult[2];
uint8_t ucI2cWrite[2];

    ucI2cWrite[0] = (TCS34725_ID | TCS34725_COMMAND_BIT);

    xDeviceId.ucAddress = 0x29;    
    xDeviceId.pucWriteBytes = ucI2cWrite;
    xDeviceId.ucWriteBytesLength = 1;
    xDeviceId.ucWriteFinished = 0;
    xDeviceId.ucRead = 1;
    xDeviceId.pucReadBytes = ucI2cResult;

    if(xQueueSend(xI2cToIsr, &xDeviceId, portMAX_DELAY) != pdTRUE)
        return 0;
    I2C1->CR1 |= (1 << I2C_CR1_START_Pos);


    if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
        return 0;

    if(ucI2cResult[0] == 0x44)
        return 1;

    return 0;
}
/*--------------------------------------------------------------------------------*/
