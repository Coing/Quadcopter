/**
 ******************************************************************************
 * @file OptimizedI2Cexamples/src/I2CRoutines.c
 * @author  MCD Application Team
 * @version  V4.0.0
 * @date  06/18/2010
 * @brief  Contains the I2Cx slave/Master read and write routines.
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "i2croutines.h"

#include "i2croutines.h"

#include "i2cdev.h"
#include "nvicconf.h"

extern uint8_t* Buffer_Rx1;
extern uint8_t* Buffer_Tx1;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t I2CDirection = I2C_DIRECTION_TX;
__IO uint32_t NumbOfBytes1;
__IO uint32_t NumbOfBytes2;
__IO uint8_t Address;
__IO uint8_t Tx_Idx1 = 0, Rx_Idx1 = 0;
__IO uint8_t Tx_Idx2 = 0, Rx_Idx2 = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Reads buffer of bytes  from the slave.
 * @param pBuffer: Buffer of bytes to be read from the slave.
 * @param NumByteToRead: Number of bytes to be read by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
ErrorStatus I2C_Master_BufferRead(I2C_TypeDef* I2Cx, uint8_t* pBuffer,
    uint32_t NumByteToRead, uint8_t SlaveAddress,
    uint32_t timeoutMs)

{
  __IO uint32_t temp = 0;
  __IO uint32_t Timeout = 0;

  /* Enable I2C errors interrupts (used in all modes: Polling, DMA and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;

  /* I2Cx Master Reception using Interrupts with highest priority in an application */
	
    /* Enable EVT IT*/
    I2Cx->CR2 |= I2C_IT_EVT;
    /* Enable BUF IT */
    I2Cx->CR2 |= I2C_IT_BUF;
    /* Set the I2C direction to reception */
    I2CDirection = I2C_DIRECTION_RX;
    Buffer_Rx1 = pBuffer;
    SlaveAddress |= OAR1_ADD0_Set;
    Address = SlaveAddress;
    if (I2Cx == I2C1)
      NumbOfBytes1 = NumByteToRead;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    Timeout = timeoutMs * I2CDEV_LOOPS_PER_MS;
    /* Wait until the START condition is generated on the bus: START bit is cleared by hardware */
    while ((I2Cx->CR1 & 0x100) == 0x100 && Timeout)
    {
      Timeout--;
    }
    /* Wait until BUSY flag is reset (until a STOP is generated) */
    while ((I2Cx->SR2 & 0x0002) == 0x0002 && Timeout)
    {
      Timeout--;
    }
    /* Enable Acknowledgement to be ready for another reception */
    I2Cx->CR1 |= CR1_ACK_Set;

    if (Timeout == 0)
      return ERROR;


  return SUCCESS;

//  temp++; //To avoid GCC warning!
}

/**
 * @brief  Writes buffer of bytes.
 * @param pBuffer: Buffer of bytes to be sent to the slave.
 * @param NumByteToWrite: Number of bytes to be sent by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
ErrorStatus I2C_Master_BufferWrite(I2C_TypeDef* I2Cx, uint8_t* pBuffer,
    uint32_t NumByteToWrite, uint8_t SlaveAddress,
    uint32_t timeoutMs)
{

  __IO uint32_t temp = 0;
  __IO uint32_t Timeout = 0;

  /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;
  /* I2Cx Master Transmission using Interrupt with highest priority in the application */
	
    /* Enable EVT IT*/
    I2Cx->CR2 |= I2C_IT_EVT;
    /* Enable BUF IT */
    I2Cx->CR2 |= I2C_IT_BUF;
    /* Set the I2C direction to Transmission */
    I2CDirection = I2C_DIRECTION_TX;
    Buffer_Tx1 = pBuffer;
    SlaveAddress &= OAR1_ADD0_Reset;
    Address = SlaveAddress;
    if (I2Cx == I2C1)
      NumbOfBytes1 = NumByteToWrite;
    else
      NumbOfBytes2 = NumByteToWrite;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    Timeout = timeoutMs * I2CDEV_LOOPS_PER_MS;
    /* Wait until the START condition is generated on the bus: the START bit is cleared by hardware */
    while ((I2Cx->CR1 & 0x100) == 0x100 && Timeout)
    {
      Timeout--;
    }
    /* Wait until BUSY flag is reset: a STOP has been generated on the bus signaling the end
     of transmission */
    while ((I2Cx->SR2 & 0x0002) == 0x0002 && Timeout)
    {
      Timeout--;
    }

    if (Timeout == 0)
      return ERROR;
		
  return SUCCESS;

//  temp++; //To avoid GCC warning!
}

/**
 * @brief  Initializes peripherals: I2Cx, GPIO, DMA channels .
 * @param  None
 * @retval None
 */
void I2C_LowLevel_Init(I2C_TypeDef* I2Cx)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Enable the DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  if (I2Cx == I2C1)
  {
    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Release I2C1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
		//NVIC_InitStructure.NVIC_IRQChannel = 0;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_EV_PRI;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_EV_PRI + 1;
		//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 + 1;
    NVIC_Init(&NVIC_InitStructure);
  }

  /* I2C1 and I2C2 configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = OwnAddress1;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
  I2C_Init(I2C1, &I2C_InitStructure);

}

/**
 * @brief  This function handles I2C1 Event interrupt request.
 * @param  None
 * @retval : None
 */
void i2cInterruptHandlerI2c1(void)
{

  __IO uint32_t SR1Register = 0;
  __IO uint32_t SR2Register = 0;

  /* Read the I2C1 SR1 and SR2 status registers */
  SR1Register = I2C1->SR1;
  SR2Register = I2C1->SR2;

  /* If SB = 1, I2C1 master sent a START on the bus: EV5) */
  if ((SR1Register & 0x0001) == 0x0001)
  {
    /* Send the slave address for transmssion or for reception (according to the configured value
     in the write master write routine */
    I2C1->DR = Address;
    SR1Register = 0;
    SR2Register = 0;
  }
  /* If I2C1 is Master (MSL flag = 1) */

  if ((SR2Register & 0x0001) == 0x0001)
  {
    /* If ADDR = 1, EV6 */
    if ((SR1Register & 0x0002) == 0x0002)
    {
      /* Write the first data in case the Master is Transmitter */
      if (I2CDirection == I2C_DIRECTION_TX)
      {
        /* Initialize the Transmit counter */
        Tx_Idx1 = 0;
        /* Write the first data in the data register */
				I2C1->DR = Buffer_Tx1[Tx_Idx1++];
        /* Decrement the number of bytes to be written */
        NumbOfBytes1--;
        /* If no further data to be sent, disable the I2C BUF IT
         in order to not have a TxE  interrupt */
        if (NumbOfBytes1 == 0)
        {
          I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      /* Master Receiver */
      else
      {
        /* Initialize Receive counter */
        Rx_Idx1 = 0;
        /* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
        /* EV6_1: used for single byte reception. The ACK disable and the STOP
         Programming should be done just after ADDR is cleared. */
        if (NumbOfBytes1 == 1)
        {
          /* Clear ACK */
          I2C1->CR1 &= CR1_ACK_Reset;
          /* Program the STOP */
          I2C1->CR1 |= CR1_STOP_Set;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* Master transmits the remaing data: from data2 until the last one.  */
    /* If TXE is set */
    if ((SR1Register & 0x0084) == 0x0080)
    {
      /* If there is still data to write */
      if (NumbOfBytes1 != 0)
      {
        /* Write the data in DR register */
        I2C1->DR = Buffer_Tx1[Tx_Idx1++];
        /* Decrment the number of data to be written */
        NumbOfBytes1--;
        /* If  no data remains to write, disable the BUF IT in order
         to not have again a TxE interrupt. */
        if (NumbOfBytes1 == 0)
        {
          /* Disable the BUF IT */
          I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If BTF and TXE are set (EV8_2), program the STOP */
    if ((SR1Register & 0x0084) == 0x0084)
    {
      /* Program the STOP */
      I2C1->CR1 |= CR1_STOP_Set;
      /* Disable EVT IT In order to not have again a BTF IT */
      I2C1->CR2 &= (uint16_t) ~I2C_IT_EVT;
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If RXNE is set */
    if ((SR1Register & 0x0040) == 0x0040)
    {
      /* Read the data register */
      Buffer_Rx1[Rx_Idx1++] = I2C1->DR;
      /* Decrement the number of bytes to be read */
      NumbOfBytes1--;
      /* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
      if (NumbOfBytes1 == 1)
      {
        /* Clear ACK */
        I2C1->CR1 &= CR1_ACK_Reset;
        /* Program the STOP */
        I2C1->CR1 |= CR1_STOP_Set;
      }
      SR1Register = 0;
      SR2Register = 0;
    }
  }
}

/**
 * @brief  This function handles I2C1 Error interrupt request.
 * @param  None
 * @retval : None
 */
void i2cErrorInterruptHandlerI2c1(void)
{

  __IO uint32_t SR1Register = 0;

  /* Read the I2C1 status register */
  SR1Register = I2C1->SR1;
  /* If AF = 1 */
  if ((SR1Register & 0x0400) == 0x0400)
  {
    I2C1->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If ARLO = 1 */
  if ((SR1Register & 0x0200) == 0x0200)
  {
    I2C1->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If BERR = 1 */
  if ((SR1Register & 0x0100) == 0x0100)
  {
    I2C1->SR1 &= 0xFEFF;
    SR1Register = 0;
  }
  /* If OVR = 1 */
  if ((SR1Register & 0x0800) == 0x0800)
  {
    I2C1->SR1 &= 0xF7FF;
    SR1Register = 0;
  }
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
