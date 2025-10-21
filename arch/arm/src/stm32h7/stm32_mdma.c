/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mdma.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "barriers.h"

#include "stm32_dma.h"
#include "stm32_mdma.h"
#include "stm32_rcc.h"

#ifdef CONFIG_STM32H7_MDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MDMA_CHANNEL_NREGS          14
#define MDMA_CCR_RESET              0x00000000
#define MDMA_CTCR_RESET             0x00000000
#define MDMA_CBNDTR_RESET           0x00000000
#define MDMA_CSAR_RESET             0x00000000
#define MDMA_CDAR_RESET             0x00000000
#define MDMA_CBRUR_RESET            0x00000000
#define MDMA_CLAR_RESET             0x00000000
#define MDMA_CTBR_RESET             0x00000000
#define MDMA_CMAR_RESET             0x00000000
#define MDMA_CMDR_RESET             0x00000000

/* MDMA Channel Configuration Register */

#define MDMA_CCR_EN                 (1 << 0)  /* Bit 0: Channel enable */
#define MDMA_CCR_TEIE               (1 << 1)  /* Bit 1: Transfer error interrupt enable */
#define MDMA_CCR_CTCIE              (1 << 2)  /* Bit 2: Channel transfer complete interrupt enable */
#define MDMA_CCR_BRTIE              (1 << 3)  /* Bit 3: Block repeat transfer interrupt enable */
#define MDMA_CCR_BTIE               (1 << 4)  /* Bit 4: Block transfer interrupt enable */
#define MDMA_CCR_TCIE               (1 << 5)  /* Bit 5: Buffer transfer complete interrupt enable */
#define MDMA_CCR_PL_SHIFT           (6)       /* Bits 7:6 Priority level */
#define MDMA_CCR_PL_MASK            (3 << MDMA_CCR_PL_SHIFT)
#define MDMA_CCR_PL_LOW             (0 << MDMA_CCR_PL_SHIFT)
#define MDMA_CCR_PL_MEDIUM          (1 << MDMA_CCR_PL_SHIFT)
#define MDMA_CCR_PL_HIGH            (2 << MDMA_CCR_PL_SHIFT)
#define MDMA_CCR_PL_VERY_HIGH       (3 << MDMA_CCR_PL_SHIFT)
#define MDMA_CCR_BEX                (1 << 12) /* Bit 12: Byte endianness exchange */
#define MDMA_CCR_HEX                (1 << 13) /* Bit 13: Half word endianness exchange */
#define MDMA_CCR_WEX                (1 << 14) /* Bit 14: Word endianness exchange */
#define MDMA_CCR_SWRQ               (1 << 16) /* Bit 16: Software request */

/* MDMA Channel Transfer Configuration Register */

#define MDMA_CTCR_SINC_SHIFT        (0)       /* Bits 1:0 Source increment mode */
#define MDMA_CTCR_SINC_MASK         (3 << MDMA_CTCR_SINC_SHIFT)
#define MDMA_CTCR_SINC_FIXED        (0 << MDMA_CTCR_SINC_SHIFT)
#define MDMA_CTCR_SINC_INC          (2 << MDMA_CTCR_SINC_SHIFT)
#define MDMA_CTCR_SINC_DEC          (3 << MDMA_CTCR_SINC_SHIFT)

#define MDMA_CTCR_DINC_SHIFT        (2)       /* Bits 3:2 Destination increment mode */
#define MDMA_CTCR_DINC_MASK         (3 << MDMA_CTCR_DINC_SHIFT)
#define MDMA_CTCR_DINC_FIXED        (0 << MDMA_CTCR_DINC_SHIFT)
#define MDMA_CTCR_DINC_INC          (2 << MDMA_CTCR_DINC_SHIFT)
#define MDMA_CTCR_DINC_DEC          (3 << MDMA_CTCR_DINC_SHIFT)

#define MDMA_CTCR_SSIZE_SHIFT       (4)       /* Bits 5:4 Source data size */
#define MDMA_CTCR_SSIZE_MASK        (3 << MDMA_CTCR_SSIZE_SHIFT)
#define MDMA_CTCR_SSIZE_BYTE        (0 << MDMA_CTCR_SSIZE_SHIFT)
#define MDMA_CTCR_SSIZE_HWORD       (1 << MDMA_CTCR_SSIZE_SHIFT)
#define MDMA_CTCR_SSIZE_WORD        (2 << MDMA_CTCR_SSIZE_SHIFT)

#define MDMA_CTCR_DSIZE_SHIFT       (6)       /* Bits 7:6 Destination data size */
#define MDMA_CTCR_DSIZE_MASK        (3 << MDMA_CTCR_DSIZE_SHIFT)
#define MDMA_CTCR_DSIZE_BYTE        (0 << MDMA_CTCR_DSIZE_SHIFT)
#define MDMA_CTCR_DSIZE_HWORD       (1 << MDMA_CTCR_DSIZE_SHIFT)
#define MDMA_CTCR_DSIZE_WORD        (2 << MDMA_CTCR_DSIZE_SHIFT)

#define MDMA_CTCR_SINCOS_SHIFT      (8)       /* Bits 9:8 Source increment offset size */
#define MDMA_CTCR_SINCOS_MASK       (3 << MDMA_CTCR_SINCOS_SHIFT)

#define MDMA_CTCR_DINCOS_SHIFT      (10)      /* Bits 11:10 Destination increment offset size */
#define MDMA_CTCR_DINCOS_MASK       (3 << MDMA_CTCR_DINCOS_SHIFT)

#define MDMA_CTCR_SBURST_SHIFT      (12)      /* Bits 14:12 Source burst transfer configuration */
#define MDMA_CTCR_SBURST_MASK       (7 << MDMA_CTCR_SBURST_SHIFT)

#define MDMA_CTCR_DBURST_SHIFT      (15)      /* Bits 17:15 Destination burst transfer configuration */
#define MDMA_CTCR_DBURST_MASK       (7 << MDMA_CTCR_DBURST_SHIFT)

#define MDMA_CTCR_TLEN_SHIFT        (18)      /* Bits 24:18 Buffer transfer length */
#define MDMA_CTCR_TLEN_MASK         (0x7f << MDMA_CTCR_TLEN_SHIFT)

#define MDMA_CTCR_PKE               (1 << 25) /* Bit 25: Pack enable */
#define MDMA_CTCR_PAM_SHIFT         (26)      /* Bits 27:26 Padding/alignment mode */
#define MDMA_CTCR_PAM_MASK          (3 << MDMA_CTCR_PAM_SHIFT)

#define MDMA_CTCR_TRGM_SHIFT        (28)      /* Bits 29:28 Trigger mode */
#define MDMA_CTCR_TRGM_MASK         (3 << MDMA_CTCR_TRGM_SHIFT)
#define MDMA_CTCR_TRGM_BUFFER       (0 << MDMA_CTCR_TRGM_SHIFT)
#define MDMA_CTCR_TRGM_BLOCK        (1 << MDMA_CTCR_TRGM_SHIFT)
#define MDMA_CTCR_TRGM_REPEAT       (2 << MDMA_CTCR_TRGM_SHIFT)
#define MDMA_CTCR_TRGM_FULL         (3 << MDMA_CTCR_TRGM_SHIFT)

#define MDMA_CTCR_SWRM              (1 << 30) /* Bit 30: Software request mode */
#define MDMA_CTCR_BWM               (1 << 31) /* Bit 31: Bufferable write mode */

/* MDMA request mapping for QSPI */

#define MDMA_REQUEST_QUADSPI_FIFO_TH    0x16  /* QSPI FIFO threshold */
#define MDMA_REQUEST_QUADSPI_TC         0x17  /* QSPI Transfer complete */

/* MDMA Channel IRQ numbers (these should be defined in chip.h but we define them here) */
#ifndef STM32_IRQ_MDMA_CH0
#define STM32_IRQ_MDMA_CH0              (STM32_IRQ_FIRST + 122)  /* MDMA Channel 0 global interrupt */
#define STM32_IRQ_MDMA_CH1              (STM32_IRQ_FIRST + 123)  /* MDMA Channel 1 global interrupt */
#define STM32_IRQ_MDMA_CH2              (STM32_IRQ_FIRST + 124)  /* MDMA Channel 2 global interrupt */
#define STM32_IRQ_MDMA_CH3              (STM32_IRQ_FIRST + 125)  /* MDMA Channel 3 global interrupt */
#define STM32_IRQ_MDMA_CH4              (STM32_IRQ_FIRST + 126)  /* MDMA Channel 4 global interrupt */
#define STM32_IRQ_MDMA_CH5              (STM32_IRQ_FIRST + 127)  /* MDMA Channel 5 global interrupt */
#define STM32_IRQ_MDMA_CH6              (STM32_IRQ_FIRST + 128)  /* MDMA Channel 6 global interrupt */
#define STM32_IRQ_MDMA_CH7              (STM32_IRQ_FIRST + 129)  /* MDMA Channel 7 global interrupt */
#define STM32_IRQ_MDMA_CH8              (STM32_IRQ_FIRST + 130)  /* MDMA Channel 8 global interrupt */
#define STM32_IRQ_MDMA_CH9              (STM32_IRQ_FIRST + 131)  /* MDMA Channel 9 global interrupt */
#define STM32_IRQ_MDMA_CH10             (STM32_IRQ_FIRST + 132)  /* MDMA Channel 10 global interrupt */
#define STM32_IRQ_MDMA_CH11             (STM32_IRQ_FIRST + 133)  /* MDMA Channel 11 global interrupt */
#define STM32_IRQ_MDMA_CH12             (STM32_IRQ_FIRST + 134)  /* MDMA Channel 12 global interrupt */
#define STM32_IRQ_MDMA_CH13             (STM32_IRQ_FIRST + 135)  /* MDMA Channel 13 global interrupt */
#define STM32_IRQ_MDMA_CH14             (STM32_IRQ_FIRST + 136)  /* MDMA Channel 14 global interrupt */
#define STM32_IRQ_MDMA_CH15             (STM32_IRQ_FIRST + 137)  /* MDMA Channel 15 global interrupt */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one MDMA channel */

struct stm32_mdma_s
{
  uint8_t          chan;     /* MDMA channel number (0-15) */
  uint8_t          irq;      /* MDMA channel IRQ number */
  uint8_t          shift;    /* ISR/IFCR bit shift value */
  uintptr_t        base;     /* MDMA channel register base address */
  dma_callback_t   callback; /* Callback invoked when the MDMA completes */
  void            *arg;      /* Argument passed to callback function */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the 16 MDMA channels available */

static struct stm32_mdma_s g_mdma[16] =
{
  /* MDMA channels 0-15 */
  {
    .chan     = 0,
    .irq      = STM32_IRQ_MDMA_CH0,
    .shift    = 0,
    .base     = STM32_MDMA_CH0_BASE,
  },
  {
    .chan     = 1,
    .irq      = STM32_IRQ_MDMA_CH1,
    .shift    = 4,
    .base     = STM32_MDMA_CH1_BASE,
  },
  /* Add remaining channels as needed... */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdma_interrupt
 *
 * Description:
 *   MDMA interrupt handler
 *
 ****************************************************************************/

static int stm32_mdma_interrupt(int irq, void *context, void *arg)
{
  struct stm32_mdma_s *mdma = (struct stm32_mdma_s *)arg;
  uint32_t isr;
  uint32_t mask;

  /* Get the interrupt status for this channel */

  isr = getreg32(STM32_MDMA_GISR0);
  mask = (0x0f << mdma->shift);

  if ((isr & mask) != 0)
    {
      /* Clear the interrupts we understand */

      putreg32(mask, STM32_MDMA_GIFCR0);

      /* Invoke the callback */

      if (mdma->callback)
        {
          mdma->callback((DMA_HANDLE)mdma, isr & mask, mdma->arg);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdma_initialize
 *
 * Description:
 *   Initialize the MDMA subsystem
 *
 ****************************************************************************/

void weak_function stm32_mdma_initialize(void)
{
  uint32_t regval;

  /* Enable MDMA clock */

  regval = getreg32(STM32_RCC_AHB3ENR);
  regval |= RCC_AHB3ENR_MDMAEN;
  putreg32(regval, STM32_RCC_AHB3ENR);

  /* Reset MDMA */

  regval = getreg32(STM32_RCC_AHB3RSTR);
  regval |= RCC_AHB3RSTR_MDMARST;
  putreg32(regval, STM32_RCC_AHB3RSTR);
  regval &= ~RCC_AHB3RSTR_MDMARST;
  putreg32(regval, STM32_RCC_AHB3RSTR);
}

/****************************************************************************
 * Name: stm32_mdma_alloc
 *
 * Description:
 *   Allocate an MDMA channel.
 *
 ****************************************************************************/

DMA_HANDLE stm32_mdma_alloc(uint32_t chanreq)
{
  struct stm32_mdma_s *mdma = NULL;
  unsigned int chan = chanreq & MDMACHAN_MASK;
  int ret;

  /* Check if the channel is available */

  if (chan < 16)
    {
      mdma = &g_mdma[chan];

      /* Attach MDMA interrupt handler */

      ret = irq_attach(mdma->irq, stm32_mdma_interrupt, mdma);
      if (ret == OK)
        {
          up_enable_irq(mdma->irq);
        }
      else
        {
          mdma = NULL;
        }
    }

  return (DMA_HANDLE)mdma;
}

/****************************************************************************
 * Name: stm32_mdma_free
 *
 * Description:
 *   Release an MDMA channel.
 *
 ****************************************************************************/

void stm32_mdma_free(DMA_HANDLE handle)
{
  struct stm32_mdma_s *mdma = (struct stm32_mdma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Disable the MDMA channel */

  putreg32(0, mdma->base + STM32_MDMA_CCR_OFFSET);

  /* Disable the IRQ */

  up_disable_irq(mdma->irq);

  /* Detach the interrupt handler */

  irq_detach(mdma->irq);
}

/****************************************************************************
 * Name: stm32_mdma_setup
 *
 * Description:
 *   Configure MDMA for one transfer.
 *
 ****************************************************************************/

int stm32_mdma_setup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                     size_t ntransfers, uint32_t ccr)
{
  struct stm32_mdma_s *mdma = (struct stm32_mdma_s *)handle;
  uint32_t ctcr;

  DEBUGASSERT(handle != NULL);

  /* Disable the MDMA channel */

  putreg32(0, mdma->base + STM32_MDMA_CCR_OFFSET);

  /* Set the source and destination addresses */

  putreg32(paddr, mdma->base + STM32_MDMA_CSAR_OFFSET);
  putreg32(maddr, mdma->base + STM32_MDMA_CDAR_OFFSET);

  /* Set the transfer count */

  putreg32((ntransfers - 1) & 0xffff, mdma->base + STM32_MDMA_CBNDTR_OFFSET);

  /* Set the transfer configuration */

  ctcr = MDMA_CTCR_SINC_FIXED | MDMA_CTCR_DINC_INC |
         MDMA_CTCR_SSIZE_BYTE | MDMA_CTCR_DSIZE_BYTE |
         MDMA_CTCR_TRGM_BUFFER | ((MDMA_REQUEST_QUADSPI_FIFO_TH) << 0);

  putreg32(ctcr, mdma->base + STM32_MDMA_CTCR_OFFSET);

  /* Configure the MDMA channel control register */

  putreg32(ccr, mdma->base + STM32_MDMA_CCR_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: stm32_mdma_start
 *
 * Description:
 *   Start the MDMA transfer
 *
 ****************************************************************************/

int stm32_mdma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                     bool half)
{
  struct stm32_mdma_s *mdma = (struct stm32_mdma_s *)handle;
  uint32_t ccr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info */

  mdma->callback = callback;
  mdma->arg      = arg;

  /* Enable the channel */

  ccr = getreg32(mdma->base + STM32_MDMA_CCR_OFFSET);
  ccr |= MDMA_CCR_EN;
  putreg32(ccr, mdma->base + STM32_MDMA_CCR_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: stm32_mdma_stop
 *
 * Description:
 *   Cancel the MDMA.
 *
 ****************************************************************************/

void stm32_mdma_stop(DMA_HANDLE handle)
{
  struct stm32_mdma_s *mdma = (struct stm32_mdma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Disable the channel */

  putreg32(0, mdma->base + STM32_MDMA_CCR_OFFSET);
}

/****************************************************************************
 * Name: stm32_mdma_residual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 ****************************************************************************/

size_t stm32_mdma_residual(DMA_HANDLE handle)
{
  struct stm32_mdma_s *mdma = (struct stm32_mdma_s *)handle;
  uint32_t residual;

  DEBUGASSERT(handle != NULL);

  /* Fetch the count of bytes remaining to be transferred */

  residual = getreg32(mdma->base + STM32_MDMA_CBNDTR_OFFSET);
  return (residual & 0xffff) + 1;
}

/****************************************************************************
 * Name: stm32_mdma_capable
 *
 * Description:
 *   Check if the MDMA channel is capable of the requested transfer
 *
 ****************************************************************************/

bool stm32_mdma_capable(uint32_t maddr, uint32_t count, uint32_t ccr)
{
  uint32_t transfer_size;
  uint32_t maddr_modulo;

  /* Decode the transfer size */

  transfer_size = 1 << ((ccr & DMA_SCR_MSIZE_MASK) >> DMA_SCR_MSIZE_SHIFT);

  /* Check modulo alignment */

  maddr_modulo = maddr & (transfer_size - 1);

  /* Memory address must be aligned to transfer size */

  if (maddr_modulo != 0)
    {
      return false;
    }

  return true;
}

#endif /* CONFIG_STM32H7_MDMA */