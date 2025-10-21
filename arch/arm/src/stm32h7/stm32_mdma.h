/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mdma.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_MDMA_H
#define __ARCH_ARM_SRC_STM32H7_STM32_MDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"
#include "stm32_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MDMA Channel Selection ***************************************************
 *
 * Each MDMA channel may be assigned to support different hardware.  Because
 * of this, the MDMA channel must be specified in the board.h header file.
 */

#define MDMACHAN_SHIFT        (0)      /* Bits 0-3: MDMA channel number */
#define MDMACHAN_MASK         (15 << MDMACHAN_SHIFT)
#  define MDMACHAN(n)         ((n) << MDMACHAN_SHIFT) /* Channel n, n=0..15 */

/* MDMA Channel Addresses ***************************************************/

#define STM32_MDMA_BASE         0x52000000 /* 0x52000000 - 0x520003ff: MDMA controller */

#define STM32_MDMA_GISR0        (STM32_MDMA_BASE + 0x0000) /* MDMA global interrupt/status register 0 */
#define STM32_MDMA_GIFCR0       (STM32_MDMA_BASE + 0x0004) /* MDMA global interrupt flag clear register 0 */

/* MDMA Channel Offsets *****************************************************/

#define STM32_MDMA_CCR_OFFSET   0x0040 /* MDMA channel configuration register */
#define STM32_MDMA_CTCR_OFFSET  0x0044 /* MDMA channel transfer configuration register */
#define STM32_MDMA_CBNDTR_OFFSET 0x0048 /* MDMA channel block number of data register */
#define STM32_MDMA_CSAR_OFFSET  0x004c /* MDMA channel source address register */
#define STM32_MDMA_CDAR_OFFSET  0x0050 /* MDMA channel destination address register */
#define STM32_MDMA_CBRUR_OFFSET 0x0054 /* MDMA channel block repeat address update register */
#define STM32_MDMA_CLAR_OFFSET  0x0058 /* MDMA channel link address register */
#define STM32_MDMA_CTBR_OFFSET  0x005c /* MDMA channel trigger and bus selection register */
#define STM32_MDMA_CMAR_OFFSET  0x0064 /* MDMA channel mask address register */
#define STM32_MDMA_CMDR_OFFSET  0x0068 /* MDMA channel mask data register */

/* MDMA Channel Base Addresses **********************************************/

#define STM32_MDMA_CHAN_OFFSET  0x0040 /* Offset between MDMA channel base addresses */
#define STM32_MDMA_FIRST        0x0000 /* Offset of first MDMA channel */

#define STM32_MDMA_CH0_BASE     (STM32_MDMA_BASE + 0x0040)
#define STM32_MDMA_CH1_BASE     (STM32_MDMA_BASE + 0x0080)
#define STM32_MDMA_CH2_BASE     (STM32_MDMA_BASE + 0x00c0)
#define STM32_MDMA_CH3_BASE     (STM32_MDMA_BASE + 0x0100)
#define STM32_MDMA_CH4_BASE     (STM32_MDMA_BASE + 0x0140)
#define STM32_MDMA_CH5_BASE     (STM32_MDMA_BASE + 0x0180)
#define STM32_MDMA_CH6_BASE     (STM32_MDMA_BASE + 0x01c0)
#define STM32_MDMA_CH7_BASE     (STM32_MDMA_BASE + 0x0200)
#define STM32_MDMA_CH8_BASE     (STM32_MDMA_BASE + 0x0240)
#define STM32_MDMA_CH9_BASE     (STM32_MDMA_BASE + 0x0280)
#define STM32_MDMA_CH10_BASE    (STM32_MDMA_BASE + 0x02c0)
#define STM32_MDMA_CH11_BASE    (STM32_MDMA_BASE + 0x0300)
#define STM32_MDMA_CH12_BASE    (STM32_MDMA_BASE + 0x0340)
#define STM32_MDMA_CH13_BASE    (STM32_MDMA_BASE + 0x0380)
#define STM32_MDMA_CH14_BASE    (STM32_MDMA_BASE + 0x03c0)
#define STM32_MDMA_CH15_BASE    (STM32_MDMA_BASE + 0x0400)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_mdma_initialize
 *
 * Description:
 *   Initialize the MDMA controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_mdma_initialize(void);

/****************************************************************************
 * Name: stm32_mdma_alloc
 *
 * Description:
 *   Allocate an MDMA channel.
 *
 * Input Parameters:
 *   chanreq - Identifies the channel resource. Channel are numbered per
 *             the MDMA channel configuration.
 *
 * Returned Value:
 *   On success, a non-NULL MDMA channel handle is returned.
 *
 ****************************************************************************/

DMA_HANDLE stm32_mdma_alloc(uint32_t chanreq);

/****************************************************************************
 * Name: stm32_mdma_free
 *
 * Description:
 *   Release an MDMA channel.
 *
 * Input Parameters:
 *   handle - The MDMA handle returned by stm32_mdma_alloc()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_mdma_free(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_mdma_setup
 *
 * Description:
 *   Configure MDMA for one transfer.
 *
 * Input Parameters:
 *   handle      - The MDMA handle returned by stm32_mdma_alloc()
 *   paddr       - Peripheral address
 *   maddr       - Memory address
 *   ntransfers  - Number of transfers to perform
 *   ccr         - Channel configuration register value
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int stm32_mdma_setup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                     size_t ntransfers, uint32_t ccr);

/****************************************************************************
 * Name: stm32_mdma_start
 *
 * Description:
 *   Start the MDMA transfer
 *
 * Input Parameters:
 *   handle   - The MDMA handle returned by stm32_mdma_alloc()
 *   callback - The function to be called when the MDMA completes or is
 *              aborted.
 *   arg      - An argument that will be passed to the callback function
 *              when it is called.
 *   half     - TRUE: Enable half-complete interrupt; FALSE: Disable
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int stm32_mdma_start(DMA_HANDLE handle, dma_callback_t callback,
                     void *arg, bool half);

/****************************************************************************
 * Name: stm32_mdma_stop
 *
 * Description:
 *   Cancel the MDMA.
 *
 * Input Parameters:
 *   handle - The MDMA handle returned by stm32_mdma_alloc()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_mdma_stop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_mdma_residual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Input Parameters:
 *   handle - The MDMA handle returned by stm32_mdma_alloc()
 *
 * Returned Value:
 *   Number of bytes remaining to be transferred
 *
 ****************************************************************************/

size_t stm32_mdma_residual(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_mdma_capable
 *
 * Description:
 *   Check if the MDMA channel is capable of the requested transfer
 *
 * Input Parameters:
 *   maddr - Memory address
 *   count - Transfer count
 *   ccr   - Channel configuration
 *
 * Returned Value:
 *   True if capable; false if not
 *
 ****************************************************************************/

bool stm32_mdma_capable(uint32_t maddr, uint32_t count, uint32_t ccr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_MDMA_H */