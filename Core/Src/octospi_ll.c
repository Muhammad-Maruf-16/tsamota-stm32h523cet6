/**
  ******************************************************************************
  * @file           : octospi_ll.c
  * @brief          : OctoSPI LL Driver untuk Macronix MX25LM (STM32H523CET6)
  *                   - Mendukung XIP Memory-Mapped mode (0x90000000)
  *                   - Unlock ke Indirect mode untuk Erase/Write
  *                   - Lock kembali ke Memory-Mapped mode
  ******************************************************************************
  */

#include "octospi_ll.h"
#include <string.h>

/* =========================================================================
 * 1. Inisialisasi GPIO untuk OctoSPI1
 * ========================================================================= */
void OCTOSPI1_GPIO_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    __DSB();

    /* PA1 -> OCTOSPI1_IO3 (AF9) */
    GPIOA->MODER   = (GPIOA->MODER   & ~GPIO_MODER_MODE1_Msk)   | (2U << GPIO_MODER_MODE1_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED1_Msk) | (3U << GPIO_OSPEEDR_OSPEED1_Pos);
    GPIOA->PUPDR   = (GPIOA->PUPDR   & ~GPIO_PUPDR_PUPD1_Msk);
    GPIOA->AFR[0]  = (GPIOA->AFR[0]  & ~(0xFU << 4))            | (9U << 4);

    /* PA3 -> OCTOSPI1_CLK (AF3) */
    GPIOA->MODER   = (GPIOA->MODER   & ~GPIO_MODER_MODE3_Msk)   | (2U << GPIO_MODER_MODE3_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED3_Msk) | (3U << GPIO_OSPEEDR_OSPEED3_Pos);
    GPIOA->PUPDR   = (GPIOA->PUPDR   & ~GPIO_PUPDR_PUPD3_Msk);
    GPIOA->AFR[0]  = (GPIOA->AFR[0]  & ~(0xFU << 12))           | (3U << 12);

    /* PA7 -> OCTOSPI1_IO2 (AF10) */
    GPIOA->MODER   = (GPIOA->MODER   & ~GPIO_MODER_MODE7_Msk)   | (2U << GPIO_MODER_MODE7_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED7_Msk) | (3U << GPIO_OSPEEDR_OSPEED7_Pos);
    GPIOA->PUPDR   = (GPIOA->PUPDR   & ~GPIO_PUPDR_PUPD7_Msk);
    GPIOA->AFR[0]  = (GPIOA->AFR[0]  & ~(0xFU << 28))           | (10U << 28);

    /* PB0 -> OCTOSPI1_IO1 (AF6) */
    GPIOB->MODER   = (GPIOB->MODER   & ~GPIO_MODER_MODE0_Msk)   | (2U << GPIO_MODER_MODE0_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0;
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED0_Msk) | (3U << GPIO_OSPEEDR_OSPEED0_Pos);
    GPIOB->PUPDR   = (GPIOB->PUPDR   & ~GPIO_PUPDR_PUPD0_Msk);
    GPIOB->AFR[0]  = (GPIOB->AFR[0]  & ~(0xFU << 0))            | (6U << 0);

    /* PB1 -> OCTOSPI1_IO0 (AF6) */
    GPIOB->MODER   = (GPIOB->MODER   & ~GPIO_MODER_MODE1_Msk)   | (2U << GPIO_MODER_MODE1_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED1_Msk) | (3U << GPIO_OSPEEDR_OSPEED1_Pos);
    GPIOB->PUPDR   = (GPIOB->PUPDR   & ~GPIO_PUPDR_PUPD1_Msk);
    GPIOB->AFR[0]  = (GPIOB->AFR[0]  & ~(0xFU << 4))            | (6U << 4);

    /* PB10 -> OCTOSPI1_NCS (AF9) */
    GPIOB->MODER   = (GPIOB->MODER   & ~GPIO_MODER_MODE10_Msk)  | (2U << GPIO_MODER_MODE10_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED10_Msk)| (3U << GPIO_OSPEEDR_OSPEED10_Pos);
    GPIOB->PUPDR   = (GPIOB->PUPDR   & ~GPIO_PUPDR_PUPD10_Msk);
    GPIOB->AFR[1]  = (GPIOB->AFR[1]  & ~(0xFU << 8))            | (9U << 8);
}

/* =========================================================================
 * 2. Inisialisasi register OctoSPI1
 * ========================================================================= */
void OCTOSPI1_Core_Init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_OCTOSPI1EN;
    __DSB();

    OCTOSPI1->CR &= ~OCTOSPI_CR_EN;
    while (OCTOSPI1->CR & OCTOSPI_CR_EN);

    OCTOSPI1->DCR1 = (0U  << OCTOSPI_DCR1_MTYP_Pos)
                   | (24U << OCTOSPI_DCR1_DEVSIZE_Pos)  /* 32 MB */
                   | (1U  << OCTOSPI_DCR1_CSHT_Pos)     /* CS High 2 cycles */
                   | (0U  << OCTOSPI_DCR1_CKMODE_Pos);  /* Mode 0 */

    OCTOSPI1->DCR2 = (1U  << OCTOSPI_DCR2_PRESCALER_Pos);
    OCTOSPI1->DCR3 = 0U;
    OCTOSPI1->DCR4 = 0U;

    OCTOSPI1->CR = (0U << OCTOSPI_CR_FMODE_Pos)
                 | (0U << OCTOSPI_CR_FTHRES_Pos)
                 | (0U << OCTOSPI_CR_DMM_Pos)
                 | (0U << OCTOSPI_CR_MSEL_Pos);

    OCTOSPI1->CR |= OCTOSPI_CR_EN;
    while (!(OCTOSPI1->CR & OCTOSPI_CR_EN));
}

void OCTOSPI1_LL_SendCmd(uint8_t cmd)
{
    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = 0U;
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos);
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR  = cmd;
    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
}

void OCTOSPI1_LL_EnableQuadMode(void)
{
    OCTOSPI1_LL_SendCmd(FLASH_CMD_EQIO);
    for (volatile uint32_t i = 0; i < 1000; i++);
}

/* =========================================================================
 * 3. RAM Functions for Mode Switch & Write/Erase
 * ========================================================================= */
__attribute__((noinline, section(".RamFunc")))
void OCTOSPI1_LL_IndirectInit_RAM(void)
{
    OCTOSPI1->CR &= ~OCTOSPI_CR_EN;
    while (OCTOSPI1->CR & OCTOSPI_CR_EN);

    OCTOSPI1->DCR1 = (24U << OCTOSPI_DCR1_DEVSIZE_Pos)
                   | (1U  << OCTOSPI_DCR1_CSHT_Pos);

    OCTOSPI1->DCR2 = (1U  << OCTOSPI_DCR2_PRESCALER_Pos);

    OCTOSPI1->TCR  = (0U  << OCTOSPI_TCR_DCYC_Pos)
                   | (0U  << OCTOSPI_TCR_SSHIFT_Pos);

    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->CR |= OCTOSPI_CR_EN;
    while (!(OCTOSPI1->CR & OCTOSPI_CR_EN));
}

__attribute__((noinline, section(".RamFunc")))
void OCTOSPI1_LL_MemoryMapped_RAM(void)
{
    OCTOSPI1->CR &= ~OCTOSPI_CR_EN;
    while (OCTOSPI1->CR & OCTOSPI_CR_EN);

    OCTOSPI1->DCR1 = (24U << OCTOSPI_DCR1_DEVSIZE_Pos)
                   | (1U  << OCTOSPI_DCR1_CSHT_Pos);

    OCTOSPI1->DCR2 = (1U  << OCTOSPI_DCR2_PRESCALER_Pos);

    OCTOSPI1->TCR  = (0U  << OCTOSPI_TCR_DCYC_Pos)
                   | (0U  << OCTOSPI_TCR_SSHIFT_Pos);

    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos)
                  | (1U << OCTOSPI_CCR_DMODE_Pos);

    OCTOSPI1->IR = 0x03U;

    OCTOSPI1->CR = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk)
                 | (3U << OCTOSPI_CR_FMODE_Pos);

    OCTOSPI1->CR |= OCTOSPI_CR_EN;
    while (!(OCTOSPI1->CR & OCTOSPI_CR_EN));
}

uint8_t OCTOSPI1_IsMemoryMapped(void)
{
    return (((OCTOSPI1->CR & OCTOSPI_CR_FMODE_Msk) == (3U << OCTOSPI_CR_FMODE_Pos)) &&
            (OCTOSPI1->CR & OCTOSPI_CR_EN)) ? 1U : 0U;
}

void OCTOSPI1_UnlockMemoryMapped(void)
{
    OCTOSPI1_LL_IndirectInit_RAM();
}

void OCTOSPI1_LockMemoryMapped(void)
{
    OCTOSPI1_LL_MemoryMapped_RAM();
}

__attribute__((noinline, section(".RamFunc")))
static void ExtFlash_WaitBusy_RAM(void)
{
    uint8_t status;
    do {
        while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
        OCTOSPI1->DLR = 0U;
        OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                      | (1U << OCTOSPI_CCR_DMODE_Pos);
        OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk)
                      | (1U << OCTOSPI_CR_FMODE_Pos);
        OCTOSPI1->IR  = 0x05U;

        while (!(OCTOSPI1->SR & OCTOSPI_SR_FTF) &&
               !(OCTOSPI1->SR & OCTOSPI_SR_TCF));
        status = *(__IO uint8_t *)(&OCTOSPI1->DR);
        while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
        OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
        OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    } while (status & 0x01U);
}

__attribute__((noinline, section(".RamFunc")))
static void ExtFlash_WriteEnable_RAM(void)
{
    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = 0U;
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos);
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR  = 0x06U;

    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
}

void ExtFlash_WaitBusy(void)
{
    ExtFlash_WaitBusy_RAM();
}

void ExtFlash_WriteEnable(void)
{
    ExtFlash_WriteEnable_RAM();
}

__attribute__((noinline, section(".RamFunc")))
void ExtFlash_EraseSector(uint32_t addr)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t was_mm = OCTOSPI1_IsMemoryMapped();
    if (was_mm) {
        OCTOSPI1_LL_IndirectInit_RAM();
    }

    ExtFlash_WaitBusy_RAM();
    ExtFlash_WriteEnable_RAM();

    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = 0U;
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos);
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR  = 0x20U;
    OCTOSPI1->AR  = addr;

    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
    ExtFlash_WaitBusy_RAM();

    if (was_mm) {
        OCTOSPI1_LL_MemoryMapped_RAM();
    }

    if (!primask) {
        __enable_irq();
    }
}

__attribute__((noinline, section(".RamFunc")))
void ExtFlash_WritePage(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > 256) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t was_mm = OCTOSPI1_IsMemoryMapped();
    if (was_mm) {
        OCTOSPI1_LL_IndirectInit_RAM();
    }

    ExtFlash_WaitBusy_RAM();
    ExtFlash_WriteEnable_RAM();

    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY);
    OCTOSPI1->DLR = len - 1U;
    OCTOSPI1->CCR = (1U << OCTOSPI_CCR_IMODE_Pos)
                  | (1U << OCTOSPI_CCR_ADMODE_Pos)
                  | (2U << OCTOSPI_CCR_ADSIZE_Pos)
                  | (1U << OCTOSPI_CCR_DMODE_Pos);
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE_Msk);
    OCTOSPI1->IR  = 0x02U;
    OCTOSPI1->AR  = addr;

    for (uint16_t i = 0; i < len; i++)
    {
        while (!(OCTOSPI1->SR & OCTOSPI_SR_FTF) &&
               !(OCTOSPI1->SR & OCTOSPI_SR_TCF));
        *(__IO uint8_t *)(&OCTOSPI1->DR) = data[i];
    }
    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF));
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
    ExtFlash_WaitBusy_RAM();

    if (was_mm) {
        OCTOSPI1_LL_MemoryMapped_RAM();
    }

    if (!primask) {
        __enable_irq();
    }
}

/* Membaca langsung via CPU memory map (sangat cepat & aman di XIP) */
void Flash_ReadBytes(uint32_t flash_offset, uint8_t *buf, uint32_t len)
{
    if (len == 0 || buf == NULL) return;
    if (!OCTOSPI1_IsMemoryMapped()) {
        OCTOSPI1_LL_MemoryMapped_RAM();
    }
    memcpy(buf, (const void *)(OCTOSPI1_MM_BASE + flash_offset), len);
}

void OCTOSPI1_LL_IndirectInit(void)
{
    OCTOSPI1_GPIO_Init();
    OCTOSPI1_Core_Init();
    OCTOSPI1_LL_IndirectInit_RAM();
    OCTOSPI1_LL_EnableQuadMode();
}

void OCTOSPI1_LL_MemoryMapped(void)
{
    OCTOSPI1_GPIO_Init();
    OCTOSPI1_Core_Init();
    OCTOSPI1_LL_MemoryMapped_RAM();
}

void OCTOSPI1_LL_Init(void)
{
    OCTOSPI1_LL_IndirectInit();
    OCTOSPI1_LL_MemoryMapped();
}
