#ifndef __OCTOSPI_LL_H
#define __OCTOSPI_LL_H

#include <stdint.h>
#include "stm32h5xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Alamat memory-mapped OctoSPI1 di STM32H5xx
 * CPU bisa baca langsung dari sini setelah mode Memory-Mapped aktif
 * ----------------------------------------------------------------------- */
#define OCTOSPI1_MM_BASE        0x90000000UL

/* -----------------------------------------------------------------------
 * Command Macronix MX25L/MX66L (Quad SPI, 1-1-4 Fast Read)
 * ----------------------------------------------------------------------- */
#define FLASH_CMD_READ_QUAD     0xEB  /* Fast Read Quad I/O              */
#define FLASH_CMD_RDSR          0x05  /* Read Status Register            */
#define FLASH_CMD_WREN          0x06  /* Write Enable                    */
#define FLASH_CMD_EQIO          0x35  /* Enter Quad Mode (Macronix)      */

#define FLASH_DUMMY_CYCLES      6

/* Inisialisasi OctoSPI1 core & GPIO */
void OCTOSPI1_GPIO_Init(void);
void OCTOSPI1_Core_Init(void);
void OCTOSPI1_LL_IndirectInit(void);
void OCTOSPI1_LL_MemoryMapped(void);
void OCTOSPI1_LL_Init(void);

/* Helper Unlock / Lock Memory Mapped mode */
void OCTOSPI1_UnlockMemoryMapped(void);
void OCTOSPI1_LockMemoryMapped(void);
uint8_t OCTOSPI1_IsMemoryMapped(void);

void OCTOSPI1_LL_SendCmd(uint8_t cmd);
void OCTOSPI1_LL_EnableQuadMode(void);

void ExtFlash_WaitBusy(void);
void ExtFlash_WriteEnable(void);
void ExtFlash_EraseSector(uint32_t addr);
void ExtFlash_WritePage(uint32_t addr, const uint8_t *data, uint16_t len);
void Flash_ReadBytes(uint32_t addr, uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOSPI_LL_H */

