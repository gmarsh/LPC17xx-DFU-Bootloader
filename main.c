/*****************************************************************************
 *                                                                            *
 * DFU/SD/SDHC Bootloader for LPC17xx                                         *
 *                                                                            *
 * by Triffid Hunter                                                          *
 *                                                                            *
 *                                                                            *
 * This firmware is Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the GNU General Public License as published by       *
 * the Free Software Foundation; either version 2 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * GNU General Public License for more details.                               *
 *                                                                            *
 * You should have received a copy of the GNU General Public License          *
 * along with this program; if not, write to the Free Software                *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA *
 *                                                                            *
 *****************************************************************************/

#include "SDCard.h"
#include "gpio.h"
#include "sbl_iap.h"
#include "sbl_config.h"
#include "ff.h"
#include "min-printf.h"
#include "lpc17xx_wdt.h"

#ifndef DEBUG_MESSAGES
#define printf(...) do {} while (0)
#endif

FATFS	fat;
FIL		file;

const char *firmware_file = "firmware.bin";
const char *firmware_old  = "firmware.cur";


void check_sd_firmware(void)
{
	volatile int f;
 	printf("Check SD\n");
	f_mount(0, &fat);
	if ((f = f_open(&file, firmware_file, FA_READ)) == FR_OK)
	{
 		printf("Flashing firmware...\n");
		uint8_t buf[512];
		unsigned int r = sizeof(buf);
		uint32_t address = USER_FLASH_START;
		while (r == sizeof(buf))
		{
			if (f_read(&file, buf, sizeof(buf), &r) != FR_OK)
			{
				f_close(&file);
				return;
			}
			write_flash((void *) address, (char *)buf, sizeof(buf));
			address += r;
		}
		f_close(&file);
		if (address > USER_FLASH_START)
		{
 			printf("Complete!\n");
			r = f_unlink(firmware_old);
			r = f_rename(firmware_file, firmware_old);
		}
	}
	else
	{
 		printf("open: %d\n", f);
	}
}

// this seems to fix an issue with handoff after poweroff
// found here http://knowledgebase.nxp.trimm.net/showthread.php?t=2869
typedef void __attribute__((noreturn))(*exec)();

static void boot(uint32_t a)
{
    uint32_t *start;

    __set_MSP(*(uint32_t *)USER_FLASH_START);
    start = (uint32_t *)(USER_FLASH_START + 4);
    ((exec)(*start))();
}

static uint32_t delay_loop(uint32_t count)
{
	volatile uint32_t j, del;
	for(j=0; j<count; ++j){
		del=j; // volatiles, so the compiler will not optimize the loop
	}
	return del;
}

static void new_execute_user_code(void)
{
	uint32_t addr=(uint32_t)USER_FLASH_START;
	// delay
	delay_loop(3000000);
	// relocate vector table
	SCB->VTOR = (addr & 0x1FFFFF80);
	// switch to RC generator
	LPC_SC->PLL0CON = 0x1; // disconnect PLL0
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;
	while (LPC_SC->PLL0STAT&(1<<25));
	LPC_SC->PLL0CON = 0x0;    // power down
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;
	while (LPC_SC->PLL0STAT&(1<<24));
	// disable PLL1
	LPC_SC->PLL1CON   = 0;
	LPC_SC->PLL1FEED  = 0xAA;
	LPC_SC->PLL1FEED  = 0x55;
	while (LPC_SC->PLL1STAT&(1<<9));

	LPC_SC->FLASHCFG &= 0x0fff;  // This is the default flash read/write setting for IRC
	LPC_SC->FLASHCFG |= 0x5000;
	LPC_SC->CCLKCFG = 0x0;     //  Select the IRC as clk
	LPC_SC->CLKSRCSEL = 0x00;
	LPC_SC->SCS = 0x00;		    // not using XTAL anymore
	delay_loop(1000);
	// reset pipeline, sync bus and memory access
	__asm (
		   "dmb\n"
		   "dsb\n"
		   "isb\n"
		  );
	boot(addr);
}

int main(void)
{
	WDT_Feed();
	
	// turn off stuff that should be off
	
	// hot end heater output
	GPIO_init(P3_26); GPIO_output(P3_26); GPIO_write(P3_26, 0);
	// heated bed heater output
	GPIO_init(P1_19); GPIO_output(P1_19); GPIO_write(P1_19, 0);
	// part cooling fan output
	GPIO_init(P3_25); GPIO_output(P3_25); GPIO_write(P3_25, 0);
	// extruder fan output
	GPIO_init(P0_27); GPIO_output(P0_27); GPIO_write(P0_27, 0);
	// beeper
	GPIO_init(P0_19); GPIO_output(P0_19); GPIO_write(P0_19, 0);

	// turn off LED for now
	GPIO_init(LED); GPIO_output(LED); GPIO_write(LED,0);

	printf("Bootloader Start\n");

	// give SD card time to wake up
	for (volatile int i = (1UL<<12); i; i--);

	SDCard_init(P0_9, P0_8, P0_7, P0_6);
	if (SDCard_disk_initialize() == 0)
		check_sd_firmware();

	int boot_failed = 0;
	
	if (WDT_ReadTimeOutFlag()) {
		WDT_ClrTimeOutFlag();
		printf("Watchdog reset, halted\n");
		boot_failed = 1;
	}
	else if (*(uint32_t *)USER_FLASH_START == 0xFFFFFFFF) {
		printf("User flash empty, halted");
		boot_failed = 1;
	}
	
	// if boot failed, turn on LED and halt
	if (boot_failed) {
		GPIO_write(LED,1);
		for (;;);
	}

#ifdef WATCHDOG
	WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
	WDT_Start(1<<22);
#endif

	// grab user code reset vector
	volatile uint32_t p = (USER_FLASH_START +4);
	printf("Jumping to 0x%lx\n", p);

	new_execute_user_code();

	printf("This should never happen\n");

	for (volatile int i = (1<<18);i;i--);

	NVIC_SystemReset();
}


DWORD get_fattime(void)
{
#define	YEAR	2012U
#define MONTH	11
#define DAY		13
#define HOUR	20
#define MINUTE	13
#define SECOND	1
	return	((YEAR  & 127) << 25) |
			((MONTH &  15) << 21) |
			((DAY   &  31) << 16) |
			((HOUR  &  31) << 11) |
			((MINUTE & 63) <<  5) |
			((SECOND & 63) <<  0);
}


int _write(int fd, const char *buf, int buflen)
{
	// doesn't do anything (stripped out UART code)
	// redirect to emulator, or add back UART code, or whatever you want :)
	return buflen;
}


void NMI_Handler() {
 	printf("NMI\n");
	for (;;);
}
void HardFault_Handler() {
 	printf("HardFault\n");
	for (;;);
}
void MemManage_Handler() {
 	printf("MemManage\n");
	for (;;);
}
void BusFault_Handler() {
 	printf("BusFault\n");
	for (;;);
}
void UsageFault_Handler() {
 	printf("UsageFault\n");
	for (;;);
}
