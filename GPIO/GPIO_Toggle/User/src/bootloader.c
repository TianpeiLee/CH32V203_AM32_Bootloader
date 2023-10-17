/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "bootloader.h"
#include <string.h>


//#define APP_START (uint32_t)0x08001000
//#define FLASH_STORAGE 0x08005000  // at the 31kb mark
#define page_size 0x100                   // 256 for L103

#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)


//FLASH_Unlock_Fast
//FLASH_ErasePage_Fast

void save_flash_nolib(uint8_t *data, int length, uint32_t add)
{
    volatile uint32_t start_addr/*,page_num*/;
	uint16_t data_to_FLASH[128];
	memset(data_to_FLASH, 0, 128);

	PRINT("len:%d add:%x\r\n",length,add);
	// unlock flash
	while ((FLASH->STATR & FLASH_STATR_BSY) != 0)
	{
	    /*  add time-out*/
	}
//	if ((FLASH->CTLR & FLASH_CTLR_FAST_LOCK) != 0)
	{
	    /* Authorize the FPEC of Bank1 Access */
	    FLASH->KEYR = FLASH_KEY1;
	    FLASH->KEYR = FLASH_KEY2;

	    /* Fast program mode unlock */
	    FLASH->MODEKEYR = FLASH_KEY1;
	    FLASH->MODEKEYR = FLASH_KEY2;
//	    FLASH_Unlock_Fast( );
	}

	// erase page if address even divisable by 1024
	if((add % 256) == 0)
	{
//	    FLASH->CTLR |= 0x00020000;
//	    FLASH->ADDR = add;
//	    FLASH->CTLR |= 0x00000040;
//	    while(FLASH->STATR & 0x00000001)
//	        ;
//	    FLASH->CTLR &= ~(0x00020000);

	    FLASH_ErasePage_Fast(add);
	}


//	page_num = (add - 0x08000000 + 255)/256;
//	start_addr = 0x08000000 + (page_num-1)*256;

	start_addr = (add & 0xFFFFFF00);  //256字节对齐  0x08001002

	for(int i=0;i<128;i++)
	{
	    data_to_FLASH[i] = *(  (volatile uint16_t *)( start_addr + 2*i )  ); //读取原来的值
	}

	for(int j=0;j<length/2;j++)
	{
	    data_to_FLASH[(add-start_addr)/2 + j] = data[j*2+1] << 8 | data[j*2];
	}



//    FLASH_BufReset( );

//    FLASH->CTLR |= 0x00010000;
//    FLASH->CTLR |= 0x00080000;
//    while(FLASH->STATR & 0x00000001);
//    FLASH->CTLR &= ~(0x00010000);


//    for(int i=0; i<64; i++)
//    {
//        FLASH_BufLoad(start_addr+4*i, *(((uint32_t *)data_to_FLASH) + i)  );
//
////        FLASH->CTLR |= 0x00010000;
////        *(__IO uint32_t *)(start_addr+4*i) = *(((uint32_t *)data_to_FLASH) + i);
////        FLASH->CTLR |= 0x00040000;
////        while(FLASH->STATR & 0x00000001)
////            ;
////        FLASH->CTLR &= ~(0x00010000);
//    }

    FLASH_ProgramPage_Fast(start_addr,((uint32_t *)data_to_FLASH));

//    FLASH->CTLR |= 0x00010000;
//    FLASH->ADDR = start_addr;
//    FLASH->CTLR |= 0x00000040;
//    while(FLASH->STATR & 0x00000001);
//    FLASH->CTLR &= ~(0x00010000);


//    FLASH->CTLR |= 0x00000080;
	 FLASH_Lock( );
}




void read_flash_bin(uint8_t*  data , uint32_t add , int out_buff_len){
	//volatile uint32_t read_data;
	for (int i = 0; i < out_buff_len ; i ++){
		data[i] = *(uint8_t*)(add + i);
	}
}



