/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "debug.h"
#include "stdbool.h"
#include "string.h"
#include "bootloader.h"

#define BOOTLOADER_VERSION 10

//#define USE_PB8
#define USE_PA0

#define CH32_FLASH_START 0x08000000         //flash起始代码

#define FIRMWARE_RELATIVE_START 0x1000       //固件起始区域
//#define EEPROM_RELATIVE_START 0xFF00         //最后一页
#define EEPROM_RELATIVE_START 0x7C00         //最后一页

uint8_t __attribute__ ((section(".bootloader_info"))) bootloader_version = BOOTLOADER_VERSION;

typedef void (*pFunction)(void);

#define APPLICATION_ADDRESS      (uint32_t)(CH32_FLASH_START + FIRMWARE_RELATIVE_START) // 4k
#define EEPROM_START_ADD         (uint32_t)(CH32_FLASH_START + EEPROM_RELATIVE_START)
#define FLASH_END_ADD            (uint32_t)(CH32_FLASH_START + 0xFFFF)               // 64 k
#define APPLICATION_ADDRESS_RV   (uint32_t)(FIRMWARE_RELATIVE_START)


#define CMD_RUN              0x00
#define CMD_PROG_FLASH       0x01
#define CMD_ERASE_FLASH      0x02
#define CMD_READ_FLASH_SIL   0x03
#define CMD_VERIFY_FLASH     0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM      0x04
#define CMD_PROG_EEPROM      0x05
#define CMD_READ_SRAM        0x06
#define CMD_READ_FLASH_ATM   0x07
#define CMD_KEEP_ALIVE       0xFD
#define CMD_SET_ADDRESS      0xFF
#define CMD_SET_BUFFER       0xFE


#ifdef USE_PB8
#define input_pin        GPIO_Pin_8
#define input_port       GPIOB
#define PIN_NUMBER       8
#define PORT_LETTER      1
#endif

#ifdef USE_PA0
#define input_pin        GPIO_Pin_0
#define input_port       GPIOA
#define PIN_NUMBER       0
#define PORT_LETTER      1
#endif



#define  TIME_FACTOR     (12)


uint16_t low_pin_count = 0;
char receviedByte;
int receivedCount;
int count = 0;
char messagereceived = 0;
uint16_t invalid_command = 0;
uint16_t address_expected_increment;
int cmd = 0;
char eeprom_req = 0;
int received;
uint8_t port_letter;

uint8_t pin_code = PORT_LETTER << 4 | PIN_NUMBER;
uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x00,0x1f,0x06,0x06,0x01,0x30};      // default device info

size_t str_len;
char connected = 0;
uint8_t rxBuffer[258];
uint8_t payLoadBuffer[256];
char rxbyte=0;
uint32_t address;
int tick = 0;


typedef union __attribute__ ((packed))
{
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;
uint16_t len;
uint8_t received_crc_low_byte;
uint8_t received_crc_high_byte;
uint8_t calculated_crc_low_byte;
uint8_t calculated_crc_high_byte;
uint16_t payload_buffer_size;
char incoming_payload_no_command = 0;

char bootloaderactive = 1;

uint32_t JumpAddress;
//pFunction JumpToApplication;


static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
static void MX_GPIO_INPUT_INIT(void);
//void processmessage(void);
void serialwriteChar(char data);
void sendString(uint8_t data[], int len);
void recieveBuffer();

#define BAUDRATE          (19200)
#define BITTIME           (1000000/BAUDRATE)
#define HALFBITTIME       (500000/BAUDRATE)




void delayMicroseconds(uint32_t micros)
{
    SysTick->CNT = 0;
    while (SysTick->CNT < micros*TIME_FACTOR)
    {

    }
}

void jump(void)
{

    __disable_irq();
    JumpAddress = (APPLICATION_ADDRESS_RV);
    uint8_t value = *(uint8_t*)(EEPROM_START_ADD);
#ifdef USE_ADC_INPUT
#else
    if (value != 0x01)
    {      // check first byte of eeprom to see if its programmed, if not do not jump
        invalid_command = 0;
        return;
    }
#endif

    asm("li t0, 0x1000");
    asm("jr t0");

//   JumpToApplication = (pFunction) JumpAddress;
//   JumpToApplication( );
}

void makeCrc(uint8_t* pBuff, uint16_t length){
    static uint8_16_u CRC_16;
        CRC_16.word=0;

        for(int i = 0; i < length; i++)
        {
             uint8_t xb = pBuff[i];
             for (uint8_t j = 0; j < 8; j++)
             {
                 if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 )
                 {
                     CRC_16.word = CRC_16.word >> 1;
                     CRC_16.word = CRC_16.word ^ 0xA001;
                 }
                 else
                 {
                     CRC_16.word = CRC_16.word >> 1;
                 }
                 xb = xb >> 1;
             }
         }
        calculated_crc_low_byte = CRC_16.bytes[0];
        calculated_crc_high_byte = CRC_16.bytes[1];

}

char checkCrc(uint8_t* pBuff, uint16_t length){

        unsigned char received_crc_low_byte2 = pBuff[length];          // one higher than len in buffer
        unsigned char received_crc_high_byte2 = pBuff[length+1];
        makeCrc(pBuff,length);

//        PRINT("%x-%x %x-%x\n ",calculated_crc_low_byte,received_crc_low_byte2, calculated_crc_high_byte,received_crc_high_byte2);

        if((calculated_crc_low_byte==(uint8_t)received_crc_low_byte2)   && (calculated_crc_high_byte==(uint8_t)received_crc_high_byte2))
        {
            return 1;
        }
        else
        {
            return 0;
        }
}


void setReceive()
{
//    MX_GPIO_INPUT_INIT();
#ifdef USE_PB8
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGHR, 0xf<<0, 0x8<<0);
#endif
#ifdef USE_PA0
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x8<<0);
#endif
    received = 0;
}

void setTransmit()
{
#ifdef  USE_PB8
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGHR, 0xf<<0, 0x3<<0);
#endif

#ifdef USE_PA0
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x3<<0);
#endif
}



void send_ACK(){
    setTransmit();
    serialwriteChar(0x30);             // good ack!
    setReceive();
}

void send_BAD_ACK(){
    setTransmit();
    serialwriteChar(0xC1);                // bad command message.
    setReceive();
}

void send_BAD_CRC_ACK(){
    setTransmit();
    serialwriteChar(0xC2);                // bad command message.
    setReceive();
}

void sendDeviceInfo(){
    setTransmit();
    sendString(deviceInfo,9);
    setReceive();

}

bool checkAddressWritable(uint32_t address) {
    return address >= APPLICATION_ADDRESS;
}

void decodeInput(){
    if(incoming_payload_no_command){
        len = payload_buffer_size;
    //  received_crc_low_byte = rxBuffer[len];          // one higher than len in buffer
    //  received_crc_high_byte = rxBuffer[len+1];
        if(checkCrc(rxBuffer,len)){
            memset(payLoadBuffer, 0, sizeof(payLoadBuffer));             // reset buffer

            for(int i = 0; i < len; i++){
                payLoadBuffer[i]= rxBuffer[i];
            }
            send_ACK();
            incoming_payload_no_command = 0;
            return;
        }else{
            send_BAD_CRC_ACK();
            return;
        }
    }

    cmd = rxBuffer[0];
    PRINT("CMD:%x\n",cmd);

    if(rxBuffer[16] == 0x7d){
        if(rxBuffer[8] == 13 && rxBuffer[9] == 66){
            sendDeviceInfo();
            rxBuffer[20]= 0;

        }
        return;
    }

    if(rxBuffer[20] == 0x7d){
            if(rxBuffer[12] == 13 && rxBuffer[13] == 66){
                sendDeviceInfo();
                rxBuffer[20]= 0;
                return;
            }

    }
    if(rxBuffer[40] == 0x7d){
                if(rxBuffer[32] == 13 && rxBuffer[33] == 66){
                    sendDeviceInfo();
                    rxBuffer[20]= 0;
                    return;
                }
        }

    if(cmd == CMD_RUN){         // starts the main app
        if((rxBuffer[1] == 0) && (rxBuffer[2] == 0) && (rxBuffer[3] == 0)){
            invalid_command = 101;
        }
    }

    if(cmd == CMD_PROG_FLASH){
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len)) {
            send_BAD_CRC_ACK();

            return;
        }

        if (!checkAddressWritable(address)) {
            send_BAD_ACK();

            return;
        }

        save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size,address);
        send_ACK();

        return;
    }

    if(cmd == CMD_SET_ADDRESS){             //  command set addressinput format is: CMD, 00 , High byte address, Low byte address, crclb ,crchb
        len = 4;  // package without 2 byte crc
        if (!checkCrc((uint8_t*)rxBuffer, len))
        {
//            PRINT("bad crc %x %x %x %x %x %x\n",rxBuffer[0],rxBuffer[1],rxBuffer[2],rxBuffer[3],rxBuffer[4],rxBuffer[5]);
            send_BAD_CRC_ACK();
            return;
        }


        // will send Ack 0x30 and read input after transfer out callback
        invalid_command = 0;
        address = CH32_FLASH_START + (rxBuffer[2] << 8 | rxBuffer[3]);
        PRINT("addr:%x\n",address);
        send_ACK();

        return;
    }

    if(cmd == CMD_SET_BUFFER){        // for writing buffer rx buffer 0 = command byte.  command set address, input , format is CMD, 00 , 00 or 01 (if buffer is 256), buffer_size,
        len = 4;  // package without 2 byte crc
        if (!checkCrc((uint8_t*)rxBuffer, len)) {
            send_BAD_CRC_ACK();

            return;
        }

        // no ack with command set buffer;

        if(rxBuffer[2] == 0x01)
        {
            payload_buffer_size = 256;                          // if nothing in this buffer
        }
        else
        {
            payload_buffer_size = rxBuffer[3];
        }
        incoming_payload_no_command = 1;
        address_expected_increment = 256;
        setReceive();

        return;
    }

    if(cmd == CMD_KEEP_ALIVE){
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len)) {
            send_BAD_CRC_ACK();

            return;
        }

        setTransmit();
        serialwriteChar(0xC1);                // bad command message.
//        PRINT("1\n");
        setReceive();

        return;
    }

    if(cmd == CMD_ERASE_FLASH){
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len)) {
            send_BAD_CRC_ACK();

            return;
        }

        if (!checkAddressWritable(address)) {
            send_BAD_ACK();

            return;
        }

        send_ACK();
        return;
    }

    if(cmd == CMD_READ_EEPROM){
        eeprom_req = 1;
    }

    if(cmd == CMD_READ_FLASH_SIL){     // for sending contents of flash memory at the memory location set in bootloader.c need to still set memory with data from set mem command
        len = 2;
        if (!checkCrc((uint8_t*)rxBuffer, len)) {
            send_BAD_CRC_ACK();

            return;
        }

        count++;
        uint16_t out_buffer_size = rxBuffer[1];//
        if(out_buffer_size == 0){
            out_buffer_size = 256;
        }
        address_expected_increment = 128;

        setTransmit();
        uint8_t read_data[out_buffer_size + 3];        // make buffer 3 larger to fit CRC and ACK
        memset(read_data, 0, sizeof(read_data));
        //    read_flash((uint8_t*)read_data , address);                 // make sure read_flash reads two less than buffer.
        read_flash_bin((uint8_t*)read_data , address, out_buffer_size);

        makeCrc(read_data,out_buffer_size);
        read_data[out_buffer_size] = calculated_crc_low_byte;
        read_data[out_buffer_size + 1] = calculated_crc_high_byte;
        read_data[out_buffer_size + 2] = 0x30;
        sendString(read_data, out_buffer_size+3);

        setReceive();

        return;
    }

    setTransmit();
//    PRINT("cmd:%x\r\n",cmd);
    serialwriteChar(0xC1);                // bad command message.
    invalid_command++;
    setReceive();
}


void serialreadChar(void)
{
    rxbyte=0;
    while(!(input_port->INDR & input_pin))
    { // wait for rx to go high
        if(SysTick->CNT > (200000*TIME_FACTOR))
        {
            invalid_command = 101;
            return;
        }
    }
    while((input_port->INDR & input_pin))
    {   // wait for it go go low
            if(SysTick->CNT  > (250*TIME_FACTOR) && messagereceived)
            {
                return;
            }
    }
    delayMicroseconds(HALFBITTIME);//wait to get the center of bit time

    int bits_to_read = 0;
    while (bits_to_read < 8)
    {
        delayMicroseconds(BITTIME);
        rxbyte = rxbyte | ((( input_port->INDR & input_pin)) >> PIN_NUMBER) << bits_to_read;
        bits_to_read++;
    }
    delayMicroseconds(HALFBITTIME); //wait till the stop bit time begins
    messagereceived = 1;
    receviedByte = rxbyte;
//return rxbyte;
}


void serialwriteChar(char data)
{
    input_port->BCR = input_pin;  //initiate start bit
    char bits_to_read = 0;
    while (bits_to_read < 8)
    {
        delayMicroseconds(BITTIME);
        if (data & 0x01)
        {
            input_port->BSHR = input_pin;
        }
        else
        {
            input_port->BCR = input_pin;
        }
        bits_to_read++;
        data = data >> 1;
    }
    delayMicroseconds(BITTIME);
    input_port->BSHR = input_pin; //write the stop bit

// if more than one byte a delay is needed after stop bit,
//if its the only one no delay, the sendstring function adds delay after each bit

//if(cmd == 255 || cmd == 254 || cmd == 1  || incoming_payload_no_command){
//
//}else{
//  delayMicroseconds(BITTIME);
//}
}


void sendString(uint8_t *data, int len){

    for(int i = 0; i < len; i++){
        serialwriteChar(data[i]);
        delayMicroseconds(BITTIME);

    }
}


void recieveBuffer(void)
{

    //int i = 0;
    count = 0;
    messagereceived = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    for(int i = 0; i < sizeof(rxBuffer); i++)
    {
        serialreadChar();
        if(incoming_payload_no_command)
        {
            if(count == payload_buffer_size+2)
            {
                break;
            }
            rxBuffer[i] = rxbyte;
            count++;
        }
        else
        {
            if(SysTick->CNT > 250*TIME_FACTOR)
            {
                count = 0;
                break;
            }
            else
            {
                rxBuffer[i] = rxbyte;
                if(i == 257)
                {
                    invalid_command+=20;       // needs one hundred to trigger a jump but will be reset on next set address commmand
                }
            }
        }
    }

//    if(count)
//    {
//        for(int i=0;i<count;i++)
//        {
//            PRINT("%x ",rxBuffer[i]);
//        }
//        PRINT("\r\n");
//
//    }
    decodeInput();
}


void update_EEPROM( )
{
    read_flash_bin(rxBuffer , EEPROM_START_ADD , 48);
    if(BOOTLOADER_VERSION != rxBuffer[2])
    {
        if (rxBuffer[2] == 0xFF || rxBuffer[2] == 0x00)
        {
            return;
        }
        rxBuffer[2] = BOOTLOADER_VERSION;
        save_flash_nolib(rxBuffer, 48, EEPROM_START_ADD);
    }
}


void checkForSignal(){
    //uint8_t floating_or_signal= 0;
#ifdef USE_PB8
      MODIFY_REG(input_port->CFGHR, 0xf<<0, 0x8<<0);
      input_port->BCR = input_pin;
#endif
#ifdef USE_PA0
      MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x8<<0);
      input_port->BCR = input_pin;
#endif

      delayMicroseconds(500);

      for(int i = 0 ; i < 500; i ++)
      {
         if( !(input_port->INDR & input_pin))
         {
             low_pin_count++;
         }
         else
         {
             //   high_pin_count++;
         }

          delayMicroseconds(10);
      }
      if(low_pin_count == 0)
      {
         return;           // all high while pin is pulled low, bootloader signal
      }
      low_pin_count = 0;

#ifdef USE_PB8
      MODIFY_REG(input_port->CFGHR, 0xf<<0, 0x4<<0);
      input_port->BCR = input_pin;
#endif

#ifdef USE_PA0
      MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x4<<0);
      input_port->BCR = input_pin;
#endif
         delayMicroseconds(500);

         for(int i = 0 ; i < 500; i ++)
         {
             if( !(input_port->INDR & input_pin))
             {
                 low_pin_count++;
             }
             delayMicroseconds(10);
         }
         if(low_pin_count == 0)
         {
             return;            // when floated all
         }

         if(low_pin_count > 0)
         {
             jump();
         }

}



/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

int main(void)
{
    (void)bootloader_version;

//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

//    SystemCoreClockUpdate();
//    Delay_Init();
//    USART_Printf_Init(115200);

    PRINT("START\r\n");
    MX_TIM4_Init( );


    MX_GPIO_INPUT_INIT( );     // init the pin with a pulldown
    checkForSignal( );


#ifdef USE_PB8 //上拉输入
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGHR, 0xf<<0, 0x8<<0);
#endif

#ifdef USE_PA0
    input_port->BSHR = input_pin;
    MODIFY_REG(input_port->CFGLR, 0xf<<0, 0x8<<0);
#endif



    deviceInfo[3] = 0x14;
    update_EEPROM();
//    sendDeviceInfo( );


//    jump( );
    while(1)
    {
        recieveBuffer();
        if (invalid_command > 100)
        {
            jump();
        }
    }
}


//设置为上拉输入
static void MX_GPIO_INPUT_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};


#ifdef USE_PB8
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif
#ifdef USE_PA0
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#endif

  GPIO_InitStructure.GPIO_Pin = input_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(input_port, &GPIO_InitStructure);

}

static void MX_TIM4_Init(void)  //主频8M,时钟1M
{
    SysTick->CTLR=0;
    SysTick->SR = 0;
    SysTick->CNT=0;
    SysTick->CMP = (uint64_t)(-1);
    SysTick->CTLR = 0x9;

#if 0
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 47;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    TIM_ARRPreloadConfig(TIM4,DISABLE);
    TIM4->ATRLR_TIM4 = 0xffffffff;
#endif
}

