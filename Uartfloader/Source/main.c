/*
 * Poject TlsrTool FLOADER
 * pvvx 09/2019
 *
 * Poject TlsrComProg
 * pvvx 02/2020 * 10/2020 TLSR825x
 */
#include "common.h"
#include "analog.h"
#include "spi_i.h"
#include "flash.h"
#include "clock.h"
#include "crc.h"

/*
 * Calculator CRC-16/MODBUS: https://crccalc.com/
 */
//-------------------------------
#define VERSION_BCD 0x10 // 0x12 -> Ver 1.0
//------------------------------- Init UART ---
#define UART_BAUD 230400 // 115200 or 230400

#if UART_BAUD == 115200
#define bwpc 		12  // 24000000/(12+1)/(15+1)=115384.615
#define uartCLKdiv	15
#elif UART_BAUD == 230400
#define bwpc 		7  // 24000000/(7+1)/(12+1)=230769.23
#define uartCLKdiv	12
#elif UART_BAUD == 500000
#define bwpc 		7  // 24000000/(7+1)/(5+1)=500000
#define uartCLKdiv	5
#endif

#define DATA_BUFF_SIZE 		1024
#define UART_DMA_BUFF_SIZE (DATA_BUFF_SIZE+16)
#define UART_RX_BUFF_SIZE	UART_DMA_BUFF_SIZE
#define UART_TX_BUFF_SIZE	UART_DMA_BUFF_SIZE
//-------------------------------

#define SWIRE_OFF 0

enum{
	CMD_GET_VERSION = 0,		//0
	CMD_FLASH_READ,				//1
	CMD_FLASH_WRITE,			//2
	CMD_FLASH_SECT_ERASE,		//3
	CMD_FLASH_ALL_ERASE,		//4
	CMD_FLASH_GET_JEDEC_ID		//5
};

typedef struct __attribute__((packed)) _blk_head_t {
	u8 cmd;
	u8 addrl;
	u16 addrh;
}blk_head_t;

typedef struct __attribute__((packed)) _cmd_fread_t {
	u16 len;
}cmd_fread_t;

typedef struct __attribute__((packed)) _cmd_fwrite_t {
	u8  data[DATA_BUFF_SIZE];
}cmd_fwrite_t;

typedef struct __attribute__((packed)) _cmd_fserase_t {
	u16 cnt;
}cmd_fserase_t;

typedef struct __attribute__((packed)) _blk_rx_pkt_t{
	blk_head_t head;
	union __attribute__((packed)) {
		cmd_fread_t		fr;
		cmd_fwrite_t	fw;
		cmd_fserase_t	fs;
	};
} blk_rx_pkt_t;

typedef struct _dma_uart_buf_t {
	volatile u32 len;
	union __attribute__((packed)) {
		blk_rx_pkt_t pkt;
		u8 uc[UART_DMA_BUFF_SIZE-4];
		u16 uw[1];
		u32 ud[1];
	};
}dma_uart_buf_t;

dma_uart_buf_t urxb;
dma_uart_buf_t utxb;

//_attribute_ram_code_
inline void uart_init(void) {
#if CHIP_TYPE == MCU_CORE_8251
	// reg_uart_clk_div/reg_uart_ctrl0
	REG_ADDR32(0x094) = MASK_VAL(FLD_UART_CLK_DIV, uartCLKdiv, FLD_UART_CLK_DIV_EN, 1)
		|	((MASK_VAL( FLD_UART_BWPC, bwpc) // set bit width
			| MASK_VAL(FLD_UART_STOP_BIT, 1) // 00: 1 bit, 01: 1.5bit 1x: 2bits;
			| FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN) // enable UART DMA mode
		<< 16);
	reg_uart_rx_timeout = MASK_VAL(FLD_UART_TIMEOUT_BW, (bwpc+1)*12) | FLD_UART_BW_MUL2;
	// reg_dma0_addr/reg_dma0_ctrl
	REG_ADDR32(0xC00) = (unsigned short)((u32)(&urxb)) //set receive buffer address
		| (MASK_VAL(FLD_DMA_BUF_SIZE, UART_RX_BUFF_SIZE>>4) << 16)  //set rx buffer size
		| (FLD_DMA_WR_MEM << 24);  // set DMA0 mode to 0x01 for receive.write to memory
	REG_ADDR32(0xC04) = (unsigned short)((u32)(&utxb)) //set tx buffer address
		| 	(MASK_VAL(FLD_DMA_BUF_SIZE, UART_TX_BUFF_SIZE>>4)<< 16); //set tx buffer size
	// reg_dma0_addrHi, reg_dma1_addrHi, reg_dma2_addrHi, reg_dma3_addrHi = 0x04
	REG_ADDR32(0xc40) = 0x04040404;
	//	reg_dma_chn_en = (FLD_DMA_CHN0 | FLD_DMA_CHN1); reg_dma_chn_irq_msk = (FLD_DMA_CHN0 | FLD_DMA_CHN1);
	REG_ADDR16(0xc20) = 0x0303;

#else
	// reg_uart_clk_div/reg_uart_ctrl0
	REG_ADDR32(0x094) = MASK_VAL(FLD_UART_CLK_DIV, uartCLKdiv, FLD_UART_CLK_DIV_EN, 1)
		|	((MASK_VAL( FLD_UART_BWPC, bwpc) // set bit width
			| MASK_VAL(FLD_UART_STOP_BIT, 1) // 00: 1 bit, 01: 1.5bit 1x: 2bits;
			| FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN) // enable UART DMA mode
			<< 16);
	reg_uart_rx_timeout = MASK_VAL(FLD_UART_TIMEOUT_BW, (bwpc+1)*12) | FLD_UART_BW_MUL2;
	// reg_dma0_addr/reg_dma0_ctrl
	REG_ADDR32(0x500) = (unsigned short)((u32)(&urxb)) //set receive buffer address
		| 	((FLD_DMA_WR_MEM // set DMA0 mode to 0x01 for receive.write to memory
			| MASK_VAL(FLD_DMA_BUF_SIZE, UART_RX_BUFF_SIZE>>4))  //set rx buffer size
			<< 16);
	REG_ADDR32(0x504) = (unsigned short)((u32)(&utxb)) //set tx buffer address
		| 	(MASK_VAL(FLD_DMA_BUF_SIZE, UART_TX_BUFF_SIZE>>4) //set tx buffer size
			<< 16);
#endif
}

_attribute_ram_code_ void flash_write_sector(u32 addr, u32 len, u8 *buf) {
	u32 sz = 256;
	while(len) {
		if (len < sz) sz = len;
		flash_write_page(addr, sz, buf);
		addr += sz;
		buf += sz;
		len -= sz;
	}
}

/*
_attribute_ram_code_  void * memcpy (void * to, const void * from, size_t size) {
	u8 * pto = (u8 *)to;
	u8 * pfrom = (u8 *)from;
	while(size) {
		size--;
		*pto++ = *pfrom++;
	}
	return to;
}
*/

_attribute_ram_code_ int main (void) {
	reg_irq_en = 0;
	// Open clk for MCU running
	REG_ADDR32(0x60) = 0xff000000;
	REG_ADDR16(0x64) = 0xffff;
#if CHIP_TYPE == MCU_CORE_8251
		analog_write(0x82,0x64);	//areg_clk_setting
		analog_write(0x52,0x80);
		analog_write(0x0b,0x38);
		analog_write(0x8c,0x02);
		analog_write(0x02,0xa2);	//rega_vol_ldo_ctrl

#if 0 		// GPIO wake up disable
		analog_write(0x27,0x00);	//PA wake up disable, rega_wakeup_en_val0
		analog_write(0x28,0x00);	//PB wake up disable, rega_wakeup_en_val1
		analog_write(0x29,0x00);	//PC wake up disable, rega_wakeup_en_val2
		analog_write(0x2a,0x00);	//PD wake up disable, rega_wakeup_en_val3
			// DMA channels disable
		REG_ADDR8(0xc20) = 0;	// reg_dma_chn_en = 0
		REG_ADDR8(0xc21) = 1;	// reg_dma_chn_irq_msk = FLD_DMA_CHN_UART_RX
			// Set DMA buffers hi address
		REG_ADDR32(0xc40) = 0x04040404; // reg_dma0_addrHi, reg_dma1_addrHi, reg_dma2_addrHi, reg_dma3_addrHi = 0x04
		REG_ADDR32(0xc44) = 0x04040404; // reg_dma4_addrHi, reg_dma5_addrHi, reg_dma_ta_addrHi, reg_dma_a3_addrHi = 0x04
		REG_ADDR8(0xc48) = 0x04;	// reg_dma7_addrHi = 0x04

		REG_ADDR16(0x750) = 0x1F40;
#else
		REG_ADDR8(0xc20) = 0;	// reg_dma_chn_en = 0
#endif
//		analog_write(0x01,((REG_ADDR8(0x7D) == 1)? 0x3c : 0x4c)); // rega_xtal_ctrl

		//reg_gpio_wakeup_irq |= FLD_GPIO_WAKEUP_EN | FLD_GPIO_INTERRUPT_EN; // [0x5b5]|=0x0c

		// select 24M as the system clock source.
		// clock.c : rc_24m_cal
		analog_write(0xc8, 0x80);
		analog_write(0x30, analog_read(0x30) | BIT(7));
		analog_write(0xc7, 0x0e);
		analog_write(0xc7, 0x0f);
		while((analog_read(0xcf) & 0x80) == 0);
		analog_write(0x33, analog_read(0xcb));		//write 24m cap into manual register
		analog_write(0x30, analog_read(0x30) & (~BIT(7)));
		analog_write(0xc7, 0x0e);
//	}
	reg_rst_clk = 0
	| FLD_CLK_SPI_EN
//	| FLD_CLK_I2C_EN
	| FLD_CLK_UART_EN
//	| FLD_CLK_USB_EN
//	| FLD_CLK_PWM_EN
//	| FLD_CLK_QDEC_EN
#if (!SWIRE_OFF)
	| FLD_CLK_SWIRE_EN
#endif
	;
    reg_clk_en = 0
	| FLD_CLK_ZB_EN
	| FLD_CLK_SYS_TIMER_EN
	| FLD_CLK_DMA_EN
//	| FLD_CLK_ALGM_EN
//	| FLD_CLK_AES_EN
//	| BIT(5)
//	| BIT(6)
//	| BIT(7)
//	| FLD_CLK_AIF_EN
//	| FLD_CLK_AUD_EN
//	| FLD_CLK_DFIFO_EN
//	| FLD_CLK_MC_EN
	| FLD_CLK_MCIC_EN
	| BIT(13) //?
//	| BIT(14)
//	| BIT(15)
	;
    reg_clk_sel = 0
    | MASK_VAL(FLD_SCLK_DIV, 6)
	// [6:5] select system clock source:
	//0: RC_24M from RC oscillator
	//1: FHS
	//2: HS divider (see 0x66[4:0])
	//3: 32M clock (48M * 2 /3 divider)
    | MASK_VAL(FLD_SCLK_SEL, 0)
    //if reg_fhs_sel = 0:
	//0: 48M clock doubled from 24M crystal, 1: RC_24M from RC oscillator,
	///if reg_fhs_sel = 1: Pad_24M from crystal oscillator
    | FLD_SCLK_HS_SEL
    ;
	reg_fhs_sel = 0
//	| FLD_FHS_SELECT	//0: 48M clock doubled from 24M crystal or RC_24M from RC oscillator, 1: Pad_24M from crystal oscillator
	;
#else
#error "Only TLSR825x!"
#endif
	// enable system tick ( clock_time() )
	reg_system_tick_ctrl = FLD_SYSTEM_TICK_START; //	REG_ADDR8(0x74f) = 0x01;
	crcInit();
	uart_init();
	// sws off and enable uart function and enable input
//	if(reg_mcu_tid == MCU_PROD_TID__825x) {
		// CPGIO PA0/PB1 enable uart function and enable input
		// PA0=RX
		analog_write(0x0e, (analog_read(0x0e) & 0xfc) | PM_PIN_PULLUP_1M);
#if (SWIRE_OFF)
		reg_gpio_func(GPIO_PA0) = ((~(GPIO_PA0)) & 0xff) | GPIO_PA7; // GPIO_PA7/SWS set gpio (0x586[7]=0)
		//BM_SET(reg_gpio_gpio_func(GPIO_PA7), GPIO_PA7 & 0xFF); // set PA7 as gpio (0x586[7]=0)
#else
		reg_gpio_func(GPIO_PA0) = (~(GPIO_PA7 | GPIO_PA0)) & 0xff; // GPIO_PA7/SWS & GPIO_PA0/RX disable as gpio (0x586[7]=0, 0x586[0]=0)
#endif
		BM_SET(reg_gpio_ie(GPIO_PA0), GPIO_PA0 & 0xFF);  // PA0 enable input (0x581[0]=1)
		reg_gpio_config_func(GPIO_PA0) = 2; // PA0 mux uart rx function (0x5a8[1:0] = 2)
		//  PB1=TX
		BM_CLR(reg_gpio_func(GPIO_PB1), GPIO_PB1 & 0xFF); // disable PB1 as gpio (0x58e[1] = 0)
		analog_write(areg_gpio_pb_ie, analog_read(areg_gpio_pb_ie) | (GPIO_PB1 & 0xFF)); // PB1 enable input afe_0xbd[1]=1
		reg_gpio_config_func(GPIO_PB1) = 0x04; // (0x5aa[3:2] = 1)
//	}
	/////////////////////////// app floader /////////////////////////////
	u16 crc16;
	while(1) {
#if MODULE_WATCHDOG_ENABLE
		WATCHDOG_CLEAR;  //in case of watchdog timeout
#endif
		if((reg_dma_tx_rdy0 & FLD_DMA_CHN_UART_TX) == 0) {
			if(reg_dma_irq_src & FLD_DMA_IRQ_UART_RX) { // new command?
				utxb.len = 0;
				if(urxb.len < sizeof(urxb.pkt.head) + 2) {
					// test only
//					utxb.ud[0] = 0x00000080; // BAD CMD
//					utxb.len = sizeof(urxb.pkt.head);
				} else {
					urxb.len -= 2;
					utxb.len = sizeof(blk_head_t);
					utxb.ud[0] = urxb.ud[0];
					crc16 = crcFast(urxb.uc, urxb.len);
					if((u8)crc16 != urxb.uc[urxb.len]
					 || (u8)(crc16>>8) != urxb.uc[urxb.len+1]) {
						utxb.pkt.head.cmd |= 0xC0; // BAD CRC
					} else {
						u32 faddr = urxb.ud[0] >> 8;
						switch(urxb.pkt.head.cmd) {
							case CMD_FLASH_READ: // rd 1024 bytes: 01 00 00 00 01 d8,  01 00 00 04 04 00 42 CB -> 01 00 00 04 55 AA 55 AA FE C4
								if(urxb.len == sizeof(urxb.pkt.head)
									|| urxb.pkt.fr.len > DATA_BUFF_SIZE)
									urxb.pkt.fr.len = DATA_BUFF_SIZE;
								utxb.len = urxb.pkt.fr.len + sizeof(urxb.pkt.head);
								flash_read_page(faddr, urxb.pkt.fr.len, utxb.pkt.fw.data);
								break;
							case CMD_FLASH_WRITE: // 02 00 00 04 55 AA 55 AA BE D1 -> 02 00 00 04 00 5F
								if(urxb.len == sizeof(urxb.pkt.head)
									|| urxb.len > DATA_BUFF_SIZE + sizeof(urxb.pkt.head))
									utxb.pkt.head.cmd |= 0x80;
								else
									flash_write_sector(faddr, urxb.len - sizeof(urxb.pkt.head), urxb.pkt.fw.data);
								break;
							case CMD_FLASH_SECT_ERASE: // 03 00 00 04 01 A3 -> 03 00 00 04 01 A3
								flash_erase_sector(faddr);
								break;
							case CMD_FLASH_GET_JEDEC_ID: // 05 00 00 00 00 E8 -> 05 51 40 13 21 34
								flash_get_jedec_id(&utxb.pkt.head.addrl);
								break;
							case CMD_FLASH_ALL_ERASE: // 04 00 00 00 01 14 -> 04 00 00 00 01 14
								if(!faddr) flash_erase_all();
								else utxb.pkt.head.cmd |= 0x80;
								break;
							case CMD_GET_VERSION: // 00 00 00 00 00 24 ->
								switch(urxb.pkt.head.addrl) {
								default: // case 0:
									utxb.pkt.head.addrl = VERSION_BCD;
									utxb.pkt.head.addrh = reg_prod_id; //chip id
									break;
								case 1:
									REG_ADDR8(0x6f) = 0x20;   // mcu reboot
									break;
								}
								break;
							default:	// 1F 34 56 78 79 BC -> 9F 34 56 78 50 7C
								utxb.pkt.head.cmd |= 0x80;
								break;
						}
					}
				}
				reg_dma_irq_src = FLD_DMA_IRQ_UART_RX | FLD_DMA_IRQ_UART_TX;
				if(utxb.len) {
					crc16 = crcFast(utxb.uc, utxb.len);
					utxb.uc[utxb.len++] = crc16;
					utxb.uc[utxb.len++] = crc16 >> 8;
					reg_dma_tx_rdy0 |= FLD_DMA_CHN_UART_TX; // start tx
				}
			}
		}
	}
	REG_ADDR8(0x6f) = 0x20;   //mcu reboot
	while (1);
}
