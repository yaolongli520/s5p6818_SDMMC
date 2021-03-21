/*
 * Synopsys DesignWare Multimedia Card Interface driver
 *  (Based on NXP driver for lpc 31xx)
 *
 * Copyright (C) 2009 NXP Semiconductors
 * Copyright (C) 2009, 2010 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _S5P_MMC_H_
#define _S5P_MMC_H_

#define DW_MMC_240A		0x240a

#define SDMMC_CTRL		0x000
#define SDMMC_PWREN		0x004
#define SDMMC_CLKDIV		0x008
#define SDMMC_CLKSRC		0x00c
#define SDMMC_CLKENA		0x010
#define SDMMC_TMOUT		0x014
#define SDMMC_CTYPE		0x018
#define SDMMC_BLKSIZ		0x01c
#define SDMMC_BYTCNT		0x020
#define SDMMC_INTMASK		0x024
#define SDMMC_CMDARG		0x028
#define SDMMC_CMD		0x02c
#define SDMMC_RESP0		0x030
#define SDMMC_RESP1		0x034
#define SDMMC_RESP2		0x038
#define SDMMC_RESP3		0x03c
#define SDMMC_MINTSTS		0x040
#define SDMMC_RINTSTS		0x044
#define SDMMC_STATUS		0x048
#define SDMMC_FIFOTH		0x04c
#define SDMMC_CDETECT		0x050
#define SDMMC_WRTPRT		0x054
#define SDMMC_GPIO		0x058
#define SDMMC_TCBCNT		0x05c
#define SDMMC_TBBCNT		0x060
#define SDMMC_DEBNCE		0x064
#define SDMMC_USRID		0x068
#define SDMMC_VERID		0x06c
#define SDMMC_HCON		0x070
#define SDMMC_UHS_REG		0x074
#define SDMMC_RST_N		0x078
#define SDMMC_BMOD		0x080
#define SDMMC_PLDMND		0x084
#define SDMMC_DBADDR		0x088
#define SDMMC_IDSTS		0x08c
#define SDMMC_IDINTEN		0x090
#define SDMMC_DSCADDR		0x094
#define SDMMC_BUFADDR		0x098
#define SDMMC_CDTHRCTL		0x100
#define SDMMC_CLKCTRL		0x114
#define SDMMC_DATA(x)		(x)
/*
* Registers to support idmac 64-bit address mode
*/
#define SDMMC_DBADDRL		0x088
#define SDMMC_DBADDRU		0x08c
#define SDMMC_IDSTS64		0x090
#define SDMMC_IDINTEN64		0x094
#define SDMMC_DSCADDRL		0x098
#define SDMMC_DSCADDRU		0x09c
#define SDMMC_BUFADDRL		0x0A0
#define SDMMC_BUFADDRU		0x0A4

/*
 * Data offset is difference according to Version
 * Lower than 2.40a : data register offest is 0x100
 */
#define DATA_OFFSET		0x100
#define DATA_240A_OFFSET	0x200

/* shift bit field */
#define _SBF(f, v)		((v) << (f))

/* Control register defines */
#define SDMMC_CTRL_USE_IDMAC		BIT(25)
#define SDMMC_CTRL_CEATA_INT_EN		BIT(11)
#define SDMMC_CTRL_SEND_AS_CCSD		BIT(10)
#define SDMMC_CTRL_SEND_CCSD		BIT(9)
#define SDMMC_CTRL_ABRT_READ_DATA	BIT(8)
#define SDMMC_CTRL_SEND_IRQ_RESP	BIT(7)
#define SDMMC_CTRL_READ_WAIT		BIT(6)
#define SDMMC_CTRL_DMA_ENABLE		BIT(5)
#define SDMMC_CTRL_INT_ENABLE		BIT(4)
#define SDMMC_CTRL_DMA_RESET		BIT(2)	//DMA 复位
#define SDMMC_CTRL_FIFO_RESET		BIT(1)	//FIFO复位
#define SDMMC_CTRL_RESET		BIT(0)		//控制器复位
/* Clock Enable register defines */
#define SDMMC_CLKEN_LOW_PWR		BIT(16)
#define SDMMC_CLKEN_ENABLE		BIT(0)
/* time-out register defines */
#define SDMMC_TMOUT_DATA(n)		_SBF(8, (n))
#define SDMMC_TMOUT_DATA_MSK		0xFFFFFF00
#define SDMMC_TMOUT_RESP(n)		((n) & 0xFF)
#define SDMMC_TMOUT_RESP_MSK		0xFF
/* card-type register defines */
#define SDMMC_CTYPE_8BIT		BIT(16)
#define SDMMC_CTYPE_4BIT		BIT(0)
#define SDMMC_CTYPE_1BIT		0
/* Interrupt status & mask register defines */
#define SDMMC_INT_SDIO(n)		BIT(16 + (n))
#define SDMMC_INT_EBE			BIT(15)
#define SDMMC_INT_ACD			BIT(14)
#define SDMMC_INT_SBE			BIT(13)
#define SDMMC_INT_HLE			BIT(12)
#define SDMMC_INT_FRUN			BIT(11)
#define SDMMC_INT_HTO			BIT(10)
#define SDMMC_INT_VOLT_SWITCH		BIT(10) /* overloads bit 10! */
#define SDMMC_INT_DRTO			BIT(9)
#define SDMMC_INT_RTO			BIT(8)
#define SDMMC_INT_DCRC			BIT(7)
#define SDMMC_INT_RCRC			BIT(6)
#define SDMMC_INT_RXDR			BIT(5)
#define SDMMC_INT_TXDR			BIT(4)
#define SDMMC_INT_DATA_OVER		BIT(3)
#define SDMMC_INT_CMD_DONE		BIT(2)
#define SDMMC_INT_RESP_ERR		BIT(1)
#define SDMMC_INT_CD			BIT(0)	//卡探测中断位
#define SDMMC_INT_ERROR			0xbfc2
/* Command register defines */
#define SDMMC_CMD_START			BIT(31)
#define SDMMC_CMD_USE_HOLD_REG	BIT(29)
#define SDMMC_CMD_VOLT_SWITCH		BIT(28)	//电压开关位
#define SDMMC_CMD_CCS_EXP		BIT(23)
#define SDMMC_CMD_CEATA_RD		BIT(22)
#define SDMMC_CMD_UPD_CLK		BIT(21)
#define SDMMC_CMD_INIT			BIT(15)
#define SDMMC_CMD_STOP			BIT(14)
#define SDMMC_CMD_PRV_DAT_WAIT		BIT(13) //等待先前数据传输完成位
#define SDMMC_CMD_SEND_STOP		BIT(12)
#define SDMMC_CMD_STRM_MODE		BIT(11)
#define SDMMC_CMD_DAT_WR		BIT(10)
#define SDMMC_CMD_DAT_EXP		BIT(9)
#define SDMMC_CMD_RESP_CRC		BIT(8)
#define SDMMC_CMD_RESP_LONG		BIT(7)
#define SDMMC_CMD_RESP_EXP		BIT(6)
#define SDMMC_CMD_INDX(n)		((n) & 0x1F)	//不是0x3F?
/* Status register defines */
#define SDMMC_GET_FCNT(x)		(((x)>>17) & 0x1FFF)
#define SDMMC_STATUS_DMA_REQ		BIT(31)
#define SDMMC_STATUS_BUSY		BIT(9)	//卡数据忙状态位 =1 表示忙
/* FIFOTH register defines */
#define SDMMC_SET_FIFOTH(m, r, t)	(((m) & 0x7) << 28 | \
					 ((r) & 0xFFF) << 16 | \
					 ((t) & 0xFFF))
/* HCON register defines */
#define DMA_INTERFACE_IDMA		(0x0)
#define DMA_INTERFACE_DWDMA		(0x1)
#define DMA_INTERFACE_GDMA		(0x2)
#define DMA_INTERFACE_NODMA		(0x3)
#define SDMMC_GET_TRANS_MODE(x)		(((x)>>16) & 0x3)
#define SDMMC_GET_SLOT_NUM(x)		((((x)>>1) & 0x1F) + 1)
#define SDMMC_GET_HDATA_WIDTH(x)	(((x)>>7) & 0x7)
#define SDMMC_GET_ADDR_CONFIG(x)	(((x)>>27) & 0x1)
/* Internal DMAC interrupt defines IDSTS寄存器*/
#define SDMMC_IDMAC_INT_AI		BIT(9)	//异常中断位 DU 和 FBE 会导致这个位被设置,当 DU和FBE都清除写1清除这位;
#define SDMMC_IDMAC_INT_NI		BIT(8)	//正常中断位
#define SDMMC_IDMAC_INT_CES		BIT(5)
#define SDMMC_IDMAC_INT_DU		BIT(4)	//描述符不可用中断 若描述符的OWN位=0会产生这个中断
#define SDMMC_IDMAC_INT_FBE		BIT(2) //重大总线错误位,这位被设置具体读 [12:10]可以知道是发送时异常还是接收时异常
#define SDMMC_IDMAC_INT_RI		BIT(1)	//接收中断 表示已完成接收描述符 写1清除
#define SDMMC_IDMAC_INT_TI		BIT(0)	//发送中断 表示已完成发送描述符 写1清除
/* Internal DMAC bus mode bits */
#define SDMMC_IDMAC_ENABLE		BIT(7)
#define SDMMC_IDMAC_FB			BIT(1)
#define SDMMC_IDMAC_SWRESET		BIT(0)
/* H/W reset */
#define SDMMC_RST_HWACTIVE		0x1
/* Version ID register define */
#define SDMMC_GET_VERID(x)		((x) & 0xFFFF)
/* Card read threshold */
#define SDMMC_SET_RD_THLD(v, x)		(((v) & 0xFFF) << 16 | (x))
#define SDMMC_UHS_18V			BIT(0)
/* All ctrl reset bits */
#define SDMMC_CTRL_ALL_RESET_FLAGS \
	(SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET)

/* FIFO register access macros. These should not change the data endian-ness
 * as they are written to memory to be dealt with by the upper layers */
#define mci_fifo_readw(__reg)	__raw_readw(__reg)
#define mci_fifo_readl(__reg)	__raw_readl(__reg)
#define mci_fifo_readq(__reg)	__raw_readq(__reg)

#define mci_fifo_writew(__value, __reg)	__raw_writew(__reg, __value)
#define mci_fifo_writel(__value, __reg)	__raw_writel(__reg, __value)
#define mci_fifo_writeq(__value, __reg)	__raw_writeq(__reg, __value)

/* Register access macros */
#define mci_readl(dev, reg)			\
	readl_relaxed((dev)->regs + SDMMC_##reg)
#define mci_writel(dev, reg, value)			\
	writel_relaxed((value), (dev)->regs + SDMMC_##reg)

/* 16-bit FIFO access macros */
#define mci_readw(dev, reg)			\
	readw_relaxed((dev)->regs + SDMMC_##reg)
#define mci_writew(dev, reg, value)			\
	writew_relaxed((value), (dev)->regs + SDMMC_##reg)

/* 64-bit FIFO access macros */
#ifdef readq
#define mci_readq(dev, reg)			\
	readq_relaxed((dev)->regs + SDMMC_##reg)
#define mci_writeq(dev, reg, value)			\
	writeq_relaxed((value), (dev)->regs + SDMMC_##reg)
#else
/*
 * Dummy readq implementation for architectures that don't define it.
 *
 * We would assume that none of these architectures would configure
 * the IP block with a 64bit FIFO width, so this code will never be
 * executed on those machines. Defining these macros here keeps the
 * rest of the code free from ifdefs.
 */
#define mci_readq(dev, reg)			\
	(*(volatile u64 __force *)((dev)->regs + SDMMC_##reg))
#define mci_writeq(dev, reg, value)			\
	(*(volatile u64 __force *)((dev)->regs + SDMMC_##reg) = (value))

#define __raw_writeq(__value, __reg) \
	(*(volatile u64 __force *)(__reg) = (__value))
#define __raw_readq(__reg) (*(volatile u64 __force *)(__reg))
#endif

enum s5p_mci_state { /*状态枚举*/
	STATE_IDLE = 0, /*空闲 没有请求在处理*/
	STATE_SENDING_CMD,/*正在发送命令*/
	STATE_SENDING_DATA,/*正在发送数据*/
	STATE_DATA_BUSY,/*正在传输数据忙*/
	STATE_SENDING_STOP,/*正在发生停止*/
	STATE_DATA_ERROR,/*数据错误*/
	STATE_SENDING_CMD11,
	STATE_WAITING_CMD11_DONE,
};

enum {
	EVENT_CMD_COMPLETE = 0,/*命令完成事件*/
	EVENT_XFER_COMPLETE,/*传输完成事件*/
	EVENT_DATA_COMPLETE,/*数据完成事件*/
	EVENT_DATA_ERROR,
	EVENT_XFER_ERROR
};

enum {	/*传输方式*/
	TRANS_MODE_PIO = 0,/*直接传输*/
	TRANS_MODE_IDMAC,/*内部DMA传输*/
	TRANS_MODE_EDMAC/*外部DMA传输*/
};

struct s5p_mci_slot {
	struct mmc_host		*mmc;
	struct s5p_mci		*host;

	u32			ctype;

	struct mmc_request	*mrq;
	struct list_head	queue_node;

	unsigned int		clock;
	unsigned int		__clk_old;

	unsigned long		flags;
#define DW_MMC_CARD_PRESENT	0
#define DW_MMC_CARD_NEED_INIT	1
#define DW_MMC_CARD_NO_LOW_PWR	2
	int			id;
	int			sdio_id;
};



struct idmac_desc {
	__le32		des0;	/* Control Descriptor 控制标志*/
#define IDMAC_DES0_DIC	BIT(1)	//关闭中断位 清除这位会产生中断
#define IDMAC_DES0_LD	BIT(2)	//最后一个描述符
#define IDMAC_DES0_FD	BIT(3)	//这个标志位表示当前描述符是 首个描述符
#define IDMAC_DES0_CH	BIT(4)
#define IDMAC_DES0_ER	BIT(5)
#define IDMAC_DES0_CES	BIT(30)
#define IDMAC_DES0_OWN	BIT(31) // OWN 位 当这个位=0,表示当前描述符不可用会产生中断,INSTS的bit4会被设置.

	__le32		des1;	/* Buffer sizes 缓存数据字节数*/
#define IDMAC_SET_BUFFER1_SIZE(d, s) \
	((d)->des1 = ((d)->des1 & 0x03ffe000) | ((s) & 0x1fff))

	__le32		des2;	/* buffer 1 physical address 数据物理地址*/

	__le32		des3;	/* buffer 2 physical address 链表下一项的物理地址*/
};

/* Each descriptor can transfer up to 4KB of data in chained mode */
#define DW_MCI_DESC_DATA_LENGTH	0x1000


/* IP Quirks/flags. */
/* DTO fix for command transmission with IDMAC configured */
#define DW_MCI_QUIRK_IDMAC_DTO			BIT(0)
/* delay needed between retries on some 2.11a implementations */
#define DW_MCI_QUIRK_RETRY_DELAY		BIT(1)
/* High Speed Capable - Supports HS cards (up to 50MHz) */
#define DW_MCI_QUIRK_HIGHSPEED			BIT(2)
/* Unreliable card detection */
#define DW_MCI_QUIRK_BROKEN_CARD_DETECTION	BIT(3)



#endif /* _S5P_MMC_H_ */
