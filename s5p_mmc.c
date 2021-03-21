#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/reset.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/blkdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/mmc/slot-gpio.h>

#include "s5p_mmc.h"


#define DW_MCI_DATA_ERROR_FLAGS	(SDMMC_INT_DRTO | SDMMC_INT_DCRC | \
				 SDMMC_INT_HTO | SDMMC_INT_SBE  | \
				 SDMMC_INT_EBE)
#define DW_MCI_CMD_ERROR_FLAGS	(SDMMC_INT_RTO | SDMMC_INT_RCRC | \
				 SDMMC_INT_RESP_ERR)
#define DW_MCI_ERROR_FLAGS	(DW_MCI_DATA_ERROR_FLAGS | \
				 DW_MCI_CMD_ERROR_FLAGS  | SDMMC_INT_HLE)
#define DW_MCI_SEND_STATUS	1
#define DW_MCI_RECV_STATUS	2
#define DW_MCI_DMA_THRESHOLD	16

#define DW_MCI_FREQ_MAX	200000000	/* unit: HZ */
#define DW_MCI_FREQ_MIN	400000		/* unit: HZ */

#define IDMAC_INT_CLR		(SDMMC_IDMAC_INT_AI | SDMMC_IDMAC_INT_NI | \
				 SDMMC_IDMAC_INT_CES | SDMMC_IDMAC_INT_DU | \
				 SDMMC_IDMAC_INT_FBE | SDMMC_IDMAC_INT_RI | \
				 SDMMC_IDMAC_INT_TI)


struct s5p_mci {
	spinlock_t		lock;
	spinlock_t		irq_lock;
	void __iomem		*regs;
	void __iomem		*fifo_reg;

	struct scatterlist	*sg;
	struct sg_mapping_iter	sg_miter;

	struct s5p_mci_slot	*cur_slot;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_command	stop_abort;
	unsigned int		prev_blksz;
	unsigned char		timing;

	/* DMA interface members*/
	int			use_dma;
	int			using_dma;

	dma_addr_t		sg_dma;//一致性DMA物理地址 :写入寄存器
	void			*sg_cpu;//一致性DMA物理地址对应的虚拟地址 :用于程序访问

	/* For idmac */
	unsigned int		ring_size;


	/* Registers's physical base address */
	resource_size_t		phy_regs;

	u32			cmd_status;
	u32			data_status;
	u32			stop_cmdr;
	u32			dir_status;
	struct tasklet_struct	tasklet;
	unsigned long		pending_events;
	unsigned long		completed_events;
	enum s5p_mci_state	state;

	u32			bus_hz;
	u32			current_speed;
	u32			num_slots;
	u32			fifoth_val;
	u16			verid;
	struct device		*dev;

	void			*priv;
	struct clk		*biu_clk;
	struct clk		*ciu_clk;
	struct s5p_mci_slot	*slot[2];
	/* FIFO push and pull */
	int			fifo_depth;
	int			data_shift;
	u8			part_buf_start;
	u8			part_buf_count;
	union {		/* 不使用DMA时 用到的共用体 */
		u16		part_buf16;
		u32		part_buf32;
		u64		part_buf;
	};

	/* Workaround flags */
	u32			quirks;

	bool			vqmmc_enabled;
	unsigned long		irq_flags; /* IRQ flags */
	int			irq;

	int			sdio_id0;

	struct timer_list       cmd11_timer;
	struct timer_list		cto_timer;
	struct timer_list       dto_timer;
};

/*
 * 获取DMA传输方向
 */	
static int s5p_mci_get_dma_dir(struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)//写
		return DMA_TO_DEVICE;//内存到外设
	else
		return DMA_FROM_DEVICE;//外设到内存
}

/*
 * 软复位 IDMAC
 */
static void s5p_mci_idmac_reset(struct s5p_mci *host) 
{
	u32 bmod = mci_readl(host, BMOD);//SDMMC_BMOD
	/* Software reset of DMA */
	bmod |= SDMMC_IDMAC_SWRESET;//软复位,设置后 1个周期自动清除
	mci_writel(host, BMOD, bmod);//设置1
}	

/*
 *  复位 MMC 控制器
 */
static bool s5p_mci_ctrl_reset(struct s5p_mci *host, u32 reset)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	u32 ctrl;

	ctrl = mci_readl(host, CTRL);//读控制寄存器
	ctrl |= reset;//设置要复位的位
	mci_writel(host, CTRL, ctrl);//写入寄存器开始复位

	/* 等待这个位被清除*/
	do {
		ctrl = mci_readl(host, CTRL);
		if (!(ctrl & reset))
			return true;//如清除返回 1
	} while (time_before(jiffies, timeout));

	dev_err(host->dev,
		"Timeout resetting block (ctrl reset %#x)\n",
		ctrl & reset);

	return false;//返回0
}

/*
 *  准备DMA传输 用于准备请求函数 和请求函数
 *  @next  准备时为1  请求处理时为0
 */
static int s5p_mci_pre_dma_transfer(struct s5p_mci *host,
				   struct mmc_data *data,
				   bool next)
{
	struct scatterlist *sg;//sg 链表指针
	unsigned int i, sg_len;

	if (!next && data->host_cookie)
		return data->host_cookie;
	//EMMC CMD8 存在 data->blocks:1 data->blksz:512 next = 0  data->host_cookie = 0
	// 这种情况在准备请求的时候不进行映射DMA 处理请求时才映射

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths. Also, we don't bother
	 * with all the DMA setup overhead for short transfers.
	 */
	if (data->blocks * data->blksz < DW_MCI_DMA_THRESHOLD)//总数据量少于DMA传输阈值
		return -EINVAL;

	if (data->blksz & 3)//数据块字节不是4的倍数
		return -EINVAL;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3)//检查地址和长度是否能整除4
			return -EINVAL;
	}

	sg_len = dma_map_sg(host->dev,
			    data->sg,
			    data->sg_len,
			    s5p_mci_get_dma_dir(data));//映射DMA链表
	if (sg_len == 0)
		return -EINVAL;

	if (next)
		data->host_cookie = sg_len;//保存链表的长度
	//EMMC CMD8 处理请求时映射1个块 但不设置到 data->host_cookie 中
	return sg_len;//返回长度
}

/*
 *  停止DMA传输 
 */	
static void s5p_mci_idmac_stop_dma(struct s5p_mci *host)//imdac停止
{
	u32 temp;

	/* Disable and reset the IDMAC interface 关闭和复位IMDAC接口*/
	temp = mci_readl(host, CTRL);//读CTRL寄存区
	temp &= ~SDMMC_CTRL_USE_IDMAC;//设置禁用IDMAC接口传输数据位
	temp |= SDMMC_CTRL_DMA_RESET;//启动复位 IDMAC接口控制位 这个位硬件自动清除
	mci_writel(host, CTRL, temp);//写入 CTRL寄存器

	/* Stop the IDMAC running 停止正在运行的IDMAC*/
	temp = mci_readl(host, BMOD);//读BMOD 寄存器
	temp &= ~(SDMMC_IDMAC_ENABLE | SDMMC_IDMAC_FB);//清除 IDMAC 使能位和突发传输位
	temp |= SDMMC_IDMAC_SWRESET;//设置IDMA 软复位 硬件清除
	mci_writel(host, BMOD, temp);//写入 BMOD 寄存器
}	

/*
 *  解除DMA映射
 */	
static void s5p_mci_dma_cleanup(struct s5p_mci *host)
{
	struct mmc_data *data = host->data;//host指针

	if (data)//host下面有data
		if (!data->host_cookie) {//如果data->host_cookie == 0
			//EMMC 这里用于解除CMD8,因为512字节不使用 pre 和 post 方法申请和释放
			dma_unmap_sg(host->dev,
				     data->sg,
				     data->sg_len,
				     s5p_mci_get_dma_dir(data));//解除dma映射
		}
}

/*
 *  完成DMA传输
 */	
static void s5p_mci_dmac_complete_dma(void *arg)
{
	struct s5p_mci *host = arg;
	struct mmc_data *data = host->data;

	dev_vdbg(host->dev, "DMA complete\n");
	//调试 host->use_dma = 1 即 TRANS_MODE_IDMAC
	if ((host->use_dma == TRANS_MODE_EDMAC) &&	//这里条件不满足 因为用的是IDMAC
	    data && (data->flags & MMC_DATA_READ)) //有数据传输且是读数据
		/* Invalidate cache after read */
		dma_sync_sg_for_cpu(mmc_dev(host->cur_slot->mmc),//这里用于未解除映射时访问分散聚集表
				    data->sg,
				    data->sg_len,
				    DMA_FROM_DEVICE);

	s5p_mci_dma_cleanup(host);//执行DMA清除操作 对于EMMC的CMD8会解除DMA映射 

	/*
	 * If the card was removed, data will be NULL. No point in trying to
	 * send the stop command or waiting for NBUSY in this case.
	 */
	if (data) {	//调试发现都是有数据,但可能存在卡被移除无数据
		set_bit(EVENT_XFER_COMPLETE, &host->pending_events);//设置完成传输位 
		tasklet_schedule(&host->tasklet);//调度工作
	}
}			   
		
static void s5p_mci_translate_sglist(struct s5p_mci *host, struct mmc_data *data,
				    unsigned int sg_len)
{
	unsigned int desc_len;
	int i;

	struct idmac_desc *desc_first, *desc_last, *desc; /*首描述符指针 下一个 */
	/*同时指向 DMA描述链表的虚拟首地址*/
	desc_first = desc_last = desc = host->sg_cpu;
	//调试发现 sg_len 链表数组长最小是1
	for (i = 0; i < sg_len; i++) {	//循环处理sg链表
		//调试发现 length = 1024 / 4096 等512的倍数 每个结构体需要传递的数据长度
		unsigned int length = sg_dma_len(&data->sg[i]);
		
		////获取物理地址:这个物理地址和描述符的物理地址不一样,这里是要写入卡的实际数据的物理地址
		u32 mem_addr = sg_dma_address(&data->sg[i]);

		for ( ; length ; desc++) {//循环直到完成这个DMA的有效数据的设置
			desc_len = (length <= DW_MCI_DESC_DATA_LENGTH) ?
				   length : DW_MCI_DESC_DATA_LENGTH;//决定DMA传输的长度 但是最大不能超过描述符限定值

			length -= desc_len;//计算剩余长度 用于控制循环结束

			/*
			 * Set the OWN bit and disable interrupts
			 * for this descriptor
			 */
			desc->des0 = cpu_to_le32(IDMAC_DES0_OWN |
						 IDMAC_DES0_DIC |
						 IDMAC_DES0_CH);

			/* Buffer length */
			IDMAC_SET_BUFFER1_SIZE(desc, desc_len);//设置这个描述符传输的长度到 bit0~bit12

			/* Physical address to DMA to/from */
			desc->des2 = cpu_to_le32(mem_addr);//获取物理地址 自动转换为小端地址 

			/* Update physical address for the next desc */
			mem_addr += desc_len;//物理地址 偏移 用于当一个描述符号装不下时 放到下一个

			/* Save pointer to the last descriptor */
			desc_last = desc;//每次操作完都 刷新 用于保存最后的描述符

			/*这里不处理desc3 成员 因为开始初始化的时候已经设置好了!!*/
		}
	}

	/* Set first descriptor */
	desc_first->des0 |= cpu_to_le32(IDMAC_DES0_FD);//对首个描述符 设置标志位

	/* Set last descriptor */
	desc_last->des0 &= cpu_to_le32(~(IDMAC_DES0_CH |
					   IDMAC_DES0_DIC));//最后一个描述符 清除关闭中断位,表示打开中断,这个描述符完成时候产生中断 
	desc_last->des0 |= cpu_to_le32(IDMAC_DES0_LD);//设置为最后一次描述符


	wmb(); /* drain writebuffer */
}

		
/*开始DMA传输*/
static int s5p_mci_idmac_start_dma(struct s5p_mci *host, unsigned int sg_len)
{
	u32 temp;

	s5p_mci_translate_sglist(host, host->data, sg_len);//把请求数据设置到描述符数组中

	/* Make sure to reset DMA in case we did PIO before this */
	s5p_mci_ctrl_reset(host, SDMMC_CTRL_DMA_RESET);//复位DMA
	s5p_mci_idmac_reset(host);//复位IDMAC

	/* Select IDMAC interface */
	temp = mci_readl(host, CTRL);//读控制寄存器
	temp |= SDMMC_CTRL_USE_IDMAC;//设置允许IDMAC进行数据传输
	mci_writel(host, CTRL, temp);//写入控制寄存器

	/* drain writebuffer */
	wmb();//写内存合并

	/* Enable the IDMAC */
	temp = mci_readl(host, BMOD);
	temp |= SDMMC_IDMAC_ENABLE | SDMMC_IDMAC_FB;//设置固定突发长度 并使能IDMAC位
	mci_writel(host, BMOD, temp);//写入BMOD寄存器

	/* Start it running */
	mci_writel(host, PLDMND, 1);//设置PLDMND寄存器 开启运行

	return 0;
}				   
				   
				   

/*
 *  获取写保护状态
 */				   
static int s5p_mci_get_ro(struct mmc_host *mmc)
{
	int read_only;
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	int gpio_ro = mmc_gpio_get_ro(mmc);

	/* Use platform get_ro function, else try on board write protect */
	if (!IS_ERR_VALUE(gpio_ro))
		read_only = gpio_ro;
	else
		read_only =
			mci_readl(slot->host, WRTPRT) & (1 << slot->id) ? 1 : 0;

	dev_dbg(&mmc->class_dev, "card is %s\n",
		read_only ? "read-only" : "read-write");

	return read_only;
}

static void s5p_mci_hw_reset(struct mmc_host *mmc)//硬件复位接口
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct s5p_mci *host = slot->host;
	int reset;

#if IS_ENABLED(CONFIG_MMC_DW_IDMAC)//如果使能IDMAC 则复位IDMAC
	dw_mci_idmac_reset(host);//复位IDMAC
#endif
	/*复位DMA 和 FIFO */
	if (!s5p_mci_ctrl_reset(host, SDMMC_CTRL_DMA_RESET |
				     SDMMC_CTRL_FIFO_RESET))	//复位成功返回1,继续执行,复位失败退出
		return;

	/*
	 * According to eMMC spec, card reset procedure:
	 * tRstW >= 1us:   RST_n pulse width
	 * tRSCA >= 200us: RST_n to Command time
	 * tRSTH >= 1us:   RST_n high period
	 */
	reset = mci_readl(host, RST_N);//读硬件复位寄存器
	reset &= ~(SDMMC_RST_HWACTIVE << slot->id);//插槽ID位 设置0
	mci_writel(host, RST_N, reset);//写入0开始复位
	usleep_range(1, 2);//延时1~2us
	reset |= SDMMC_RST_HWACTIVE << slot->id;//插槽ID位 设置1
	mci_writel(host, RST_N, reset);//写入1关闭复位
	usleep_range(200, 300);//复位完毕 延时200~300us
}

//等待卡数据线退出繁忙状态 最多等待500MS 
static void dw_mci_wait_while_busy(struct s5p_mci *host, u32 cmd_flags)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);//500MS

	/*
	 * Databook says that before issuing a new data transfer command
	 * we need to check to see if the card is busy.  Data transfer commands
	 * all have SDMMC_CMD_PRV_DAT_WAIT set, so we'll key off that.
	 *
	 * ...also allow sending for SDMMC_CMD_VOLT_SWITCH where busy is
	 * expected.
	 */
	if ((cmd_flags & SDMMC_CMD_PRV_DAT_WAIT) &&	//等待数据完成且位设置电压开关位
	    !(cmd_flags & SDMMC_CMD_VOLT_SWITCH)) {
		while (mci_readl(host, STATUS) & SDMMC_STATUS_BUSY) {//知道状态位清除
			if (time_after(jiffies, timeout)) {
				/* Command will fail; we'll pass error then */
				dev_err(host->dev, "Busy; trying anyway\n");
				break;
			}
			udelay(10);
		}
	}
}


/*
 * 发送命令 直到命令发送完成 
 * 
 */
static void mci_send_cmd(struct s5p_mci_slot *slot, u32 cmd, u32 arg)
{
	struct s5p_mci *host = slot->host;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);//500MS
	unsigned int cmd_status = 0;

	mci_writel(host, CMDARG, arg);//写入命令参数
	wmb(); /* drain writebuffer */
	dw_mci_wait_while_busy(host, cmd);//等待卡数据线退出繁忙
	mci_writel(host, CMD, SDMMC_CMD_START | cmd);//设置CMD开始位并写入

	while (time_before(jiffies, timeout)) {
		cmd_status = mci_readl(host, CMD);
		if (!(cmd_status & SDMMC_CMD_START))//直到 CMD开始位被硬件清除
			return;
	}
	dev_err(&slot->mmc->class_dev,
		"Timeout sending command (cmd %#x arg %#x status %#x)\n",
		cmd, arg, cmd_status);
}

static void s5p_mci_setup_bus(struct s5p_mci_slot *slot, bool force_clkinit)
{
	struct s5p_mci *host = slot->host;
	unsigned int clock = slot->clock;
	u32 div;
	u32 clk_en_a;
	u32 sdmmc_cmd_bits = SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT;


	/* We must continue to set bit 28 in CMD until the change is complete */
	if (host->state == STATE_WAITING_CMD11_DONE)
		sdmmc_cmd_bits |= SDMMC_CMD_VOLT_SWITCH;

	if (!clock) {
		mci_writel(host, CLKENA, 0);
		mci_send_cmd(slot, sdmmc_cmd_bits, 0);
	} else if (clock != host->current_speed || force_clkinit) {
		div = host->bus_hz / clock;
		if (host->bus_hz % clock && host->bus_hz > clock)
			/*
			 * move the + 1 after the divide to prevent
			 * over-clocking the card.
			 */
			div += 1;

		div = (host->bus_hz != clock) ? DIV_ROUND_UP(div, 2) : 0;

		if ((clock << div) != slot->__clk_old || force_clkinit)
			dev_info(&slot->mmc->class_dev,
				 "Bus speed (slot %d) = %dHz (slot req %dHz, actual %dHZ div = %d)\n",
				 slot->id, host->bus_hz, clock,
				 div ? ((host->bus_hz / div) >> 1) :
				 host->bus_hz, div);

		/* disable clock */
		mci_writel(host, CLKENA, 0);
		mci_writel(host, CLKSRC, 0);

		/* inform CIU */
		mci_send_cmd(slot, sdmmc_cmd_bits, 0);

		/* set clock to desired speed */
		mci_writel(host, CLKDIV, div);

		/* inform CIU */
		mci_send_cmd(slot, sdmmc_cmd_bits, 0);

		/* enable clock; only low power if no SDIO */
		clk_en_a = SDMMC_CLKEN_ENABLE << slot->id;
		if (!test_bit(DW_MMC_CARD_NO_LOW_PWR, &slot->flags))
			clk_en_a |= SDMMC_CLKEN_LOW_PWR << slot->id;
		mci_writel(host, CLKENA, clk_en_a);

		/* inform CIU */
		mci_send_cmd(slot, sdmmc_cmd_bits, 0);

		/* keep the clock with reflecting clock dividor */
		slot->__clk_old = clock << div;
	}

	host->current_speed = clock;

	/* Set the current slot bus width */
	mci_writel(host, CTYPE, (slot->ctype << slot->id));
}

static void s5p_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	u32 regs;

	switch (ios->bus_width) {//设置的数据线宽度
	case MMC_BUS_WIDTH_4:
		slot->ctype = SDMMC_CTYPE_4BIT;//4bit
		break;
	case MMC_BUS_WIDTH_8:
		slot->ctype = SDMMC_CTYPE_8BIT;//8bit
		break;
	default:
		/* set default 1 bit mode */
		slot->ctype = SDMMC_CTYPE_1BIT;//1bit
	}

	regs = mci_readl(slot->host, UHS_REG);//读寄存器 SDMMC_UHS_REG

	/* DDR mode set */
	if (ios->timing == MMC_TIMING_MMC_DDR52 ||
	    ios->timing == MMC_TIMING_UHS_DDR50 ||
	    ios->timing == MMC_TIMING_MMC_HS400)
		regs |= ((0x1 << slot->id) << 16);//如果需要DDR模式
	else
		regs &= ~((0x1 << slot->id) << 16);//如果不需要DDR模式

	mci_writel(slot->host, UHS_REG, regs);//设置到寄存器中
	slot->host->timing = ios->timing;//更新插槽的时序类型

	/*
	 * Use mirror of ios->clock to prevent race with mmc
	 * core ios update when finding the minimum.
	 */
	slot->clock = ios->clock;//更新插槽的时钟频率


	switch (ios->power_mode) {
	case MMC_POWER_UP://更新电源
		set_bit(DW_MMC_CARD_NEED_INIT, &slot->flags);//设置卡需要初始化
		regs = mci_readl(slot->host, PWREN);//读电源使能寄存器
		regs |= (1 << slot->id);//设置位
		mci_writel(slot->host, PWREN, regs);//写入寄存器
		break;
	case MMC_POWER_ON://打开电源
		if (!slot->host->vqmmc_enabled) {

			/* Keep track so we don't reset again */
			slot->host->vqmmc_enabled = true;
			

			/* Reset our state machine after powering on */
			s5p_mci_ctrl_reset(slot->host,
					  SDMMC_CTRL_ALL_RESET_FLAGS);
		}

		/* Adjust clock / bus width after power is up */
		s5p_mci_setup_bus(slot, false);

		break;
	case MMC_POWER_OFF://关闭电源
		/* Turn clock off before power goes down */
		s5p_mci_setup_bus(slot, false);
		
		slot->host->vqmmc_enabled = false;

		regs = mci_readl(slot->host, PWREN);
		regs &= ~(1 << slot->id);
		mci_writel(slot->host, PWREN, regs);
		break;
	default:
		break;
	}

	if (slot->host->state == STATE_WAITING_CMD11_DONE && ios->clock != 0)
		slot->host->state = STATE_IDLE;
}

static void s5p_mci_enable_sdio_irq(struct mmc_host *mmc, int enb)
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct s5p_mci *host = slot->host;
	unsigned long irqflags;
	u32 int_mask;

	spin_lock_irqsave(&host->irq_lock, irqflags);

	/* Enable/disable Slot Specific SDIO interrupt */
	int_mask = mci_readl(host, INTMASK);
	if (enb)
		int_mask |= SDMMC_INT_SDIO(slot->sdio_id);
	else
		int_mask &= ~SDMMC_INT_SDIO(slot->sdio_id);
	mci_writel(host, INTMASK, int_mask);

	spin_unlock_irqrestore(&host->irq_lock, irqflags);
}

static int s5p_mci_card_busy(struct mmc_host *mmc)//获取卡数据线忙状态
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	u32 status;

	/*
	 * Check the busy bit which is low when DAT[3:0]
	 * (the data lines) are 0000 数据线被拉低表示忙
	 */
	status = mci_readl(slot->host, STATUS);//读状态寄存器

	return !!(status & SDMMC_STATUS_BUSY);
}

static int s5p_mci_execute_tuning(struct mmc_host *mmc, u32 opcode)//几乎不调用
{
	int err = -EINVAL;

	printk("%s fail\n",__func__);
	return err;
}

static int s5p_mci_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct s5p_mci *host = slot->host;
	u32 uhs;
	u32 v18 = SDMMC_UHS_18V << slot->id;


	/*
	 * Program the voltage.  Note that some instances of dw_mmc may use
	 * the UHS_REG for this.  For other instances (like exynos) the UHS_REG
	 * does no harm but you need to set the regulator directly.  Try both.
	 */
	uhs = mci_readl(host, UHS_REG);
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		uhs &= ~v18;
	else
		uhs |= v18;

	mci_writel(host, UHS_REG, uhs);

	return 0;
}

static void s5p_mci_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct s5p_mci *host = slot->host;

	/*
	 * Low power mode will stop the card clock when idle.  According to the
	 * description of the CLKENA register we should disable low power mode
	 * for SDIO cards if we need SDIO interrupts to work.
	 */
	if (mmc->caps & MMC_CAP_SDIO_IRQ) {
		const u32 clken_low_pwr = SDMMC_CLKEN_LOW_PWR << slot->id;
		u32 clk_en_a_old;
		u32 clk_en_a;

		clk_en_a_old = mci_readl(host, CLKENA);

		if (card->type == MMC_TYPE_SDIO ||
		    card->type == MMC_TYPE_SD_COMBO) {
			set_bit(DW_MMC_CARD_NO_LOW_PWR, &slot->flags);
			clk_en_a = clk_en_a_old & ~clken_low_pwr;
		} else {
			clear_bit(DW_MMC_CARD_NO_LOW_PWR, &slot->flags);
			clk_en_a = clk_en_a_old | clken_low_pwr;
		}

		if (clk_en_a != clk_en_a_old) {
			mci_writel(host, CLKENA, clk_en_a);
			mci_send_cmd(slot, SDMMC_CMD_UPD_CLK |
				     SDMMC_CMD_PRV_DAT_WAIT, 0);
		}
	}
}

static int s5p_mci_prepare_hs400_tuning(struct mmc_host *mmc,
				       struct mmc_ios *ios)
{
	return 0;
}

/*
 * 准备请求函数 EMMC的CMD8 读512字节不使用这个函数映射
 */
static void s5p_mci_pre_req(struct mmc_host *mmc,
			   struct mmc_request *mrq,
			   bool is_first_req)//准备请求
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	//调试发现进入这个函数的请求都是有数据传输的请求
	if (!slot->host->use_dma || !data)//不使用DMA或者没有数据传输直接退出 
		return;

	if (data->host_cookie) {
		data->host_cookie = 0;//每一个新的请求 data->host_cookie 一般都是0
		return;
	}
	
	//这里会被调用 调用时 host->state:2 或者 host->state:0
	//准备DMA传输
	if (s5p_mci_pre_dma_transfer(slot->host, mrq->data, 1) < 0)
		data->host_cookie = 0;
}


static void s5p_mci_post_req(struct mmc_host *mmc,
			    struct mmc_request *mrq,
			    int err)
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!slot->host->use_dma || !data)
		return;

	if (data->host_cookie)
		dma_unmap_sg(slot->host->dev,
			     data->sg,
			     data->sg_len,
			     s5p_mci_get_dma_dir(data));
	data->host_cookie = 0;
}

static u32 s5p_mci_prepare_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct mmc_data	*data;
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct s5p_mci *host = slot->host;
	u32 cmdr;//用于写入cmd寄存器

	cmd->error = -EINPROGRESS;
	cmdr = cmd->opcode;//命令操作令牌号

	if (cmd->opcode == MMC_STOP_TRANSMISSION || 
	    cmd->opcode == MMC_GO_IDLE_STATE ||
	    cmd->opcode == MMC_GO_INACTIVE_STATE ||
	    (cmd->opcode == SD_IO_RW_DIRECT &&
	     ((cmd->arg >> 9) & 0x1FFFF) == SDIO_CCCR_ABORT))
		cmdr |= SDMMC_CMD_STOP;//CMD12 CMD0 CMD15 等停止命令要设置 停止位
	else if (cmd->opcode != MMC_SEND_STATUS && cmd->data)
		cmdr |= SDMMC_CMD_PRV_DAT_WAIT;//如果有数据且不是CMD13 设置等待之前的数据传输完成再发送命令

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		u32 clk_en_a;

		/* Special bit makes CMD11 not die */
		cmdr |= SDMMC_CMD_VOLT_SWITCH;

		/* Change state to continue to handle CMD11 weirdness */
		WARN_ON(slot->host->state != STATE_SENDING_CMD);
		slot->host->state = STATE_SENDING_CMD11;

		/*
		 * We need to disable low power mode (automatic clock stop)
		 * while doing voltage switch so we don't confuse the card,
		 * since stopping the clock is a specific part of the UHS
		 * voltage change dance.
		 *
		 * Note that low power mode (SDMMC_CLKEN_LOW_PWR) will be
		 * unconditionally turned back on in dw_mci_setup_bus() if it's
		 * ever called with a non-zero clock.  That shouldn't happen
		 * until the voltage change is all done.
		 */
		clk_en_a = mci_readl(host, CLKENA);
		clk_en_a &= ~(SDMMC_CLKEN_LOW_PWR << slot->id);
		mci_writel(host, CLKENA, clk_en_a);
		mci_send_cmd(slot, SDMMC_CMD_UPD_CLK |
			     SDMMC_CMD_PRV_DAT_WAIT, 0);
	}

	if (cmd->flags & MMC_RSP_PRESENT) { //命令有响应
		/* We expect a response, so set this bit */
		cmdr |= SDMMC_CMD_RESP_EXP;//命令有响应 设置期望响应位
		if (cmd->flags & MMC_RSP_136)
			cmdr |= SDMMC_CMD_RESP_LONG;//命令是长响应
	}

	if (cmd->flags & MMC_RSP_CRC)//命令需要检查响应的CRC
		cmdr |= SDMMC_CMD_RESP_CRC;//检查响应的CRC

	data = cmd->data;
	if (data) {
		cmdr |= SDMMC_CMD_DAT_EXP;//命令有数据传输
		if (data->flags & MMC_DATA_STREAM)
			cmdr |= SDMMC_CMD_STRM_MODE;//流模式
		if (data->flags & MMC_DATA_WRITE)
			cmdr |= SDMMC_CMD_DAT_WR;//数据块模式
	}

	cmdr |= SDMMC_CMD_USE_HOLD_REG;//prepare_command
	
	return cmdr;
}

static inline void dw_mci_set_cto(struct s5p_mci *host)//card time out
{
	unsigned int cto_clks;
	unsigned int cto_ms;
	unsigned long irqflags;

	cto_clks = mci_readl(host, TMOUT) & 0xff;//读卡超时寄存器SDMMC_TMOUT的响应超时值
	/*计算超时的毫秒数 1000 * cto_clks/host->bus_hz */
	cto_ms = DIV_ROUND_UP(cto_clks, host->bus_hz / 1000);//cto_clks,50000; cto_ms=1

	/* add a bit spare time */
	cto_ms += 10;//预留10MS

	/*
	 * 这个定时器是防止很长时间不产生中断,理论上中断很快产生
	 *
	 *如果cto_timer 定时器处理函数运行了我们认为这个中断是不会产生了!!!
	 */
	spin_lock_irqsave(&host->irq_lock, irqflags);
	if (!test_bit(EVENT_CMD_COMPLETE, &host->pending_events))//如果命令完成位 = 0 启动定时器
		mod_timer(&host->cto_timer,
			jiffies + msecs_to_jiffies(cto_ms) + 1);//设置定时器启动时间,处理函数 dw_mci_dto_timer
	spin_unlock_irqrestore(&host->irq_lock, irqflags);
}


static void s5p_mci_start_command(struct s5p_mci *host,
				 struct mmc_command *cmd, u32 cmd_flags)
{
	host->cmd = cmd;//host指向当前发送cmd
	dev_vdbg(host->dev,
		 "start command: ARGR=0x%08x CMDR=0x%08x\n",
		 cmd->arg, cmd_flags);

	mci_writel(host, CMDARG, cmd->arg);//SDMMC_CMDARG 写入命令参数
	wmb(); /* drain writebuffer */
	dw_mci_wait_while_busy(host, cmd_flags);//等待SD控制器不忙

	mci_writel(host, CMD, cmd_flags | SDMMC_CMD_START);//设置开始命令位 并写入命令寄存器

	/* response expected command only */
	if (cmd_flags & SDMMC_CMD_RESP_EXP)//如果命令会有响应
		dw_mci_set_cto(host);//设置响应超时函数
}


static void s5p_mci_ctrl_rd_thld(struct s5p_mci *host, struct mmc_data *data)
{
	unsigned int blksz = data->blksz;//数据块字节数
	u32 blksz_depth, fifo_depth;//块大小深度  / FIFO深度
	u16 thld_size;

	WARN_ON(!(data->flags & MMC_DATA_READ));//不是读操作

	/*
	 * 在240A之前不存在(事实上寄存器偏移在FIFO区域，
	 * 所以我们真的不应该访问它)。
	 */
	if (host->verid < DW_MMC_240A)//版本低于少于240a不操作 我们的是250a
		return;

	if (host->timing != MMC_TIMING_MMC_HS200 &&
	    host->timing != MMC_TIMING_MMC_HS400 &&
	    host->timing != MMC_TIMING_UHS_SDR104)	//如果时序不是 HS200,HS400,SDR104 则退出
		goto disable;

	blksz_depth = blksz / (1 << host->data_shift);//计算块字节深度
	fifo_depth = host->fifo_depth;//计算FIFO深度

	if (blksz_depth > fifo_depth)//块深度 > FIFO深度 则退出
		goto disable;

	/*
	 * If (blksz_depth) >= (fifo_depth >> 1), should be 'thld_size <= blksz'
	 * If (blksz_depth) <  (fifo_depth >> 1), should be thld_size = blksz
	 * Currently just choose blksz.
	 */
	thld_size = blksz;//阈值字节设置为块字节数
	/*设置卡读阈值并使能卡读阈值*/
	mci_writel(host, CDTHRCTL, SDMMC_SET_RD_THLD(thld_size, 1));//SDMMC_CDTHRCTL
	return;

disable:
	mci_writel(host, CDTHRCTL, SDMMC_SET_RD_THLD(0, 0));
}


static void s5p_mci_adjust_fifoth(struct s5p_mci *host, struct mmc_data *data)
{
	unsigned int blksz = data->blksz;
	const u32 mszs[] = {1, 4, 8, 16, 32, 64, 128, 256};
	u32 fifo_width = 1 << host->data_shift;
	u32 blksz_depth = blksz / fifo_width, fifoth_val;
	u32 msize = 0, rx_wmark = 1, tx_wmark, tx_wmark_invers;
	int idx = ARRAY_SIZE(mszs) - 1;

	/* pio should ship this scenario */
	if (!host->use_dma)
		return;

	tx_wmark = (host->fifo_depth) / 2;
	tx_wmark_invers = host->fifo_depth - tx_wmark;

	/*
	 * MSIZE is '1',
	 * if blksz is not a multiple of the FIFO width
	 */
	if (blksz % fifo_width) {
		msize = 0;
		rx_wmark = 1;
		goto done;
	}

	do {
		if (!((blksz_depth % mszs[idx]) ||
		     (tx_wmark_invers % mszs[idx]))) {
			msize = idx;
			rx_wmark = mszs[idx] - 1;
			break;
		}
	} while (--idx > 0);
	/*
	 * If idx is '0', it won't be tried
	 * Thus, initial values are uesed
	 */
done:
	fifoth_val = SDMMC_SET_FIFOTH(msize, rx_wmark, tx_wmark);
	mci_writel(host, FIFOTH, fifoth_val);
}

static int s5p_mci_submit_data_dma(struct s5p_mci *host, struct mmc_data *data)
{
	unsigned long irqflags;
	int sg_len;
	u32 temp;

	host->using_dma = 0;

	/* If we don't have a channel, we can't do DMA */
	if (!host->use_dma)
		return -ENODEV;

	sg_len = s5p_mci_pre_dma_transfer(host, data, 0);
	if (sg_len < 0) {
		s5p_mci_idmac_stop_dma(host);
		return sg_len;
	}

	host->using_dma = 1;

	if (host->use_dma == TRANS_MODE_IDMAC)
		dev_vdbg(host->dev,
			 "sd sg_cpu: %#lx sg_dma: %#lx sg_len: %d\n",
			 (unsigned long)host->sg_cpu,
			 (unsigned long)host->sg_dma,
			 sg_len);

	/*
	 * Decide the MSIZE and RX/TX Watermark.
	 * If current block size is same with previous size,
	 * no need to update fifoth.
	 */
	if (host->prev_blksz != data->blksz)
		s5p_mci_adjust_fifoth(host, data);

	/* Enable the DMA interface */
	temp = mci_readl(host, CTRL);
	temp |= SDMMC_CTRL_DMA_ENABLE;
	mci_writel(host, CTRL, temp);

	/* Disable RX/TX IRQs, let DMA handle it */
	spin_lock_irqsave(&host->irq_lock, irqflags);
	temp = mci_readl(host, INTMASK);
	temp  &= ~(SDMMC_INT_RXDR | SDMMC_INT_TXDR);
	mci_writel(host, INTMASK, temp);
	spin_unlock_irqrestore(&host->irq_lock, irqflags);

	if (s5p_mci_idmac_start_dma(host, sg_len)) {
		/* We can't do DMA */
		dev_err(host->dev, "%s: failed to start DMA.\n", __func__);
		return -ENODEV;
	}

	return 0;
}

/* 请求提交数据 */
static void s5p_mci_submit_data(struct s5p_mci *host, struct mmc_data *data)
{
	unsigned long irqflags;
	int flags = SG_MITER_ATOMIC;//使用原子方式
	u32 temp;

	data->error = -EINPROGRESS;

	WARN_ON(host->data);//host下挂有数据 报警
	host->sg = NULL;//sg设为NULL
	host->data = data;//下挂数据

	if (data->flags & MMC_DATA_READ) {
		host->dir_status = DW_MCI_RECV_STATUS;
		s5p_mci_ctrl_rd_thld(host, data);
	} else {
		host->dir_status = DW_MCI_SEND_STATUS;
	}

	if (s5p_mci_submit_data_dma(host, data)) {
		if (host->data->flags & MMC_DATA_READ)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;

		sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
		host->sg = data->sg;
		host->part_buf_start = 0;
		host->part_buf_count = 0;

		mci_writel(host, RINTSTS, SDMMC_INT_TXDR | SDMMC_INT_RXDR);

		spin_lock_irqsave(&host->irq_lock, irqflags);
		temp = mci_readl(host, INTMASK);
		temp |= SDMMC_INT_TXDR | SDMMC_INT_RXDR;
		mci_writel(host, INTMASK, temp);
		spin_unlock_irqrestore(&host->irq_lock, irqflags);

		temp = mci_readl(host, CTRL);
		temp &= ~SDMMC_CTRL_DMA_ENABLE;
		mci_writel(host, CTRL, temp);

		/*
		 * Use the initial fifoth_val for PIO mode.
		 * If next issued data may be transfered by DMA mode,
		 * prev_blksz should be invalidated.
		 */
		mci_writel(host, FIFOTH, host->fifoth_val);
		host->prev_blksz = 0;
	} else {
		/*
		 * Keep the current block size.
		 * It will be used to decide whether to update
		 * fifoth register next time.
		 */
		host->prev_blksz = data->blksz;
	}
}


static u32 s5p_mci_prep_stop_abort(struct s5p_mci *host, struct mmc_command *cmd)
{
	struct mmc_command *stop;//停止的mmc命令指针
	u32 cmdr;

	if (!cmd->data)//如果没数据传输 直接退出
		return 0;

	stop = &host->stop_abort;//获取主机的 停止命令地址
	cmdr = cmd->opcode;//获取命令索引
	memset(stop, 0, sizeof(struct mmc_command));

	if (cmdr == MMC_READ_SINGLE_BLOCK ||	//读单块
	    cmdr == MMC_READ_MULTIPLE_BLOCK ||	//读多块
	    cmdr == MMC_WRITE_BLOCK ||	//写单块
	    cmdr == MMC_WRITE_MULTIPLE_BLOCK || //写多块
	    cmdr == MMC_SEND_TUNING_BLOCK ||
	    cmdr == MMC_SEND_TUNING_BLOCK_HS200) {
		stop->opcode = MMC_STOP_TRANSMISSION;//设置命令为CMD12
		stop->arg = 0;//命令参数0
		stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	} else if (cmdr == SD_IO_RW_EXTENDED) {
		stop->opcode = SD_IO_RW_DIRECT;
		stop->arg |= (1 << 31) | (0 << 28) | (SDIO_CCCR_ABORT << 9) |
			     ((cmd->arg >> 28) & 0x7);
		stop->flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;
	} else {
		return 0;
	}

	cmdr = stop->opcode | SDMMC_CMD_STOP |
		SDMMC_CMD_RESP_CRC | SDMMC_CMD_RESP_EXP;

	return cmdr;
}

/*
 * 开始发送请求的命令
 */
static void __s5p_mci_start_request(struct s5p_mci *host,
				   struct s5p_mci_slot *slot,
				   struct mmc_command *cmd)
{
	struct mmc_request *mrq;
	struct mmc_data	*data;
	u32 cmdflags;//命令标志

	mrq = slot->mrq;//从插槽里面取出请求

	host->cur_slot = slot;//host指向当前插槽
	host->mrq = mrq;//host->mrq 指向这个请求
	
	/*清空挂起 完成事件 命令数据状态 方向状态*/
	host->pending_events = 0;//挂起事件=  0
	host->completed_events = 0;//完成事件=0
	host->cmd_status = 0;//命令状态=0
	host->data_status = 0;//数据状态=0
	host->dir_status = 0;//方向状态=0

	data = cmd->data;//指向命令的数据
	if (data) {//如果命令包含数据
		mci_writel(host, TMOUT, 0xFFFFFFFF);
		mci_writel(host, BYTCNT, data->blksz*data->blocks);//设置计数值为块字节 * 块总数
		mci_writel(host, BLKSIZ, data->blksz);//设置数据的块字节
	}
	/*命令结构体转SD控制器命令寄存器写入值 */
	cmdflags = s5p_mci_prepare_command(slot->mmc, cmd);//准备命令但不发送 

	/* this is the first command, send the initialization clock */
	if (test_and_clear_bit(DW_MMC_CARD_NEED_INIT, &slot->flags))
		cmdflags |= SDMMC_CMD_INIT;//上电后第一次发送命令需要 会调用1次 c0062000

	if (data) {
		s5p_mci_submit_data(host, data);//如果有数据则 提交数据
		wmb(); /* drain writebuffer */
	}

	s5p_mci_start_command(host, cmd, cmdflags);//发送命令

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		unsigned long irqflags;

		/*
		 * Databook says to fail after 2ms w/ no response, but evidence
		 * shows that sometimes the cmd11 interrupt takes over 130ms.
		 * We'll set to 500ms, plus an extra jiffy just in case jiffies
		 * is just about to roll over.
		 *
		 * We do this whole thing under spinlock and only if the
		 * command hasn't already completed (indicating the the irq
		 * already ran so we don't want the timeout).
		 */
		spin_lock_irqsave(&host->irq_lock, irqflags);
		if (!test_bit(EVENT_CMD_COMPLETE, &host->pending_events))
			mod_timer(&host->cmd11_timer,
				jiffies + msecs_to_jiffies(500) + 1);
		spin_unlock_irqrestore(&host->irq_lock, irqflags);
	}

	if (mrq->stop)//调试发现:有些请求没有数据也有停止命令 CMD18 读多块 CMD25写多块 但是他们没数据?
		host->stop_cmdr = s5p_mci_prepare_command(slot->mmc, mrq->stop);//如果请求中包含停止
	else	//CMD53 mrq->data 是有的但是没有 mrq->stop 
		host->stop_cmdr = s5p_mci_prep_stop_abort(host, cmd);//请求没停止命令,则根据命令自己组织停止命令
}



static void s5p_mci_start_request(struct s5p_mci *host,
				 struct s5p_mci_slot *slot)
{
	struct mmc_request *mrq = slot->mrq;//从插槽取出请求
	struct mmc_command *cmd;
	cmd = mrq->sbc ? mrq->sbc : mrq->cmd;//如果mrq->sbc不空,先发 mrq->sbc
	__s5p_mci_start_request(host, slot, cmd);
}


static void s5p_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)//实际这里要完成请求后才会再次调用
{
	struct s5p_mci_slot *slot = mmc_priv(mmc);//取出自己放入的私有数据
	struct s5p_mci *host = slot->host;//找到主机驱动的host结构

	WARN_ON(slot->mrq);

	/*
	 * The check for card presence and queueing of the request must be
	 * atomic, otherwise the card could be removed in between and the
	 * request wouldn't fail until another card was inserted.
	 */
	spin_lock_bh(&host->lock);

	if (!test_bit(DW_MMC_CARD_PRESENT, &slot->flags)) { /*如果卡被拔出*/
		spin_unlock_bh(&host->lock);
		mrq->cmd->error = -ENOMEDIUM;//设置错误码
		mmc_request_done(mmc, mrq);//直接设置当前请求为完成
		return;
	}

	slot->mrq = mrq;//插槽指向当前请求
	if (host->state == STATE_WAITING_CMD11_DONE) {
		dev_warn(&slot->mmc->class_dev,
			 "Voltage change didn't complete\n");
		host->state = STATE_IDLE;
	}
	host->state = STATE_SENDING_CMD;
	s5p_mci_start_request(host, slot);/*开始处理请求*/
	
	spin_unlock_bh(&host->lock);
}


//获取卡插拔状态 拔出返回0
static int s5p_mci_get_cd(struct mmc_host *mmc)
{
	int present;//为1表示卡存在,为0表示拔出
	struct s5p_mci_slot *slot = mmc_priv(mmc);
	struct s5p_mci *host = slot->host;
	int gpio_cd = mmc_gpio_get_cd(mmc);//发现TF卡插入返回1 拔出返回0  SDIO卡始终返回-38

	/* Use platform get_cd function, else try onboard card detect */
	if (mmc->caps & MMC_CAP_NONREMOVABLE)//比如EMMC不可拔出
		present = 1;
	else if (!IS_ERR_VALUE(gpio_cd))
		present = gpio_cd;
	else
		present = (mci_readl(slot->host, CDETECT) & (1 << slot->id))
			== 0 ? 1 : 0;//读卡探测寄存器 调试发现插拔卡都是读 1 出来

	spin_lock_bh(&host->lock);
	if (present) {
		set_bit(DW_MMC_CARD_PRESENT, &slot->flags);//
		dev_dbg(&mmc->class_dev, "card is present\n");
	} else {
		clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);
		dev_dbg(&mmc->class_dev, "card is not present\n");
	}
	spin_unlock_bh(&host->lock);

	return present;
}

static const struct mmc_host_ops s5p_mci_ops = {
	.request		= s5p_mci_request,//请求处理函数
	.pre_req		= s5p_mci_pre_req,//预备
	.post_req		= s5p_mci_post_req,//解出准备
	.set_ios		= s5p_mci_set_ios,//修改IO参数
	.get_ro			= s5p_mci_get_ro,//获取写保护状态
	.get_cd			= s5p_mci_get_cd,//获取卡插拔状态 插入返回1
	.hw_reset       = s5p_mci_hw_reset,//硬件复位接口
	.enable_sdio_irq	= s5p_mci_enable_sdio_irq,
	.execute_tuning		= s5p_mci_execute_tuning,
	.card_busy		= s5p_mci_card_busy,//卡忙状态获取
	.start_signal_voltage_switch = s5p_mci_switch_voltage,
	.init_card		= s5p_mci_init_card,
	.prepare_hs400_tuning	= s5p_mci_prepare_hs400_tuning,
};





static int s5p_mci_idmac_init(struct s5p_mci *host)
{
	int i;
	struct idmac_desc *p;
	/* 环缓冲区中的描述符数 */
	host->ring_size = PAGE_SIZE / sizeof(struct idmac_desc);/* 4096/16 = 256  */

	/* 设置传输链接描述符列表*/
	for (i = 0, p = host->sg_cpu; i < host->ring_size - 1;
		 i++, p++) { /*设置描述符结构体数组的[0]~[254]这255块的链表项*/
		p->des3 = cpu_to_le32(host->sg_dma +
				(sizeof(struct idmac_desc) * (i + 1)));
		p->des0 = 0;
		p->des1 = 0;
	}

	/* Set the last descriptor as the end-of-ring descriptor */
	p->des3 = cpu_to_le32(host->sg_dma);//最后一块指向第一块的地址
	p->des0 = cpu_to_le32(IDMAC_DES0_ER);//设置为结束块
	
	s5p_mci_idmac_reset(host);//复位IDMA
	
	/* Mask out interrupts - get Tx & Rx complete only */
	mci_writel(host, IDSTS, IDMAC_INT_CLR);//清除IDMAC 中断状态寄存器标志位
	mci_writel(host, IDINTEN, SDMMC_IDMAC_INT_NI |
			SDMMC_IDMAC_INT_RI | SDMMC_IDMAC_INT_TI);//设置中断使能寄存器 使能中断位

	/* Set the descriptor base address */
	mci_writel(host, DBADDR, host->sg_dma);//写入sg链表头部	
	
	return 0;
}

static void s5p_mci_init_dma(struct s5p_mci *host)
{
	int addr_config;

	host->use_dma = TRANS_MODE_IDMAC;
	
	/* Determine which DMA interface to use */

	/*
	* Check ADDR_CONFIG bit in HCON to find
	* IDMAC address bus width  32bit 地址模式
	*/
	addr_config = SDMMC_GET_ADDR_CONFIG(mci_readl(host, HCON));//打印 addr_config = 0 
	if(!addr_config){
		printk("32bit IDMAC address bus width \n");
	}
	
	/* host supports IDMAC in 32-bit address mode */
	dev_info(host->dev,
		 "IDMAC supports 32-bit address mode.\n");

	/* Alloc memory for sg translation */
	host->sg_cpu = dmam_alloc_coherent(host->dev, PAGE_SIZE,
					   &host->sg_dma, GFP_KERNEL);//分配一致性DMA内存 1页
	if (!host->sg_cpu) {
		dev_err(host->dev,
			"%s: could not alloc DMA memory\n",
			__func__);
		goto no_dma;
	}
	
	dev_info(host->dev, "Using internal DMA controller.\n");



	if (s5p_mci_idmac_init(host))	//执行init接口 s5p_mci_idmac_init 函数
		dev_err(host->dev, "%s: Unable to initialize DMA Controller.\n",
			__func__);
		
	return;

no_dma:
	dev_info(host->dev, "Using PIO mode.\n");
	host->use_dma = TRANS_MODE_PIO;
}

static void s5p_mci_cmd11_timer(unsigned long arg)
{
	struct s5p_mci *host = (struct s5p_mci *)arg;

	if (host->state != STATE_SENDING_CMD11) {
		dev_warn(host->dev, "Unexpected CMD11 timeout\n");
		return;
	}

	host->cmd_status = SDMMC_INT_RTO;
	set_bit(EVENT_CMD_COMPLETE, &host->pending_events);
	tasklet_schedule(&host->tasklet);
}


static void s5p_mci_cto_timer(unsigned long arg)
{
	struct s5p_mci *host = (struct s5p_mci *)arg;
	unsigned long irqflags;
	u32 pending;

	spin_lock_irqsave(&host->irq_lock, irqflags);

	/*
	 * If somehow we have very bad interrupt latency it's remotely possible
	 * that the timer could fire while the interrupt is still pending or
	 * while the interrupt is midway through running.  Let's be paranoid
	 * and detect those two cases.  Note that this is paranoia is somewhat
	 * justified because in this function we don't actually cancel the
	 * pending command in the controller--we just assume it will never come.
	 */
	pending = mci_readl(host, MINTSTS); /* read-only mask reg */
	if (pending & (DW_MCI_CMD_ERROR_FLAGS | SDMMC_INT_CMD_DONE)) {
		/* The interrupt should fire; no need to act but we can warn */
		dev_warn(host->dev, "Unexpected interrupt latency\n");
		goto exit;
	}
	if (test_bit(EVENT_CMD_COMPLETE, &host->pending_events)) {
		/* Presumably interrupt handler couldn't delete the timer */
		dev_warn(host->dev, "CTO timeout when already completed\n");
		goto exit;
	}

	/*
	 * Continued paranoia to make sure we're in the state we expect.
	 * This paranoia isn't really justified but it seems good to be safe.
	 */
	switch (host->state) {
	case STATE_SENDING_CMD11:
	case STATE_SENDING_CMD:
	case STATE_SENDING_STOP:
		/*
		 * If CMD_DONE interrupt does NOT come in sending command
		 * state, we should notify the driver to terminate current
		 * transfer and report a command timeout to the core.
		 */
		host->cmd_status = SDMMC_INT_RTO;
		set_bit(EVENT_CMD_COMPLETE, &host->pending_events);
		tasklet_schedule(&host->tasklet);
		break;
	default:
		dev_warn(host->dev, "Unexpected command timeout, state %d\n",
			 host->state);
		break;
	}

exit:
	spin_unlock_irqrestore(&host->irq_lock, irqflags);
}

static void s5p_mci_dto_timer(unsigned long arg)//命令响应中断长时间不产生时调用
{
	struct s5p_mci *host = (struct s5p_mci *)arg;
	unsigned long irqflags;
	u32 pending;

	spin_lock_irqsave(&host->irq_lock, irqflags);

	/*
	 * The DTO timer is much longer than the CTO timer, so it's even less
	 * likely that we'll these cases, but it pays to be paranoid.
	 */
	pending = mci_readl(host, MINTSTS); /* read-only mask reg */
	if (pending & SDMMC_INT_DATA_OVER) { //有数据完成中断 认为是意外的数据中断延迟
		/* The interrupt should fire; no need to act but we can warn */
		dev_warn(host->dev, "Unexpected data interrupt latency\n");
		goto exit;
	}
	if (test_bit(EVENT_DATA_COMPLETE, &host->pending_events)) {
		/* Presumably interrupt handler couldn't delete the timer */
		dev_warn(host->dev, "DTO timeout when already completed\n");
		goto exit;//当DTO超时却已经完成时
	}

	/*
	 * Continued paranoia to make sure we're in the state we expect.
	 * This paranoia isn't really justified but it seems good to be safe.
	 */
	switch (host->state) {
	case STATE_SENDING_DATA:
	case STATE_DATA_BUSY:
		/*
		 * If DTO interrupt does NOT come in sending data state,
		 * we should notify the driver to terminate current transfer
		 * and report a data timeout to the core.
		 */
		host->data_status = SDMMC_INT_DRTO;
		set_bit(EVENT_DATA_ERROR, &host->pending_events);//设置数据错误位
		set_bit(EVENT_DATA_COMPLETE, &host->pending_events);//设置数据完成位
		tasklet_schedule(&host->tasklet);//调度
		break;
	default:
		dev_warn(host->dev, "Unexpected data timeout, state %d\n",
			 host->state);
		break;
	}

exit:
	spin_unlock_irqrestore(&host->irq_lock, irqflags);
}

/*
 * 清除 pending_events 标志中的 EVENT_CMD_COMPLETE 位
 * 如果此位是0 返回0
 * 如果此位是1 停止CMD超时函数  清除 EVENT_CMD_COMPLETE 位并返回1
 */
static bool s5p_mci_clear_pending_cmd_complete(struct s5p_mci *host)
{
	if (!test_bit(EVENT_CMD_COMPLETE, &host->pending_events))
		return false;

	/*
	 * Really be certain that the timer has stopped.  This is a bit of
	 * paranoia and could only really happen if we had really bad
	 * interrupt latency and the interrupt routine and timeout were
	 * running concurrently so that the del_timer() in the interrupt
	 * handler couldn't run.
	 */
	WARN_ON(del_timer_sync(&host->cto_timer));
	clear_bit(EVENT_CMD_COMPLETE, &host->pending_events);

	return true;
}

/*
 * 命令完成处理函数
 * 
 * 清空 host->cmd_status
 * 拷贝响应寄存器的响应值
 * 检查命令的状态是否有错
 *
 * 命令完成无错返回 0 否则设置错误并返回错误码
 */
static int s5p_mci_command_complete(struct s5p_mci *host, struct mmc_command *cmd)
{
	u32 status = host->cmd_status;

	host->cmd_status = 0;/*清空host的命令状态*/

	/* Read the response from the card (up to 16 bytes) */
	if (cmd->flags & MMC_RSP_PRESENT) {	//如果命令需要响应
		if (cmd->flags & MMC_RSP_136) {	//如果是长影响
			cmd->resp[3] = mci_readl(host, RESP0);
			cmd->resp[2] = mci_readl(host, RESP1);
			cmd->resp[1] = mci_readl(host, RESP2);
			cmd->resp[0] = mci_readl(host, RESP3);
		} else {
			cmd->resp[0] = mci_readl(host, RESP0);//短响应
			cmd->resp[1] = 0;
			cmd->resp[2] = 0;
			cmd->resp[3] = 0;
		}
	}

	if (status & SDMMC_INT_RTO)
		cmd->error = -ETIMEDOUT;
	else if ((cmd->flags & MMC_RSP_CRC) && (status & SDMMC_INT_RCRC))
		cmd->error = -EILSEQ;
	else if (status & SDMMC_INT_RESP_ERR)
		cmd->error = -EIO;
	else
		cmd->error = 0;

	if (cmd->error) {
		/* newer ip versions need a delay between retries */
		if (host->quirks & DW_MCI_QUIRK_RETRY_DELAY)
			mdelay(20);
	}

	return cmd->error;
}


static void s5p_mci_stop_dma(struct s5p_mci *host)//停止DMA
{
	if (host->using_dma) {
		s5p_mci_idmac_stop_dma(host);//执行dma停止操作
		s5p_mci_dma_cleanup(host);//执行dma清除操作
	}

	/* Data transfer was stopped by the interrupt handler */
	set_bit(EVENT_XFER_COMPLETE, &host->pending_events);//设置传输完成位
}

/*
 * 发送停止命令如CMD12
 * 调试发现有mrq->sbc 的命令是直接跳过发送CMD12去停止传输的！！！
 * 因此这个函数在正常情况下不调用,只有异常的时候用于停止!!
 */
static inline void send_stop_abort(struct s5p_mci *host, struct mmc_data *data)
{
	//如果数据有停止则使用停止,否则使用中止
	struct mmc_command *stop = data->stop ? data->stop : &host->stop_abort;
	s5p_mci_start_command(host, stop, host->stop_cmdr);//发送停止或中止命令
}



static void s5p_mci_request_end(struct s5p_mci *host, struct mmc_request *mrq)
	__releases(&host->lock)
	__acquires(&host->lock)
{
	struct mmc_host	*prev_mmc = host->cur_slot->mmc;

	WARN_ON(host->cmd || host->data);

	host->cur_slot->mrq = NULL;
	host->mrq = NULL;

	dev_vdbg(host->dev, "list empty\n");//每个请求都是这里完成的

	if (host->state == STATE_SENDING_CMD11)
		host->state = STATE_WAITING_CMD11_DONE;
	else
		host->state = STATE_IDLE;//恢复空闲状态
	

	spin_unlock(&host->lock);
	mmc_request_done(prev_mmc, mrq);//提交到内核完成请求
	spin_lock(&host->lock);
}

/*
 * 启动读数据的软定时器函数 dto_timer
 * 
 */
static void s5p_mci_set_drto(struct s5p_mci *host)
{
	unsigned int drto_clks;
	unsigned int drto_ms;
	unsigned long irqflags;
	
	drto_clks = mci_readl(host, TMOUT) >> 8;//获取读数据计数超时值
	drto_ms = DIV_ROUND_UP(drto_clks, host->bus_hz / 1000);//根据时钟频率计算超时的毫秒

	/* add a bit spare time */
	drto_ms += 10;//增加10ms 

	spin_lock_irqsave(&host->irq_lock, irqflags);
	if (!test_bit(EVENT_DATA_COMPLETE, &host->pending_events)) //如果没设置数据传输完成
		mod_timer(&host->dto_timer,
			  jiffies + msecs_to_jiffies(drto_ms));//添加定时器
	spin_unlock_irqrestore(&host->irq_lock, irqflags);
}

/*
 * 测试 host->pending_events 的 EVENT_DATA_COMPLETE 位状态
 * 如果是0 返回0
 * 如果是1 清除位 并停止定时器  dto_timer 返回1
 */
static bool s5p_mci_clear_pending_data_complete(struct s5p_mci *host)
{
	if (!test_bit(EVENT_DATA_COMPLETE, &host->pending_events))
		return false;

	/* Extra paranoia just like dw_mci_clear_pending_cmd_complete() */
	WARN_ON(del_timer_sync(&host->dto_timer));
	clear_bit(EVENT_DATA_COMPLETE, &host->pending_events);

	return true;
}


static bool s5p_mci_reset(struct s5p_mci *host)
{
	u32 flags = SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET;//FIFO 和 控制器复位
	bool ret = false;

	/*
	 * Reseting generates a block interrupt, hence setting
	 * the scatter-gather pointer to NULL.
	 */
	if (host->sg) {
		sg_miter_stop(&host->sg_miter);
		host->sg = NULL;
	}

	if (host->use_dma)
		flags |= SDMMC_CTRL_DMA_RESET;

	if (s5p_mci_ctrl_reset(host, flags)) {
		/*
		 * In all cases we clear the RAWINTS register to clear any
		 * interrupts.
		 */
		mci_writel(host, RINTSTS, 0xFFFFFFFF);

		/* if using dma we wait for dma_req to clear */
		if (host->use_dma) {
			unsigned long timeout = jiffies + msecs_to_jiffies(500);
			u32 status;

			do {
				status = mci_readl(host, STATUS);
				if (!(status & SDMMC_STATUS_DMA_REQ))
					break;
				cpu_relax();
			} while (time_before(jiffies, timeout));

			if (status & SDMMC_STATUS_DMA_REQ) {
				dev_err(host->dev,
					"%s: Timeout waiting for dma_req to clear during reset\n",
					__func__);
				goto ciu_out;
			}

			/* when using DMA next we reset the fifo again */
			if (!s5p_mci_ctrl_reset(host, SDMMC_CTRL_FIFO_RESET))
				goto ciu_out;
		}
	} else {
		/* if the controller reset bit did clear, then set clock regs */
		if (!(mci_readl(host, CTRL) & SDMMC_CTRL_RESET)) {
			dev_err(host->dev,
				"%s: fifo/dma reset bits didn't clear but ciu was reset, doing clock update\n",
				__func__);
			goto ciu_out;
		}
	}

	if (host->use_dma == TRANS_MODE_IDMAC)
		/* It is also required that we reinit idmac */
		s5p_mci_idmac_init(host);

	ret = true;

ciu_out:
	/* After a CTRL reset we need to have CIU set clock registers  */
	mci_send_cmd(host->cur_slot, SDMMC_CMD_UPD_CLK, 0);

	return ret;
}

/*
 * 数据完成事件
 */
static int s5p_mci_data_complete(struct s5p_mci *host, struct mmc_data *data)
{
	u32 status = host->data_status;//获取中断状态 一把是0x8

	if (status & DW_MCI_DATA_ERROR_FLAGS) {
		if (status & SDMMC_INT_DRTO) {
			data->error = -ETIMEDOUT;
		} else if (status & SDMMC_INT_DCRC) {
			data->error = -EILSEQ;
		} else if (status & SDMMC_INT_EBE) {
			if (host->dir_status ==
				DW_MCI_SEND_STATUS) {
				/*
				 * No data CRC status was returned.
				 * The number of bytes transferred
				 * will be exaggerated in PIO mode.
				 */
				data->bytes_xfered = 0;
				data->error = -ETIMEDOUT;
			} else if (host->dir_status ==
					DW_MCI_RECV_STATUS) {
				data->error = -EIO;
			}
		} else {
			/* SDMMC_INT_SBE is included */
			data->error = -EIO;
		}

		dev_dbg(host->dev, "data error, status 0x%08x\n", status);

		/*
		 * After an error, there may be data lingering
		 * in the FIFO
		 */
		s5p_mci_reset(host);
	} else {	//没有错误
		data->bytes_xfered = data->blocks * data->blksz;//设置传输的字节数
		data->error = 0;//设置返回值0
	}

	return data->error;
}

/*
 * tasklet 任务操作函数
 */
static void s5p_mci_tasklet_func(unsigned long priv) //这个任务函数负责请求的结束
{
	struct s5p_mci *host = (struct s5p_mci *)priv;
	struct mmc_data	*data;
	struct mmc_command *cmd;
	struct mmc_request *mrq;
	enum s5p_mci_state state; /*状态*/
	enum s5p_mci_state prev_state; /*前一个状态*/
	unsigned int err;

	spin_lock(&host->lock);//自选锁

	state = host->state;//获取host状态
	data = host->data;//获取host的数据
	mrq = host->mrq;//获取host的请求

	/*循环处理直到状态变量 state 稳定*/
	do {
		prev_state = state;//记录初始化状态

		switch (state) {
		case STATE_IDLE:	//空闲状态
		case STATE_WAITING_CMD11_DONE:
			break;

		case STATE_SENDING_CMD11:
		case STATE_SENDING_CMD://正在发送命令状态 EVENT_CMD_COMPLETE, &host->pending_events
			if (!s5p_mci_clear_pending_cmd_complete(host)) /*pending 的命令完成位是0 则退出*/
				break;
			/*命令完成位被设置 并清除*/
			cmd = host->cmd;
			host->cmd = NULL;
			set_bit(EVENT_CMD_COMPLETE, &host->completed_events); /*设置完成事件的命令完成位 */
			err = s5p_mci_command_complete(host, cmd);//命令完成 拷贝响应 并返回错误码
			if (cmd == mrq->sbc && !err) {	//如果命令是sbc,且发送成功, 现在才开始真正发数据命令
				prev_state = state = STATE_SENDING_CMD;
				__s5p_mci_start_request(host, host->cur_slot,
						       mrq->cmd);
				goto unlock;
			}

			if (cmd->data && err) { /*传输数据且出错?*/
				s5p_mci_stop_dma(host);
				send_stop_abort(host, data);
				state = STATE_SENDING_STOP;
				break;
			}

			if (!cmd->data || err) { /*无数据的命令*/
				s5p_mci_request_end(host, mrq);//无数据又不是 sbc 命令完成 可结束请求
				goto unlock;
			}
			/*至此 表示数据命令的命令完成了 切换到数据发送状态*/
			prev_state = state = STATE_SENDING_DATA;/*状态切换为 数据发送中....*/
			/* fall through */

		case STATE_SENDING_DATA:	/* 数据发送中 */
			/*
			 * We could get a data error and never a transfer
			 * complete so we'd better check for it here.
			 *
			 * Note that we don't really care if we also got a
			 * transfer complete; stopping the DMA and sending an
			 * abort won't hurt.
			 */
			if (test_and_clear_bit(EVENT_DATA_ERROR,
					       &host->pending_events)) {
				s5p_mci_stop_dma(host);
				if (data->stop ||
				    !(host->data_status & (SDMMC_INT_DRTO |
							   SDMMC_INT_EBE)))
					send_stop_abort(host, data);
				state = STATE_DATA_ERROR;
				break;
			}
			
			if (!test_and_clear_bit(EVENT_XFER_COMPLETE,
						&host->pending_events)) {
				/*
				 * If all data-related interrupts don't come
				 * within the given time in reading data state.
				 */
				if (host->dir_status == DW_MCI_RECV_STATUS)
					s5p_mci_set_drto(host);//设置并启动 接收超时软定时函数 dto_timer
				break;
			}

			set_bit(EVENT_XFER_COMPLETE, &host->completed_events);//设置 completed_events 的传输完成位

			/*
			 * Handle an EVENT_DATA_ERROR that might have shown up
			 * before the transfer completed.  This might not have
			 * been caught by the check above because the interrupt
			 * could have gone off between the previous check and
			 * the check for transfer complete.
			 *
			 * Technically this ought not be needed assuming we
			 * get a DATA_COMPLETE eventually (we'll notice the
			 * error and end the request), but it shouldn't hurt.
			 *
			 * This has the advantage of sending the stop command.
			 */
			if (test_and_clear_bit(EVENT_DATA_ERROR,
					       &host->pending_events)) {
				s5p_mci_stop_dma(host);
				if (data->stop ||
				    !(host->data_status & (SDMMC_INT_DRTO |
							   SDMMC_INT_EBE)))
					send_stop_abort(host, data);
				state = STATE_DATA_ERROR;
				break;
			}
			prev_state = state = STATE_DATA_BUSY;

			/* fall through */

		case STATE_DATA_BUSY:
			/*返回1 清除 pending 的 数据完成位并停止 dto_timer*/
			if (!s5p_mci_clear_pending_data_complete(host)) { 
				/*
				 * If data error interrupt comes but data over
				 * interrupt doesn't come within the given time.
				 * in reading data state.
				 */
				if (host->dir_status == DW_MCI_RECV_STATUS)
					s5p_mci_set_drto(host);//返回0 启动定时器
				break;
			}

			host->data = NULL;
			set_bit(EVENT_DATA_COMPLETE, &host->completed_events);
			err = s5p_mci_data_complete(host, data);//数据完成 检查错误,和保存传输的字节总数

			if (!err) {	//正常返回err = 0
				if (!data->stop || mrq->sbc) { //不需要停止或者有 sbc 
					if (mrq->sbc && data->stop)//有sbc 不发送CMD12停止了 直接设置CMD12错误码为0
						data->stop->error = 0;
					s5p_mci_request_end(host, mrq);//不发CMD12直接结束请求
					goto unlock;
				}

				/* stop command for open-ended transfer*/
				if (data->stop)
					send_stop_abort(host, data);
			} else {
				/*
				 * If we don't have a command complete now we'll
				 * never get one since we just reset everything;
				 * better end the request.
				 *
				 * If we do have a command complete we'll fall
				 * through to the SENDING_STOP command and
				 * everything will be peachy keen.
				 */
				if (!test_bit(EVENT_CMD_COMPLETE,
					      &host->pending_events)) {
					host->cmd = NULL;
					s5p_mci_request_end(host, mrq);
					goto unlock;
				}
			}

			/*
			 * If err has non-zero,
			 * stop-abort command has been already issued.
			 */
			prev_state = state = STATE_SENDING_STOP;

			/* fall through */

		case STATE_SENDING_STOP:
			if (!s5p_mci_clear_pending_cmd_complete(host))
				break;

			/* CMD error in data command */
			if (mrq->cmd->error && mrq->data)
				s5p_mci_reset(host);

			host->cmd = NULL;
			host->data = NULL;

			if (mrq->stop)
				s5p_mci_command_complete(host, mrq->stop);
			else
				host->cmd_status = 0;

			s5p_mci_request_end(host, mrq);
			goto unlock;

		case STATE_DATA_ERROR:
			if (!test_and_clear_bit(EVENT_XFER_COMPLETE,
						&host->pending_events))
				break;

			state = STATE_DATA_BUSY;
			break;
		}
	} while (state != prev_state);//状态不一样继续循环

	host->state = state;//记录当前host的状态
unlock:
	spin_unlock(&host->lock);	
	
}

/* 命令完成中断时调用 */
static void s5p_mci_cmd_interrupt(struct s5p_mci *host, u32 status)
{
	del_timer(&host->cto_timer);//删除命令超时定时器

	if (!host->cmd_status)
		host->cmd_status = status;//将中断状态保存到命令状态中

	smp_wmb(); /* drain writebuffer */

	set_bit(EVENT_CMD_COMPLETE, &host->pending_events);//设置命令完成位
	tasklet_schedule(&host->tasklet);//s5p_mci_tasklet_func
}

//s5p_mci_set_part_bytes  负责把不足4字节的数据放入 part_buf 供给下一次写的时候用
//s5p_mci_push_part_bytes 负责在新的一次写入前把新的数据前面部分填入 part_buf 让他"尽量凑够" 4个字节

/* push final bytes to part_buf, only use during push */
static void s5p_mci_set_part_bytes(struct s5p_mci *host, void *buf, int cnt)
{
	memcpy((void *)&host->part_buf, buf, cnt);//把不足4字节的部分拷贝到 part_buf
	host->part_buf_count = cnt;//记录计数值
}

static int s5p_mci_push_part_bytes(struct s5p_mci *host, void *buf, int cnt)
{
	cnt = min(cnt, (1 << host->data_shift) - host->part_buf_count); 
	//写入数 >= (4 - part_buf_count) : cnt = (4 - part_buf_count)  这里是把上次不足4字节的缓冲填够4字节 比如缓冲填入2剩余2 这里写入98 填入2 凑够4 + 96
	//写入数 < (4 - part_buf_count) :  cnt = 写入数	这里是上次不足4字节的缓冲继续填入 比如缓冲填入2剩余2 这里再写入1 
	memcpy((void *)&host->part_buf + host->part_buf_count, buf, cnt);//将cnt字节填入 part_buf 中
	host->part_buf_count += cnt;
	return cnt;//返回填入字节
}

//FIFO 寄存器地址偏移是 0x200 重复往这个地址写入数据
static void s5p_mci_push_data32(struct s5p_mci *host, void *buf, int cnt)
{
	struct mmc_data *data = host->data;
	int init_cnt = cnt;

	/* 把上次写FIFO不足4字节的部分 和并到这里写操作一起处理*/
	if (unlikely(host->part_buf_count)) {	//如果上次有不足4字节没写的部分
		int len = s5p_mci_push_part_bytes(host, buf, cnt);//这里填入了len字节到 part_buf

		buf += len;//缓冲区基地址增加len 
		cnt -= len;//写入总数减去len 
		if (host->part_buf_count == 4) {	//如果part_buf_count 凑够4字节！！
			mci_fifo_writel(host->fifo_reg,	host->part_buf32);//写入这4个字节到FIFO
			host->part_buf_count = 0;//清空 part_buff
		}
	}
	
	{
		u32 *pdata = buf;

		for (; cnt >= 4; cnt -= 4)//每次写入4个字节
			mci_fifo_writel(host->fifo_reg, *pdata++);
		buf = pdata;
	}
	
	/* put anything remaining in the part_buf */
	if (cnt) {		/*不足4个字节部分 放入 part_buf */
		s5p_mci_set_part_bytes(host, buf, cnt);
		 /* Push data if we have reached the expected data length */
		if ((data->bytes_xfered + init_cnt) ==
		    (data->blksz * data->blocks)) //如果这次写完可以完成这个请求了 则把最后几个字节都写入。
			mci_fifo_writel(host->fifo_reg, host->part_buf32);
	}
}




/* 从 part_buf 中提取第一个字节数组，仅在提取期间使用 */
static int s5p_mci_pull_part_bytes(struct s5p_mci *host, void *buf, int cnt)
{
	cnt = min_t(int, cnt, host->part_buf_count);
	if (cnt) {
		memcpy(buf, (void *)&host->part_buf + host->part_buf_start,
		       cnt);
		host->part_buf_count -= cnt;
		host->part_buf_start += cnt;
	}
	return cnt;
}

//FIFO 寄存器地址偏移是 0x200 重复往这个地址读出数据
// EMMC 不使用这个函数
static void s5p_mci_pull_data32(struct s5p_mci *host, void *buf, int cnt)
{
		
	u32 *pdata = buf;

	for (; cnt >= 4; cnt -= 4)//因为32位的地址每次读出来是4个字节 
		*pdata++ = mci_fifo_readl(host->fifo_reg);//基地址+0x200 对应的虚拟地址
	buf = pdata;
	
	if (cnt) {	/** 不足4字节部分 **/
		/* part_buf32 和 part_buf 是共用体  */
		host->part_buf32 = mci_fifo_readl(host->fifo_reg);//读FIFO
		memcpy(buf, &host->part_buf, cnt);
		host->part_buf_start = cnt;
		host->part_buf_count = (1 << host->data_shift) - cnt; //4 - cnt
	}
}


static void s5p_mci_pull_data(struct s5p_mci *host, void *buf, int cnt)
{
	int len;

	/* get remaining partial bytes */
	len = s5p_mci_pull_part_bytes(host, buf, cnt);
	if (unlikely(len == cnt))
		return;
	buf += len;
	cnt -= len;

	/* get the rest of the data */
	s5p_mci_pull_data32(host, buf, cnt);

}

/** 当长度不足16字节,不用DMA,此时用下面方式 **/
static void s5p_mci_read_data_pio(struct s5p_mci *host, bool dto) //这个函数会执行
{
	struct sg_mapping_iter *sg_miter = &host->sg_miter;/* 获取迭代项地址 */
	void *buf;
	unsigned int offset;
	struct mmc_data	*data = host->data;
	int shift = host->data_shift;//2 比如FIFO可读2 可读的字节为 2<<2 = 8个
	u32 status;
	unsigned int len;
	unsigned int remain, fcnt;
	
	do {
		if (!sg_miter_next(sg_miter))//迭代映射sg
			goto done;
		/* 如果映射成功 */
		host->sg = sg_miter->piter.sg;//保存映射的sg
		buf = sg_miter->addr;//获取目的缓存地址
		remain = sg_miter->length;//获取目的缓存长度
		offset = 0;//目的缓存偏移

		do {
			fcnt = (SDMMC_GET_FCNT(mci_readl(host, STATUS))
					<< shift) + host->part_buf_count;//获取FIFO可读数据量
			len = min(remain, fcnt);//求可读和可存的最小值
			if (!len)
				break;
			/** 读取 len 字节数据到目的缓存 **/
			s5p_mci_pull_data(host, (void *)(buf + offset), len);
		
			data->bytes_xfered += len;//传输完成字节统计
			offset += len;//偏移自加
			remain -= len;//计算剩余
		} while (remain);

		sg_miter->consumed = offset;
		status = mci_readl(host, MINTSTS);
		mci_writel(host, RINTSTS, SDMMC_INT_RXDR);//清除接收FIFO中断
	/* if the RXDR is ready read again */
	} while ((status & SDMMC_INT_RXDR) ||
		 (dto && SDMMC_GET_FCNT(mci_readl(host, STATUS))));

	if (!remain) {
		if (!sg_miter_next(sg_miter))
			goto done;
		sg_miter->consumed = 0;
	}
	sg_miter_stop(sg_miter);
	return;

done:
	sg_miter_stop(sg_miter);
	host->sg = NULL;
	smp_wmb(); /* drain writebuffer */
	set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
}


//这个函数暂时未发现使用
static void s5p_mci_write_data_pio(struct s5p_mci *host)
{
	struct sg_mapping_iter *sg_miter = &host->sg_miter;/*迭代项*/
	void *buf;
	unsigned int offset;
	struct mmc_data	*data = host->data;/*获取 data*/
	int shift = host->data_shift;/* 2 */
	u32 status;
	unsigned int len;
	unsigned int fifo_depth = host->fifo_depth;
	unsigned int remain, fcnt;
	
	do {
		if (!sg_miter_next(sg_miter))
			goto done;

		host->sg = sg_miter->piter.sg;
		buf = sg_miter->addr;
		remain = sg_miter->length;
		offset = 0;

		do {
			fcnt = ((fifo_depth -
				 SDMMC_GET_FCNT(mci_readl(host, STATUS))) //FIFO深度 - FIFO中数据数 ,转换为字节数再乘以4
					<< shift) - host->part_buf_count;//计算FIFO最多可写入字节数
			len = min(remain, fcnt);//去所需写入和最大可写入的最小值
			if (!len)
				break;
			s5p_mci_push_data32(host, (void *)(buf + offset), len);//把数据提交到FIFO中,不足4字节部分,可能这次调用未写入FIFO！！！
			data->bytes_xfered += len; //已写入字节增加
			offset += len; //buf 便宜增加
			remain -= len;//剩余数减少 len
		} while (remain);

		sg_miter->consumed = offset;
		status = mci_readl(host, MINTSTS);
		mci_writel(host, RINTSTS, SDMMC_INT_TXDR);
	} while (status & SDMMC_INT_TXDR); /* if TXDR write again */

	if (!remain) {
		if (!sg_miter_next(sg_miter))
			goto done;
		sg_miter->consumed = 0;
	}
	sg_miter_stop(sg_miter);
	return;

done:
	sg_miter_stop(sg_miter);
	host->sg = NULL;
	smp_wmb(); /* drain writebuffer */
	set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
}


static void s5p_mci_handle_cd(struct s5p_mci *host)
{
	int i;

	for (i = 0; i < host->num_slots; i++) {
		struct s5p_mci_slot *slot = host->slot[i];

		if (!slot)
			continue;

		if (slot->mmc->ops->card_event)
			slot->mmc->ops->card_event(slot->mmc);
		mmc_detect_change(slot->mmc,
			msecs_to_jiffies(200));
	}
}



static irqreturn_t s5p_mci_interrupt(int irq, void *dev_id)
{
	struct s5p_mci *host = dev_id;
	u32 pending;
	int i;
	unsigned long irqflags;
	//读屏蔽后的中断状态寄存器
	pending = mci_readl(host, MINTSTS); /* read-only mask reg */

	dev_err(host->dev, "%s MINTSTS-0x%x \n",__func__,pending);
	/*
	 * DTO fix - version 2.10a and below, and only if internal DMA
	 * is configured.
	 */
	if (host->quirks & DW_MCI_QUIRK_IDMAC_DTO) { //对于2.10a以下的数据溢出中断补充
		if (!pending &&
		    ((mci_readl(host, STATUS) >> 17) & 0x1fff))
			pending |= SDMMC_INT_DATA_OVER;
	}
	
	if (pending) { //如果有中断发生
		/* Check volt switch first, since it can look like an error */
		if ((host->state == STATE_SENDING_CMD11) &&
		    (pending & SDMMC_INT_VOLT_SWITCH)) { /*首先检查电压开关，因为它可能看起来像一个错误*/
			mci_writel(host, RINTSTS, SDMMC_INT_VOLT_SWITCH);
			pending &= ~SDMMC_INT_VOLT_SWITCH;

			/*
			 * Hold the lock; we know cmd11_timer can't be kicked
			 * off after the lock is released, so safe to delete.
			 */
			spin_lock_irqsave(&host->irq_lock, irqflags);
			s5p_mci_cmd_interrupt(host, pending);
			spin_unlock_irqrestore(&host->irq_lock, irqflags);

			del_timer(&host->cmd11_timer);
		}

		if (pending & DW_MCI_CMD_ERROR_FLAGS) { //CMD相关错误标志位检查
			spin_lock_irqsave(&host->irq_lock, irqflags);//关中断 保留中断字
			del_timer(&host->cto_timer);//结束 命令超时 软定时
			/*不需要复位CMD线 直接清除 CMD_ERROR 相关的中断状态位*/
			mci_writel(host, RINTSTS, DW_MCI_CMD_ERROR_FLAGS);
			host->cmd_status = pending;//保留中断状态到 cmd_status 成员中
			smp_wmb(); /* drain writebuffer */
			set_bit(EVENT_CMD_COMPLETE, &host->pending_events);//事件位设置完成命令
			spin_unlock_irqrestore(&host->irq_lock, irqflags);//开中断 恢复中断字
		}

		if (pending & DW_MCI_DATA_ERROR_FLAGS) { //DATA 相关错误标志位检查
			/*不需要复位DATA线 直接清除 DATA_ERROR 相关的中断状态位*/
			mci_writel(host, RINTSTS, DW_MCI_DATA_ERROR_FLAGS);
			host->data_status = pending;//保留中断状态到 data_status 成员中
			smp_wmb(); /* drain writebuffer */
			set_bit(EVENT_DATA_ERROR, &host->pending_events);
			tasklet_schedule(&host->tasklet);
		}

		if (pending & SDMMC_INT_DATA_OVER) { //数据传输结束
			spin_lock_irqsave(&host->irq_lock, irqflags);

			del_timer(&host->dto_timer);//删除dto定时器

			mci_writel(host, RINTSTS, SDMMC_INT_DATA_OVER);//清除 DATA_OVER 中断位
			if (!host->data_status)
				host->data_status = pending; //保存中断状态到 data_status
			smp_wmb(); /* drain writebuffer */
			if (host->dir_status == DW_MCI_RECV_STATUS) { /*如果是接收数据*/
				if (host->sg != NULL)	//采用PIO方式
					s5p_mci_read_data_pio(host, true);//true = 1
			}
			set_bit(EVENT_DATA_COMPLETE, &host->pending_events);//设置数据完成到 pending_events
			tasklet_schedule(&host->tasklet);

			spin_unlock_irqrestore(&host->irq_lock, irqflags);
		}

		if (pending & SDMMC_INT_RXDR) { //接收FIFO有数据中断 暂时发现无此中断
			mci_writel(host, RINTSTS, SDMMC_INT_RXDR);
			if (host->dir_status == DW_MCI_RECV_STATUS && host->sg)
				s5p_mci_read_data_pio(host, false);
		}

		if (pending & SDMMC_INT_TXDR) {	//发送FIFO数据请求中断 暂时发现无此中断
			mci_writel(host, RINTSTS, SDMMC_INT_TXDR);
			if (host->dir_status == DW_MCI_SEND_STATUS && host->sg)
				s5p_mci_write_data_pio(host);
		}

		if (pending & SDMMC_INT_CMD_DONE) { //命令完成中断
			spin_lock_irqsave(&host->irq_lock, irqflags);

			mci_writel(host, RINTSTS, SDMMC_INT_CMD_DONE);//清除 CMD_DONE 中断标志位
			s5p_mci_cmd_interrupt(host, pending);//命令中断函数

			spin_unlock_irqrestore(&host->irq_lock, irqflags);
		}

		if (pending & SDMMC_INT_CD) { //卡探测中断
			mci_writel(host, RINTSTS, SDMMC_INT_CD);
			s5p_mci_handle_cd(host);
		}

		/* Handle SDIO Interrupts */
		for (i = 0; i < host->num_slots; i++) {
			struct s5p_mci_slot *slot = host->slot[i];

			if (!slot)
				continue;

			if (pending & SDMMC_INT_SDIO(slot->sdio_id)) {
				mci_writel(host, RINTSTS,
					   SDMMC_INT_SDIO(slot->sdio_id));
				mmc_signal_sdio_irq(slot->mmc);
			}
		}

	}

	if (host->use_dma != TRANS_MODE_IDMAC) //如果不是使用IDMAC则退出
		return IRQ_HANDLED;

	/* Handle IDMA interrupts */
	pending = mci_readl(host, IDSTS);
	
	if (pending & (SDMMC_IDMAC_INT_TI | SDMMC_IDMAC_INT_RI)) {
		mci_writel(host, IDSTS, SDMMC_IDMAC_INT_TI |
						SDMMC_IDMAC_INT_RI);//清除bit 0 和 bit 1
		mci_writel(host, IDSTS, SDMMC_IDMAC_INT_NI);//0和1 清除后再 清除bit 8
		s5p_mci_dmac_complete_dma((void *)host);//设置传输完成位 并调度工作
	}
	
	return IRQ_HANDLED;
}




static int s5p_mci_init_slot(struct s5p_mci *host, unsigned int id)
{
	struct mmc_host *mmc;
	struct s5p_mci_slot *slot;
	int ret;
	
	mmc = mmc_alloc_host(sizeof(struct s5p_mci_slot), host->dev);
	if (!mmc) {
		printk("mmc_alloc_host is fail \n");
		return -ENOMEM;	
	}
	slot = mmc_priv(mmc);
	slot->id = id;
	slot->sdio_id = host->sdio_id0 + id;
	slot->mmc = mmc;
	slot->host = host;//回指host
	host->slot[id] = slot;
	
	mmc->ops = &s5p_mci_ops;//设置mmc操作集合
	mmc->f_min = DW_MCI_FREQ_MIN;
	mmc->f_max = DW_MCI_FREQ_MAX;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_ERASE | MMC_CAP_CMD23 | MMC_CAP_HW_RESET;
	
	mmc->caps2 |= MMC_CAP2_NO_WRITE_PROTECT; //意思是没有物理写保护引脚
	mmc->caps |= MMC_CAP_4_BIT_DATA; /*总线宽度 4 */
	
	ret = mmc_of_parse(mmc); //使用核心层函数解析设备树
	if (ret) {
		printk("mmc_of_parse is fail \n");
	}
	
	mmc->max_segs = host->ring_size;//256
	mmc->max_blk_size = 65536;
	mmc->max_seg_size = 0x1000;//1页
	/* 1MB = 256个描述符 * 8 (1个描述符8个扇区) */
	mmc->max_req_size = mmc->max_seg_size * host->ring_size;//一个请求最大1MB数据
	mmc->max_blk_count = mmc->max_req_size / 512;//一个请求最多2048个块 
	
	if (s5p_mci_get_cd(mmc)) //卡插拔状态,拔出返回0
		set_bit(DW_MMC_CARD_PRESENT, &slot->flags);//插入设置位
	else
		clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);//拔出清除位
	
	ret = mmc_add_host(mmc);//注册一个MMC host
	if (ret) {
		printk("mmc_add_host is fail\n");	
	}
	
	/* 设备树含 cd-type-external 属性表示使用外部cd */
	
	return 0;
}





int s5p_mci_nexell_probe(struct platform_device *pdev)
{
	struct s5p_mci *host;
	struct resource	*regs;
	struct	reset_control *rst;
	int width, i, ret = 0;
	u32 fifo_size;
	int init_slots = 0;
	
	host = devm_kzalloc(&pdev->dev, sizeof(struct s5p_mci), GFP_KERNEL);
	if (!host) {
		printk("devm_kzalloc is fail \n");
		return -ENOMEM;
	}
	
	host->irq = platform_get_irq(pdev, 0);//获取中断
	if (host->irq < 0) {
		printk("get irq is fail \n");
		return host->irq;
	}
	
	host->dev = &pdev->dev;
	
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(host->regs)) {
		printk("devm_ioremap_resource is fail \n");
		return PTR_ERR(host->regs);
	}

	host->phy_regs = regs->start;

	host->biu_clk = devm_clk_get(host->dev, "biu");
	if (IS_ERR(host->biu_clk)) {
		dev_err(host->dev, "biu clock not available\n");/*无法获取时钟*/
	} else {
		ret = clk_prepare_enable(host->biu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable biu clock\n");/*无法使能时钟*/
			return ret;
		}
	}

	host->ciu_clk = devm_clk_get(host->dev, "ciu");//ciu 时钟获取
	if (IS_ERR(host->ciu_clk)) {
		dev_dbg(host->dev, "ciu clock not available\n");
		host->bus_hz = 100000000;
	} else {
		ret = clk_prepare_enable(host->ciu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable ciu clock\n");
			return ret;
		}
	}

	ret = clk_set_rate(host->ciu_clk, 100000000);
	host->bus_hz = clk_get_rate(host->ciu_clk);
	
	rst = devm_reset_control_get(host->dev, "dw_mmc-reset"); /*获取复位*/
	if (reset_control_status(rst))
		reset_control_reset(rst); /*完成IP复位*/
	
	mci_writel(host, CLKCTRL, 0x1020000); /*设置时钟偏移*/
	host->bus_hz /= 2; /*50MHZ*/

	/*初始化三个软定时器*/
	setup_timer(&host->cmd11_timer,
		    s5p_mci_cmd11_timer, (unsigned long)host);

	setup_timer(&host->cto_timer,
		    s5p_mci_cto_timer, (unsigned long)host);//命令超时

	setup_timer(&host->dto_timer,
		    s5p_mci_dto_timer, (unsigned long)host);//数据超时

	spin_lock_init(&host->lock);
	spin_lock_init(&host->irq_lock);

	width = 32;	/*数据寄存器宽度 32位*/
	host->data_shift = 2;/*1<<2 = 4 bytes */

	if (!s5p_mci_ctrl_reset(host, SDMMC_CTRL_ALL_RESET_FLAGS))
		return -ENODEV;
	
	s5p_mci_init_dma(host); //初始化IDMAC
	
	/* Clear the interrupts for the host controller 清除主控器的中断寄存器*/
	mci_writel(host, RINTSTS, 0xFFFFFFFF);//中断状态寄存器清除
	mci_writel(host, INTMASK, 0); /* disable all mmc interrupt first 首先禁用所有mmc中断*/

	/* Put in max timeout */
	mci_writel(host, TMOUT, 0xFFFFFFFF);//设置MMC超时计时为最大值	
	
	fifo_size = mci_readl(host, FIFOTH);
	fifo_size = 1 + ((fifo_size >> 16) & 0xfff);//SD=16  SDIO=32
	host->fifo_depth = fifo_size;
	host->fifoth_val =	//设置收发FIFO阈值和DMA突发字节数
		SDMMC_SET_FIFOTH(0x2, fifo_size / 2 - 1, fifo_size / 2);
	
	mci_writel(host, FIFOTH, host->fifoth_val);

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);//关闭CIU时钟

	host->verid = SDMMC_GET_VERID(mci_readl(host, VERID));
	host->fifo_reg = host->regs + DATA_240A_OFFSET;
	
	
	tasklet_init(&host->tasklet, s5p_mci_tasklet_func, (unsigned long)host);
	ret = devm_request_irq(host->dev, host->irq, s5p_mci_interrupt,
			       host->irq_flags, "dw-mci", host);	

	host->num_slots = SDMMC_GET_SLOT_NUM(mci_readl(host, HCON));

	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER |
		   SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS);//使能命令完成中断 数据溢出 发送就绪 接收就绪 错误中断
		   
	/* Enable mci interrupt */
	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE);//打开全局中断控制位
	

	dev_info(host->dev,
		 "DW MMC controller at irq %d,%d bit host data width,%u deep fifo\n",
		 host->irq, width, fifo_size);

	for (i = 0; i < host->num_slots; i++) { //初始化MMC HOST 并注册 
		
		ret = s5p_mci_init_slot(host, i);//同一个MMC0下每个插槽执行一次;理论只有1个插槽
		if (ret)
			dev_dbg(host->dev, "slot %d init failed\n", i);
		else
			init_slots++;
	}
	
	return ret;
	
}


int s5p_mci_nexell_remove(struct platform_device *pdev)
{
	return 0;
}



static const struct of_device_id s5p_mci_nexell_match[] = {
	{ .compatible = "nexell,s5p6818-dw-mshc" }, //匹配列表
	{},
};

MODULE_DEVICE_TABLE(of, s5p_mci_nexell_match);


static struct platform_driver s5p_mci_nexell_driver = {
	.probe		= s5p_mci_nexell_probe,
	.remove		= s5p_mci_nexell_remove,
	.driver		= {
		.name		= "dwmmc_nexell",
		.of_match_table	= s5p_mci_nexell_match,
	},
};


static int __init s5p_mci_init(void)
{
	return platform_driver_register(&s5p_mci_nexell_driver);
	
}


static void __exit s5p_mci_exit(void)
{
	platform_driver_unregister(&s5p_mci_nexell_driver);
	
}

module_init(s5p_mci_init);
module_exit(s5p_mci_exit);

MODULE_DESCRIPTION("S5P Multimedia Card Interface driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_AUTHOR("Imagination Technologies Ltd");
MODULE_LICENSE("GPL v2");
