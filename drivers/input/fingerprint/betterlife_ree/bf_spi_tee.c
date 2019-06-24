/*
 * Copyright (C) 2016 BetterLife.Co.Ltd. All rights  reserved.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/miscdevice.h>


#include <linux/signal.h>
#include <linux/ctype.h>
#include <linux/wakelock.h>
#include <linux/kobject.h>
#include <linux/poll.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/of_gpio.h>



/* MTK header */
//#include <mach/irqs.h>
#include <mach/emi_mpu.h>
//#include <mach/mtk_clkmgr.h>
#include <mtk_chip.h>
#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include "bf_spi_tee.h"
#if defined(PLATFORM_MT6739)
#include <linux/platform_data/spi-mt65xx.h>
#endif
//#include "../../../misc/mediatek/tkcore/include/linux/tee_fp.h"

#define BF_IOCTL_MAGIC_NO			0xFC

#define BF_IOCTL_INIT_ARGS                  _IOWR(BF_IOCTL_MAGIC_NO, 0,uint32_t)
#define BF_IOCTL_REGISTER_READ_WRITE        _IOWR(BF_IOCTL_MAGIC_NO,  1, uint32_t)
#define BF_IOCTL_RESET                      _IO(BF_IOCTL_MAGIC_NO,  2)
#define BF_IOCTL_DISABLE_INTERRUPT          _IO(BF_IOCTL_MAGIC_NO,  3)
#define BF_IOCTL_GAIN_ADJUST                _IOWR(BF_IOCTL_MAGIC_NO, 4,uint32_t)
#define BF_IOCTL_ENABLE_POWER               _IO(BF_IOCTL_MAGIC_NO, 5)
#define BF_IOCTL_DISABLE_POWER              _IO(BF_IOCTL_MAGIC_NO, 6)
#define BF_IOCTL_ENABLE_SPI_CLOCK           _IOW(BF_IOCTL_MAGIC_NO,  7,uint32_t)
#define BF_IOCTL_DISABLE_SPI_CLOCK          _IOW(BF_IOCTL_MAGIC_NO,  8,uint32_t)
#define BF_IOCTL_GET_ID                     _IOWR(BF_IOCTL_MAGIC_NO, 9, uint32_t)
#define BF_IOCTL_INIT_DEVICE                _IOW(BF_IOCTL_MAGIC_NO,  10,uint32_t)
#define BF_IOCTL_REMOVE_DEVICE              _IOW(BF_IOCTL_MAGIC_NO,  11,uint32_t)
#define BF_IOCTL_INPUT_KEY                  _IOW(BF_IOCTL_MAGIC_NO,  12,uint32_t)
#define BF_IOCTL_ENBACKLIGHT                _IOW(BF_IOCTL_MAGIC_NO,  13,uint32_t)
#define BF_IOCTL_ISBACKLIGHT                _IOWR(BF_IOCTL_MAGIC_NO, 14,uint32_t)
#define BF_IOCTL_DISPALY_STATUS             _IOW(BF_IOCTL_MAGIC_NO,  15,uint32_t)
#define BF_IOCTL_SET_PID                    _IOW(BF_IOCTL_MAGIC_NO,  16,uint32_t)
#define BF_IOCTL_INPUT_KEY_DOWN             _IOW(BF_IOCTL_MAGIC_NO,  17,uint32_t)
#define BF_IOCTL_INPUT_KEY_UP               _IOW(BF_IOCTL_MAGIC_NO,  18,uint32_t)
#define BF_IOCTL_LOW_RESET                  _IO(BF_IOCTL_MAGIC_NO,  19)
#define BF_IOCTL_HIGH_RESET                 _IO(BF_IOCTL_MAGIC_NO,  20)
#define BF_IOCTL_NETLINK_INIT               _IOW(BF_IOCTL_MAGIC_NO,  21,uint32_t)
#define BF_IOCTL_NETLINK_PORT               _IOWR(BF_IOCTL_MAGIC_NO, 22,uint32_t)
#define BF_IOCTL_ENABLE_INTERRUPT           _IO(BF_IOCTL_MAGIC_NO,  23)
#define BF_IOCTL_RESET_FLAG                 _IOW(BF_IOCTL_MAGIC_NO,  24,uint32_t)
#define BF_IOCTL_IS_OPT_POWER_ON2V8       	_IOWR(BF_IOCTL_MAGIC_NO,  25, uint32_t)

int key_event_down = 0;

static DEFINE_MUTEX(spi_lock);
static DEFINE_MUTEX(irq_count_lock);
static struct wake_lock fp_suspend_lock;
static struct wake_lock hw_reset_lock;
static int g_netlink_port = NETLINK_BF;

typedef enum bf_key {
    BF_KEY_NONE = 0,
    BF_KEY_POWER,
    BF_KEY_CAMERA,
    BF_KEY_UP,
    BF_KEY_DOWN,
    BF_KEY_RIGHT,
    BF_KEY_LEFT,
    BF_KEY_HOME,
    BF_KEY_F10,
	BF_KEY_F11
} bf_key_t;


#if defined(CONFIG_FB)
static struct notifier_block fb_notif;
static volatile int display_status = 0;
#endif

#ifdef FAST_VERSION
extern int g_bl229x_enbacklight;
#endif
int g_bl229x_enbacklight_tmp = 0;

extern void mt_spi_enable_master_clk(struct spi_device *ms);
extern void mt_spi_disable_master_clk(struct spi_device *ms);

static LIST_HEAD (device_list);
static DEFINE_MUTEX (device_list_lock);
/* for netlink use */
static int g_pid;
static struct bf_device *g_bf_dev=NULL;
static struct input_dev *bf_inputdev = NULL;
static uint32_t bf_key_need_report = 0;


/*
static struct madev_chip_conf spi_init_conf = {

.setuptime = 10,
	.holdtime = 10,
	.high_time = 8,
	.low_time =  8,
	.cs_idletime = 20, //10,
	//.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.tx_endian = 0,
	.rx_endian = 0,

	.com_mod = 4,//FIFO_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,

};

*/

static struct mtk_chip_config spi_init_conf = {
    .cs_pol = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .sample_sel = 0,
};



int spi_set_dma_en(int mode)
{
#if defined(PLATFORM_MTK)
    struct mt_chip_conf* spi_par;
    spi_par = &spi_init_conf;
    if (!spi_par) {
        return -1;
    }
    if (1 == mode) {
        if (spi_par->com_mod == DMA_TRANSFER) {
            return 0;
        }
        spi_par->com_mod = DMA_TRANSFER;
    } else {
        if (spi_par->com_mod == FIFO_TRANSFER) {
            return 0;
        }
        spi_par->com_mod = FIFO_TRANSFER;
    }
    spi_setup(g_bf_dev->spi);
#endif
    return 0;
}



int spi_send_cmd(struct bf_device *bf_dev, u8 *tx, u8 *rx, u16 spilen)
{
#ifdef TEE_TK
    extern int tee_spi_transfer(void *conf, uint32_t conf_size, void *inbuf, void *outbuf, uint32_t size);
#ifdef PLATFORM_MT6739
    int ret = tee_spi_transfer(&spi_conf_tk, sizeof(spi_conf_tk), tx, rx, spilen);
#else
    int ret = tee_spi_transfer(&spi_init_conf, sizeof(spi_init_conf), tx, rx, spilen);
#endif

#else
    int ret = 0;
    struct spi_message m;
    struct spi_transfer t = {
        .tx_buf = tx,
        .rx_buf = rx,
        .len = spilen,
#ifdef USE_SPI1_4GB_TEST
        .tx_dma = SpiDmaBufTx_pa,
        .rx_dma = SpiDmaBufRx_pa,
#else
        .tx_dma = 0,
        .rx_dma = 0,
#endif
    };
#ifndef KERNEL_4_9
    mutex_lock(&spi_lock);
#endif
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    ret = spi_sync(bf_dev->spi, &m);
    
#ifndef KERNEL_4_9
    mutex_unlock(&spi_lock);
#endif
#endif
    return ret;
}



int bf_read_chipid(void)
{
    int id=0;
    uint8_t id_low=0,id_high=0;
    uint8_t version=0;
    uint8_t type=0;

    id_high = bf_spi_read_reg(0x37);
    id_low = bf_spi_read_reg(0x36);
    id = (id_high << 8)|id_low;
    version = bf_spi_read_reg(0x38);//0
    type = bf_spi_read_reg(0x3e);    ////3

    BF_LOG(" BTL:3582P---- chipid=0x%x,version=0x%x,type=0x%x \n",
    id,version,type);

    return id;

}
uint8_t bf_spi_write_reg(uint8_t reg, uint8_t value)
{
     u8 nAddr;
    u8 data_tx[4] = {0};
    u8 data_rx[4] = {0};

    nAddr = reg << 1;
    nAddr |= 0x80;

    data_tx[0] = nAddr;
    data_tx[1] = value;

    spi_send_cmd(g_bf_dev, data_tx, data_rx, 2);
    return data_rx[1];
}
uint8_t bf_spi_read_reg(uint8_t reg)
{
     u8 nAddr;
    u8 data_tx[4] = {0};
    u8 data_rx[4] = {0};

    nAddr = reg << 1;
    nAddr &= 0x7F;

    data_tx[0] = nAddr;
    data_tx[1] = 0xff;

    spi_send_cmd(g_bf_dev, data_tx, data_rx, 2);
    return data_rx[1];
}
#if defined(CONFIG_FB)
/*----------------------------------------------------------------------------*/
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank =  evdata->data;

    BF_LOG("%s fb notifier callback event = %lu, evdata->data = %d\n",__func__, event, *blank);
    if (evdata && evdata->data){
	    if (event == FB_EVENT_BLANK ){		  
            if (*blank == FB_BLANK_UNBLANK){
                g_bf_dev->need_report = 0;
			    display_status = 1;
		    }
            else if (*blank == FB_BLANK_POWERDOWN){
                g_bf_dev->need_report = 1;
			    display_status = 0;
		    }
		}
    }	
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int bf_hw_power (struct bf_device *bf_dev, bool enable)
{
#ifdef NEED_OPT_POWER_ON
#ifdef CONFIG_OF
	if (enable) {
		pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_power_high);
	//	pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_power_1v8_high);
	} 
	else {
		pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_power_low);
	//	pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_power_1v8_low);
	}
#else
	if (enable) {
		gpio_direction_output (bf_dev->power_en_gpio, 1);
	} 
	else {
		gpio_direction_output (bf_dev->power_en_gpio, 0);
	}
#endif
#endif
	return 0;
}

static int bf_hw_reset(struct bf_device *bf_dev)
{
#ifdef CONFIG_OF
	pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_reset_low);
	mdelay(5);
	pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_reset_high);
#else
	gpio_direction_output (bf_dev->reset_gpio, 0);
	mdelay(5);
	gpio_direction_output (bf_dev->reset_gpio, 1);
#endif
	return 0;
}

static int bf_hw_reset_level (struct bf_device *bf_dev, bool enable)
{
    BF_LOG("bf_hw_reset_level %d", enable);

    if (enable) {
        pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_reset_high);
    } else {
        pinctrl_select_state (bf_dev->pinctrl_gpios, bf_dev->pins_reset_low);
    }

    return 0;
}


static void bf_enable_irq(struct bf_device *bf_dev)
{
	if (1 == bf_dev->irq_count) {
		BF_LOG("irq already enabled\n");
	} else {
		enable_irq_wake(bf_dev->irq_num);
		bf_dev->irq_count = 1;
		BF_LOG(" enable interrupt!\n");
	}
}

static void bf_disable_irq(struct bf_device *bf_dev)
{
	if (0 == bf_dev->irq_count) {
		BF_LOG(" irq already disabled\n");
	} else {
		disable_irq(bf_dev->irq_num);
		bf_dev->irq_count = 0;
		BF_LOG(" disable interrupt!\n");
	}
}

#if defined (PLATFORM_MT6739)
struct mtk_spi {
    void __iomem *base;
    void __iomem *peri_regs;
    u32 state;
    int pad_num;
    u32 *pad_sel;
    struct clk *parent_clk, *sel_clk, *spi_clk;
    struct spi_transfer *cur_transfer;
    u32 xfer_len;
    struct scatterlist *tx_sgl, *rx_sgl;
    u32 tx_sgl_len, rx_sgl_len;
    const struct mtk_spi_compatible *dev_comp;
    u32 dram_8gb_offset;
};
#endif
static void bf_spi_clk_enable(struct bf_device *bf_dev, u8 bonoff)
{
	  static int count = 0;
#if defined(PLATFORM_MT6739)
    struct mtk_spi *mdata = NULL;
    int ret = -1;
    mdata = spi_master_get_devdata(bf_dev->spi->master);

    if (bonoff && (count == 0)) {
        ret = clk_prepare_enable(mdata->spi_clk);
        if (ret < 0) {
            BF_LOG("failed to enable spi_clk (%d)\n", ret);
        } else
            count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        clk_disable_unprepare(mdata->spi_clk);
        count = 0;
    }
#else
	static int count;


	if (bonoff && (count == 0)) {
		mt_spi_enable_master_clk(bf_dev->spi);
		count = 1;
	} else if ((count > 0) && (bonoff == 0)) {
		mt_spi_disable_master_clk(bf_dev->spi);
		count = 0;
	}
#endif
}


/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration								           */
/* -------------------------------------------------------------------- */


static int bf_get_gpio_info_from_dts (struct bf_device *bf_dev)
{
#ifdef CONFIG_OF
	int ret;
/*
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	//node = of_find_compatible_node (NULL, NULL, "mediatek,betterlife-fp");
        node = of_find_compatible_node (NULL, NULL, "mediatek,BL229X");*/
#if 1
/*	if (node) {
		pdev = of_find_device_by_node (node);
		if (pdev) {
			bf_dev->pinctrl_gpios = devm_pinctrl_get (&pdev->dev);
			if (IS_ERR (bf_dev->pinctrl_gpios)) {
				ret = PTR_ERR (bf_dev->pinctrl_gpios);
				BF_LOG( "can't find fingerprint pinctrl");
				return ret;
			}
		} else {
			BF_LOG( "platform device is null");
		}
	} else {
		BF_LOG( "device node is null");

	}
*/
	bf_dev->pinctrl_gpios = devm_pinctrl_get (&bf_dev->spi->dev);
	/* it's normal that get "default" will failed */
	bf_dev->pins_default = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "spi0_default");
	if (IS_ERR (bf_dev->pins_default)) {
		ret = PTR_ERR (bf_dev->pins_default);
		BF_LOG( "can't find fingerprint pinctrl default");
		/* return ret; */
	}
	bf_dev->pins_reset_high = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "rst_output1");
	if (IS_ERR (bf_dev->pins_reset_high)) {
		ret = PTR_ERR (bf_dev->pins_reset_high);
		BF_LOG( "can't find fingerprint pinctrl pins_reset_high");
		return ret;
	}
	bf_dev->pins_reset_low = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "rst_output0");
	if (IS_ERR (bf_dev->pins_reset_low)) {
		ret = PTR_ERR (bf_dev->pins_reset_low);
		BF_LOG( "can't find fingerprint pinctrl reset_low");
		return ret;
	}
	bf_dev->pins_fp_interrupt = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "int_default");
	if (IS_ERR (bf_dev->pins_fp_interrupt)) {
		ret = PTR_ERR (bf_dev->pins_fp_interrupt);
		BF_LOG( "can't find fingerprint pinctrl fp_interrupt");
		return ret;
	}

#ifdef NEED_OPT_POWER_ON
	bf_dev->pins_power_high = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "power_en_output1");
	if (IS_ERR (bf_dev->pins_power_high)) {
		ret = PTR_ERR (bf_dev->pins_power_high);
		BF_LOG ("can't find fingerprint pinctrl power_high");
		return ret;
	}
	bf_dev->pins_power_low = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "power_en_output0");
	if (IS_ERR (bf_dev->pins_power_low)) {
		ret = PTR_ERR (bf_dev->pins_power_low);
		BF_LOG ("can't find fingerprint pinctrl power_low");
		return ret;
	}
/*
	bf_dev->pins_power_1v8_high = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "power_en1_output1");
	if (IS_ERR (bf_dev->pins_power_1v8_high)) {
		ret = PTR_ERR (bf_dev->pins_power_1v8_high);
		BF_LOG ("can't find fingerprint pinctrl pins_power_1v8_high");
		return ret;
	}
	bf_dev->pins_power_1v8_low = pinctrl_lookup_state (bf_dev->pinctrl_gpios, "power_en1_output0");
	if (IS_ERR (bf_dev->pins_power_1v8_low)) {
		ret = PTR_ERR (bf_dev->pins_power_1v8_low);
		BF_LOG ("can't find fingerprint pinctrl power_1v8_low");
		return ret;
	}
*/
#endif
	BF_LOG( "get pinctrl success!");
#endif 

	bf_dev->reset_gpio = of_get_named_gpio(bf_dev->spi->dev.of_node, "fingerprint,rst-gpio", 0);
    BF_LOG("blest reset_gpip_num= %d\n",bf_dev->reset_gpio);
//	bf_dev->irq_gpio =  of_get_named_gpio(bf_dev->spi->dev.of_node, "int_gpio", 0);
//    BF_LOG("blest ire_gpio_num= %d\n",bf_dev->irq_gpio);
//	bf_dev->irq_num = irq_of_parse_and_map(bf_dev->spi->dev.of_node, 0);
//	BF_LOG("blest irq_num= %d\n",bf_dev->irq_num);
    bf_dev->irq_gpio = of_get_named_gpio_flags(bf_dev->spi->dev.of_node,
                        "fingerprint,int-gpio", 0, NULL);
    if (!gpio_is_valid(bf_dev->irq_gpio)){
        BF_LOG("invalid irq gpio!");
//        return -EINVAL;
    }
    bf_dev->irq_num = gpio_to_irq(bf_dev->irq_gpio);


#ifdef NEED_OPT_POWER_ON
	bf_dev->power_en_gpio = of_get_named_gpio(bf_dev->spi->dev.of_node, "fingerprint,en-gpio", 0);
//	bf_dev->power1v8_en_gpio = of_get_named_gpio(bf_dev->spi->dev.of_node, "en1v8-gpio", 0);
    BF_LOG("blest en_num= %d\n",bf_dev->power_en_gpio);
  //  BF_LOG("blest en1.8_num= %d\n",bf_dev->power1v8_en_gpio);


#endif


#endif
	return 0;

}


/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
void bf_send_netlink_msg(struct bf_device *bf_dev, const int command)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret;
	char data_buffer[2];

	BF_LOG("enter, send command %d",command);
	memset(data_buffer,0,2);
	data_buffer[0] = (char)command;
	if (NULL == bf_dev->netlink_socket) {
		BF_LOG("invalid socket");
		return;
	}

	if (0 == g_pid) {
		BF_LOG("invalid native process pid");
		return;
	}

	/*alloc data buffer for sending to native*/
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL) {
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		BF_LOG("nlmsg_put failed");
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	*((char *)NLMSG_DATA(nlh)+1) = 0; 
	ret = netlink_unicast(bf_dev->netlink_socket, skb, g_pid, MSG_DONTWAIT);
	if (ret < 0) {
		BF_LOG("send failed");
		return;
	}

	BF_LOG("send done, data length is %d",ret);
	return ;
}

static void bf_recv_netlink_msg(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];


	skb = skb_get(__skb);
	if (skb == NULL) {
		BF_LOG("skb_get return NULL");
		return;
	}

	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		//memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		g_pid = nlh->nlmsg_pid;
		BF_LOG("pid: %d, msg: %s",g_pid, str);

	} else {
		BF_LOG("not enough data length");
	}

	kfree_skb(__skb);

}


static int bf_close_netlink(struct bf_device *bf_dev)
{
	if (bf_dev->netlink_socket != NULL) {
		netlink_kernel_release(bf_dev->netlink_socket);
		bf_dev->netlink_socket = NULL;
		return 0;
	}

	BF_LOG("no netlink socket yet");
	return -1;
}


static int bf_init_netlink(struct bf_device *bf_dev)
{
	struct netlink_kernel_cfg cfg;

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = bf_recv_netlink_msg;

	bf_dev->netlink_socket = netlink_kernel_create(&init_net, NETLINK_BF, &cfg);
	if (bf_dev->netlink_socket == NULL) {
		BF_LOG("netlink create failed");
		return -1;
	}
	BF_LOG("netlink create success");
	return 0;
}


static irqreturn_t bf_eint_handler (int irq, void *data)
{
    struct bf_device *bf_dev = (struct bf_device *)data;
	BF_LOG("++++irq_handler netlink send+++++");
	bf_send_netlink_msg(bf_dev, BF_NETLINK_CMD_IRQ);
	bf_dev->sig_count++;
/*#ifdef	FAST_VERSION
	if (g_bf_dev->report_key!=0 && g_bf_dev->need_report!=0){
		//g_bf_dev->need_report = 0;

		input_report_key(bf_inputdev,g_bf_dev->report_key ,1);
    	input_sync(bf_inputdev);

		input_report_key(bf_inputdev,g_bf_dev->report_key ,0);
    	input_sync(bf_inputdev);
		BF_LOG("report power key %d\n",g_bf_dev->report_key );
   	}
#endif	*/
	BF_LOG("-----irq_handler netlink bf_dev->sig_count=%d-----",bf_dev->sig_count);
    return IRQ_HANDLED;
}

/* -------------------------------------------------------------------- */
/* file operation function                                                                                */
/* -------------------------------------------------------------------- */

#ifdef FAST_VERSION
int g_bl229x_enbacklight = 1;
#endif

static long bf_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct bf_device *bf_dev = NULL;
	int error = 0;
#ifdef FAST_VERSION
	u32 bl229x_enbacklight = 0;
#endif 
 	int dma_size = 0;
	u32 chipid = 0x5283;
	bf_key_t bf_input_key = BF_KEY_NONE;
	unsigned int key_event = 0;
	int display_status_tmp = 0;
	bl_read_write_reg_command_t read_write_cmd;

	bf_dev = (struct bf_device *)filp->private_data;

	if (_IOC_TYPE(cmd) != BF_IOCTL_MAGIC_NO){
		BF_LOG("Not blestech fingerprint cmd.");
		return -EINVAL;
	}
	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

	if (error == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		error = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (error){
		BF_LOG("Not blestech fingerprint cmd direction.");
		return -EINVAL;
	}


	switch (cmd) {
    case BF_IOCTL_RESET:
        BF_LOG("BF_IOCTL_RESET: chip reset command\n");
			bf_hw_reset(bf_dev);
			break;

    case BF_IOCTL_ENABLE_INTERRUPT:
        BF_LOG("BF_IOCTL_ENABLE_INTERRUPT:  command,%d\n", bf_dev->irq_count);
        mutex_lock(&irq_count_lock);
        bf_dev->irq_count = 1;
        mutex_unlock(&irq_count_lock);
			break;

    case BF_IOCTL_DISABLE_INTERRUPT:

        BF_LOG("BF_IOCTL_DISABLE_INTERRUPT:  command,%d\n", bf_dev->irq_count);
        mutex_lock(&irq_count_lock);
        bf_dev->irq_count = 0;
        mutex_unlock(&irq_count_lock);
			break;

    case BF_IOCTL_ENABLE_POWER:
        BF_LOG("BF_IOCTL_ENABLE_POWER:  command\n");
			bf_hw_power(bf_dev,1);
			break;

    case BF_IOCTL_DISABLE_POWER:
        BF_LOG("BF_IOCTL_DISABLE_POWER:  command\n");
			bf_hw_power(bf_dev,0);
			break;
    case BF_IOCTL_INPUT_KEY:
        key_event = (unsigned int)arg;
        BF_LOG("key:%d\n", key_event);

        input_report_key(bf_inputdev, key_event, 1);
        input_sync(bf_inputdev);
        input_report_key(bf_inputdev, key_event, 0);
        input_sync(bf_inputdev);
        break;
#ifdef FAST_VERSION			
    case BF_IOCTL_ENBACKLIGHT:
        BF_LOG("BF_IOCTL_ENBACKLIGHT arg:%d\n", (int)arg);
			g_bl229x_enbacklight = (int)arg;
			break;
    case BF_IOCTL_ISBACKLIGHT:
        BF_LOG("BF_IOCTL_ISBACKLIGHT\n");
			bl229x_enbacklight = g_bl229x_enbacklight;
			if (copy_to_user((void __user*)arg,&bl229x_enbacklight,sizeof(u32)*1) != 0 ){
			   error = -EFAULT;
			}
			break;
			
    case BF_IOCTL_GET_ID:
			if (copy_to_user((void __user*)arg,&chipid,sizeof(u32)*1) != 0 ){
			   error = -EFAULT;
			}
			break;
#endif
    case BF_IOCTL_REGISTER_READ_WRITE:
        BF_LOG("BTL:BF_IOCTL_REGISTER_READ_WRITE\n");

        if(copy_from_user(&read_write_cmd, (bl_read_write_reg_command_t *)arg, sizeof(read_write_cmd))) {
            error = -EFAULT;
            BF_LOG("BF_IOCTL_REGISTER_READ_WRITE copy_to_user faile!");
            break;
        }

        if(read_write_cmd.len < 32) {
            spi_send_cmd(g_bf_dev, read_write_cmd.data_tx, read_write_cmd.data_rx, read_write_cmd.len);
            if (copy_to_user((void __user*)arg, &read_write_cmd, sizeof(bl_read_write_reg_command_t)) != 0 ) {
                BF_LOG("BF_IOCTL_REGISTER_READ_WRITE copy_to_user faile!");
                error = -EFAULT;
                break;
            }
        } else {
            dma_size = ((read_write_cmd.len / 1024) + 1) * 1024;
            memset(g_bf_dev->image_buf, 0xff, get_order(dma_size));
            spi_set_dma_en(1);
#ifdef USE_SPI1_4GB_TEST
            spi_dma_exchange(g_bf_dev, read_write_cmd, dma_size);
#else
            spi_send_cmd(g_bf_dev, read_write_cmd.data_tx, g_bf_dev->image_buf, dma_size);
#endif
	BF_LOG("lqw dma_size = %d \n", dma_size);
            spi_set_dma_en(0);
        }
        break;

#ifdef CONFIG_MTK_CLK
    case BF_IOCTL_ENABLE_SPI_CLOCK:
        BF_LOG("BF_IOCTL_ENABLE_SPI_CLK:  command\n");
        bf_spi_clk_enable(bf_dev, 1);
        break;
    case BF_IOCTL_DISABLE_SPI_CLOCK:
        BF_LOG("BF_IOCTL_DISABLE_SPI_CLK:  command\n");
        bf_spi_clk_enable(bf_dev, 0);
        break;
#endif
    case BF_IOCTL_INPUT_KEY_DOWN:
			//input_report_key(bf_inputdev, KEY_HOMEPAGE, 1);
			//input_sync(bf_inputdev);
#ifdef FAST_VERSION
			if(g_bl229x_enbacklight && g_bf_dev->need_report==0){
#else
			if(g_bf_dev->need_report==0){
#endif
				bf_key_need_report = 1;
				key_event = (int)arg;
				input_report_key(bf_inputdev, key_event, 1);
				input_sync(bf_inputdev);			
			}			
			break;
    case BF_IOCTL_INPUT_KEY_UP:
			//input_report_key(bf_inputdev, KEY_HOMEPAGE, 0);
			//input_sync(bf_inputdev);
			if(bf_key_need_report == 1){
				bf_key_need_report = 0;
				key_event = (int)arg;
				input_report_key(bf_inputdev, key_event, 0);
				input_sync(bf_inputdev);
			}			
			break;

    case BF_IOCTL_LOW_RESET:
        BF_LOG("BF_IOCTL_LOW_RESET:  command\n");
        wake_lock_timeout(&hw_reset_lock, 2 * HZ);

        bf_hw_reset_level(g_bf_dev, 0);
        break;
    case BF_IOCTL_HIGH_RESET:
        BF_LOG("BF_IOCTL_HIGH_RESET:  command\n");
        bf_hw_reset_level(g_bf_dev, 1);
        break;
    case BF_IOCTL_NETLINK_INIT:
        BF_LOG("BF_IOCTL_NETLINK_INIT:  command\n");
        g_netlink_port = (int)arg;
        bf_close_netlink(g_bf_dev);
        error = bf_init_netlink(g_bf_dev);
        if (error < 0) {
            BF_LOG("BF_IOCTL_NETLINK_INIT:  error\n");
			}
        break;
    case BF_IOCTL_NETLINK_PORT:
        BF_LOG("BF_IOCTL_NETLINK_PORT:  command\n");
        if (copy_to_user((void __user*)arg, &g_netlink_port, sizeof(u32) * 1) != 0 ) {
				error = -EFAULT;
			}
			//display_status = 0;//reset display_status
			break;		
    case BF_IOCTL_INIT_DEVICE:
        BF_LOG("BF_IOCTL_INIT_DEVICE:  command\n");
        error = bf_init_dts_and_irq(g_bf_dev);
        break;
    case BF_IOCTL_REMOVE_DEVICE:
        BF_LOG("BF_IOCTL_REMOVE_DEVICE:  command\n");
        bf_remove(g_bf_dev->spi);
        break;
    case BF_IOCTL_RESET_FLAG:
        bf_dev->doing_reset = (u8)arg;
        BF_LOG("BF_IOCTL_RESET_FLAG:  command,%d\n", bf_dev->doing_reset);
			break;
    case BF_IOCTL_IS_OPT_POWER_ON2V8:
        BF_LOG("BF_IOCTL_IS_OPT_POWER_ON2V8:  command\n");
#ifdef NEED_OPT_POWER_ON2V8
        key_event = 1;
#else
        key_event = 0;
#endif
        if (copy_to_user((void __user*)arg, &key_event, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }
        break;
			default:
			BF_LOG("Supportn't this command(%x)\n",cmd);
			break;
	}

	return error;

}
#ifdef CONFIG_COMPAT
static long bf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	retval = bf_ioctl(filp, cmd, arg);

	return retval;
}
#endif

/*----------------------------------------------------------------------------*/
static int bf_open (struct inode *inode, struct file *filp)
{
	struct bf_device *bf_dev = g_bf_dev;
	int status = 0;
	
	filp->private_data = bf_dev;
	BF_LOG( " Success to open device.");
	
	return status;
}


/* -------------------------------------------------------------------- */
static ssize_t bf_write (struct file *file, const char *buff, size_t count, loff_t *ppos)
{
	return -ENOMEM;
}

/* -------------------------------------------------------------------- */
static ssize_t bf_read (struct file *filp, char  *buff, size_t count, loff_t *ppos)
{
	ssize_t status = 0;
#ifdef BF_REE
    int ret = 0;
    //    spi_read_frame(struct  bf_device *bf_dev);

    ret = copy_to_user(buff, g_bf_dev->image_buf , count); //skip
    if (ret) {
        BF_LOG("copy_to_user failed\n");
        status = -EFAULT;
    }
#endif
	return status;
}

/* -------------------------------------------------------------------- */
static int bf_release (struct inode *inode, struct file *file)
{
	int status = 0 ;

	return status;
}

static int bf_suspend (struct device *dev)
{
    //struct bl229x_data *bl229x = dev_get_drvdata(dev);
    BF_LOG("  ++\n");
    //atomic_set(&suspended, 1);
	g_bf_dev->need_report = 1;
	BF_LOG("\n");
    return 0;
}

/* -------------------------------------------------------------------- */
static int bf_resume (struct device *dev)
{
    //struct bl229x_data *bl229x = dev_get_drvdata(dev);
    //dev_err (&bl229x->spi->dev,"[bl229x]%s\n", __func__);
    BF_LOG("  ++\n");
    //atomic_set(&suspended, 0);
    //wake_up_interruptible(&waiting_spi_prepare);
	BF_LOG("\n");
    return 0;
}
/*----------------------------------------------------------------------------*/
static const struct file_operations bf_fops = {
	.owner = THIS_MODULE,
	.open  = bf_open,
	.write = bf_write,
	.read  = bf_read,
	.release = bf_release,
	.unlocked_ioctl = bf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bf_compat_ioctl,
#endif
};

static struct miscdevice bf_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = BF_DEV_NAME,
    .fops = &bf_fops,
};

static int bf_remove(struct spi_device *spi)
{
	struct bf_device *bf_dev = g_bf_dev;

	/* make sure ops on existing fds can abort cleanly */
	if (bf_dev->irq_num) {
		free_irq(bf_dev->irq_num, bf_dev);
		bf_dev->irq_count = 0;
		bf_dev->irq_num= 0;
	}

	bf_close_netlink(bf_dev);
	
	bf_hw_power(bf_dev,0);
	bf_spi_clk_enable(bf_dev, 0);

	spi_set_drvdata(spi, NULL);
	bf_dev->spi = NULL;

	kfree(bf_dev);

	return 0;
}

static int bf_create_inputdev(void)
{
	bf_inputdev = input_allocate_device();
	if (!bf_inputdev) {
		BF_LOG("bf_inputdev create faile!\n");
		return -ENOMEM;
	}
	__set_bit(EV_KEY,bf_inputdev->evbit);
	__set_bit(KEY_F10,bf_inputdev->keybit);		//68
	__set_bit(KEY_F11,bf_inputdev->keybit);		//88
	__set_bit(KEY_F12,bf_inputdev->keybit);		//88
	__set_bit(KEY_CAMERA,bf_inputdev->keybit);	//212
	__set_bit(KEY_POWER,bf_inputdev->keybit);	//116
	__set_bit(KEY_PHONE,bf_inputdev->keybit);  //call 169
	__set_bit(KEY_BACK,bf_inputdev->keybit);  //call 158
	__set_bit(KEY_HOMEPAGE,bf_inputdev->keybit);  //call 172
	__set_bit(KEY_MENU,bf_inputdev->keybit);  //call 158

	__set_bit(KEY_F1,bf_inputdev->keybit);	//69
	__set_bit(KEY_F2,bf_inputdev->keybit);	//60
	__set_bit(KEY_F3,bf_inputdev->keybit);	//61
	__set_bit(KEY_F4,bf_inputdev->keybit);	//62
	__set_bit(KEY_F5,bf_inputdev->keybit);	//63
	__set_bit(KEY_F6,bf_inputdev->keybit);	//64
	__set_bit(KEY_F7,bf_inputdev->keybit);	//65
	__set_bit(KEY_F8,bf_inputdev->keybit);	//66
	__set_bit(KEY_F9,bf_inputdev->keybit);	//67

	__set_bit(KEY_UP,bf_inputdev->keybit);	//103
	__set_bit(KEY_DOWN,bf_inputdev->keybit);	//108
	__set_bit(KEY_LEFT,bf_inputdev->keybit);	//105
	__set_bit(KEY_RIGHT,bf_inputdev->keybit);	//106

	bf_inputdev->id.bustype = BUS_HOST;
	bf_inputdev->name = "bl229x_inputdev";
	if (input_register_device(bf_inputdev)) {
		BF_LOG("%s, register inputdev failed\n", __func__);
		input_free_device(bf_inputdev);
		return -ENOMEM;
	}
	return 0;
}

#ifdef VANZO_BEE_A03
extern int get_fp_vendor(void);
#endif

int bf_init_dts_and_irq(struct bf_device *bf_dev)
{
    int32_t status = -EINVAL;
    BF_LOG( "    ++++");
    status = bf_get_gpio_info_from_dts (bf_dev);
    if(status) {
        BF_LOG("bf_get_gpio_info_from_dts fail:%d", status);
        return -1;
    }

    status = request_threaded_irq (bf_dev->irq_num, NULL, bf_eint_handler,  IRQ_TYPE_EDGE_RISING /*IRQF_TRIGGER_RISING*/ | IRQF_ONESHOT, BF_DEV_NAME, bf_dev);


    if (status) {
        BF_LOG("irq thread request failed, retval=%d\n", status);
        return -3;
    }

	bf_hw_power(bf_dev, 1);
    bf_hw_reset(bf_dev);

    enable_irq_wake(bf_dev->irq_num);
    BF_LOG( "    ----");
    return 0;
}

static int32_t bf_platform_pinctrl_spi_init(struct bf_device *bf_dev)
{
    int32_t error = 0;


    error = pinctrl_select_state(bf_dev->pinctrl_gpios, bf_dev->pins_default);
    if (error) {
        dev_err(&bf_dev->spi, "failed to activate pins_spi_default state\n");
    }

    return error;
}

static int  bf_probe (struct spi_device *spi)
{
	int chip_id = 0;
	struct bf_device *bf_dev = NULL;
	int status = -EINVAL;
	
	bf_dev = kzalloc(sizeof (struct bf_device), GFP_KERNEL);
	if (NULL == bf_dev){
	   BF_LOG( "kzalloc bf_dev failed.");	
	   status = -ENOMEM;
	   goto err;
	}
    bf_dev->irq_count = 0;
    bf_dev->doing_reset = 0;
    bf_dev->report_key = KEY_F10;
	g_bf_dev=bf_dev;
	
	/* Initialize the driver data */
	BF_LOG( "bf config spi ");
	bf_dev->spi = spi;
	/* setup SPI parameters */
	bf_dev->spi->mode = SPI_MODE_0;
	bf_dev->spi->bits_per_word = 8;
	bf_dev->spi->max_speed_hz = 6 * 1000 * 1000;
	//memcpy (&bf_dev->mtk_spi_config, &spi_init_conf, sizeof (struct madev_chip_conf));
	//bf_dev->spi->controller_data = (void*)&bf_dev->mtk_spi_config;
        g_bf_dev->spi->controller_data = (void*)&spi_init_conf;
	spi_setup (bf_dev->spi);    
	spi_set_drvdata (spi, bf_dev);


//	bf_dev->spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,BL229X"); 
	bf_spi_clk_enable(bf_dev,1);



	BF_LOG("before misc_register\n");
#if defined(BF_REE)
    g_bf_dev->image_buf = (u8*)__get_free_pages(GFP_KERNEL, get_order(BUF_SIZE));
    if (!bf_dev->image_buf) {
        BF_LOG("\ng_bf_dev->image_buf __get_free_pages failed\n");
        status = -ENOMEM;
    }
#endif

	bf_get_gpio_info_from_dts(bf_dev);

	bf_hw_power(bf_dev,1);
	bf_hw_reset(bf_dev);
	msleep(200);	

	status = bf_platform_pinctrl_spi_init(bf_dev);
    if(status) {
        BF_LOG("bf_platform_pinctrl_init fail");
        bf_remove(g_bf_dev->spi);
        return status;
    }



	chip_id = bf_read_chipid();
	BF_LOG ("chip_type=0x%x \n", chip_id);
	if (chip_id != 0x5783) {
		BF_LOG("bf probe read chip id is failed\n");
       // goto err;
    }
	BF_LOG("blestech_fp add secussed.");

	status = request_threaded_irq (bf_dev->irq_num, NULL, bf_eint_handler, IRQ_TYPE_EDGE_RISING | IRQF_ONESHOT, BF_DEV_NAME, bf_dev);
	if (!status)
	   BF_LOG("irq thread request success!\n");
	else {
	   BF_LOG("irq thread request failed, retval=%d\n", status);
           goto err;
        }

	/* netlink interface init */
	BF_LOG ("bf netlink config");
	if (bf_init_netlink(bf_dev) < 0){
	    BF_LOG ("bf_netlink create failed");
            status = -1;
            goto err;
	}

	bf_dev->irq_count=0;
	bf_dev->sig_count=0;
	bf_dev->report_key = KEY_F10;
	
	bf_enable_irq(bf_dev);
	bf_create_inputdev();
	status = misc_register(&bf_misc_device);
        if(status) {
           BF_LOG("bl229x_misc_device register failed\n");
           goto err;
        }
	BF_LOG("blestech_fp add secussed.");

#if defined(CONFIG_FB)
	fb_notif.notifier_call = fb_notifier_callback;
	fb_register_client(&fb_notif);
#endif	
	
	BF_LOG ("---ok--");
	return 0;

err:	
	return status;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static struct of_device_id bf_of_table[] = {
	{.compatible = "mediatek,BL229X", },
  	{},
};
MODULE_DEVICE_TABLE(of,bf_of_table);
#endif 

static const struct dev_pm_ops bf_pm = {
    .suspend = bf_suspend,
    .resume =  bf_resume
};

static struct spi_driver bf_driver = {
	.driver = {
		.name	= BF_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bf_of_table,
#endif
		.pm = &bf_pm,
	},
	.probe	= bf_probe,
	.remove = bf_remove,
	//.id_table = 
};


/*----------------------------------------------------------------------------*/
static int bf_spi_init(void)
{
	int status = 0;

#ifdef VANZO_BEE_A03
	int blestech_id =0;

   	blestech_id = get_fp_vendor();
    	BF_LOG("%s blestech_id= %d !\n", __func__,blestech_id);
    	if(blestech_id != 0x3)
    	{
	    BF_LOG("%s() blestech_id= %d !\n", __func__,blestech_id);
       	    return -1;
    	}
#endif

printk(" bf_spi_init begin\n");
	status = spi_register_driver(&bf_driver);
	if (status < 0)
		BF_LOG( "Failed to register SPI driver.\n");
	else
		printk(" bf_spi_init ok\n");
	return status;
}
static void bf_spi_exit(void)
{
	spi_unregister_driver (&bf_driver);
}

late_initcall (bf_spi_init);
module_exit (bf_spi_exit);


MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("betterlife@blestech.com");
MODULE_DESCRIPTION ("Blestech fingerprint sensor TEE driver.");

