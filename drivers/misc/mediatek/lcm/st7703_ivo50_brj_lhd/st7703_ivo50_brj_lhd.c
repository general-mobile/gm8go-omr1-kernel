/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif

#endif

#if !defined(CONFIG_FPGA_EARLY_PORTING)
#ifdef BUILD_LK
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

/*********************************************************
* Gate Driver
*********************************************************/

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/*#include <linux/jiffies.h>*/
#include <linux/uaccess.h>
/*#include <linux/delay.h>*/
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/*****************************************************************************
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING

#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
#ifdef CONFIG_MTK_LEGACY
static struct i2c_board_info tps65132_board_info  __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
#endif

#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
        { .compatible = "mediatek,i2c_lcd_bias" },
        {},
};
#endif

static struct i2c_client *tps65132_i2c_client;


/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev {
    struct i2c_client   *client;

};

static const struct i2c_device_id tps65132_id[] = {
    { I2C_ID_NAME, 0 },
    { }
};

/*#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
 * static struct i2c_client_address_data addr_data = { .forces = forces,};
 * #endif
 */
static struct i2c_driver tps65132_iic_driver = {
    .id_table   = tps65132_id,
    .probe      = tps65132_probe,
    .remove     = tps65132_remove,
    .driver     = {
        .owner  = THIS_MODULE,
        .name   = "tps65132",
#if !defined(CONFIG_MTK_LEGACY)
        .of_match_table = lcm_of_match,
#endif
    },

};
/*****************************************************************************
 * Extern Area
 *****************************************************************************/



/*****************************************************************************
 * Function
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    pr_debug("tps65132_iic_probe\n");
    pr_debug("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
    tps65132_i2c_client  = client;
    return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
    pr_debug("tps65132_remove\n");
    tps65132_i2c_client = NULL;
    i2c_unregister_device(client);
    return 0;
}

#if !defined(CONFIG_ARCH_MT6797)
static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
    int ret = 0;
    struct i2c_client *client = tps65132_i2c_client;
    char write_data[2] = {0};

    write_data[0] = addr;
    write_data[1] = value;
    ret = i2c_master_send(client, write_data, 2);
    if (ret < 0)
        printk("tps65132 write data fail !!\n");
        return ret;
}
#endif

/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

    pr_debug("tps65132_iic_init\n");
#ifdef CONFIG_MTK_LEGACY
    i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
    pr_debug("tps65132_iic_init2\n");
#endif
    i2c_add_driver(&tps65132_iic_driver);
    pr_debug("tps65132_iic_init success\n");
    return 0;
}

static void __exit tps65132_iic_exit(void)
{
    pr_debug("tps65132_iic_exit\n");
    i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");

#endif
#endif
#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING

#define TPS65132_SLAVE_ADDR_WRITE  0x7C
static struct mt_i2c_t TPS65132_i2c;

static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0] = addr;
    write_data[1] = value;

    TPS65132_i2c.id = I2C0;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    /*printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);*/

    return ret_code;
}

#endif
#endif

#define LCM_ID	 0x3821

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
  lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
  lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
  lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
  lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
  lcm_util.set_gpio_lcd_enp_bias(cmd)

#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)

#ifndef CONFIG_FPGA_EARLY_PORTING
#endif
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif

static unsigned int lcm_compare_id(void);

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  

#ifdef BUILD_LK
#define GPIO_LCM_RST_1         (GPIO83 | 0x80000000)
#define GPIO_LCM_ENP           (GPIO118 | 0x80000000)
#define GPIO_LCM_ENN           (GPIO31 | 0x80000000)
#else
#define GPIO_LCM_RST           (83+343)
#define GPIO_LCM_ENP           (118+343)
#define GPIO_LCM_ENN           (31+343)
#endif

#ifdef BUILD_LK
static void lcm_set_rst_lk(int output)
{
  mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
static void lcm_set_power_lk(int output)
{
  mt_set_gpio_mode(GPIO_LCM_ENP, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_ENP, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_ENP, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
  mt_set_gpio_mode(GPIO_LCM_ENN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_ENN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_ENN, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
#else
static void lcm_set_power_kernel(int output)
{
  gpio_set_value_cansleep(GPIO_LCM_ENP, ((output>0) ? 1:0));
  gpio_set_value_cansleep(GPIO_LCM_ENN, ((output>0) ? 1:0));
}
#endif

static void lcm_init_power(void)
{
    unsigned char cmd = 0x0;
    unsigned char data = 0x12;
    int ret = 0;

    //display_bias_enable();
#ifdef BUILD_LK
  lcm_set_power_lk(1);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(10);
  lcm_set_rst_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(80);
#else
  lcm_set_power_kernel(1);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(80);
#endif
    MDELAY(20);
#ifdef BUILD_LK
    ret = TPS65132_write_byte(cmd, data);
    if (ret)
        dprintf(0, "[LK]lcm----tps6132----cmd=%0x--i2c write error----\n", cmd);
    else
        dprintf(0, "[LK]lcm----tps6132----cmd=%0x--i2c write success----\n", cmd);
#else
    ret = tps65132_write_bytes(cmd, data);
    if (ret < 0)
        pr_debug("[KERNEL]lcm----tps6132---cmd=%0x-- i2c write error-----\n", cmd);
    else
        pr_debug("[KERNEL]lcm----tps6132---cmd=%0x-- i2c write success-----\n", cmd);
#endif

    cmd = 0x01;
    data = 0x12;
#ifdef BUILD_LK
    ret = TPS65132_write_byte(cmd, data);
    if (ret)
        dprintf(0, "[LK]lcm----tps6132----cmd=%0x--i2c write error----\n", cmd);
    else
        dprintf(0, "[LK]lcm----tps6132----cmd=%0x--i2c write success----\n", cmd);
#else
    ret = tps65132_write_bytes(cmd, data);
    if (ret < 0)
        pr_debug("[KERNEL]lcm----tps6132---cmd=%0x-- i2c write error-----\n", cmd);
    else
        pr_debug("[KERNEL]lcm----tps6132---cmd=%0x-- i2c write success-----\n", cmd);
#endif

}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
  lcm_set_power_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(0);
  MDELAY(120);
#else
  lcm_set_power_kernel(0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(120);
#endif
}
static void lcm_resume_power(void)
{
    lcm_init_power();
}

struct LCM_setting_table
{
    unsigned char cmd;
  unsigned char count;
  unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{

{0xb9, 3,{0xf1,0x12,0x83}},
{0xba, 27,{0x33,0x81,0x05,0xf9,0x0e,0x0e,0x02,0x00,0x00,0x00,
		   0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0a,0x00,
		   0x00,0x02,0x4f,0x11,0x00,0x00,0x37}},
{0xb8, 4,{0x75,0x22,0x20,0x03}},
{0xb3, 10,{0x10,0x10,0x05,0x05,0x03,0xff,0x00,0x00,0x00,0x00}},
{0xc0, 9,{0x70,0x73,0x50,0x50,0x00,0x00,0x08,0x70,0x00}},
{0xbc, 1,{0x4e}},
{0xcc, 1,{0x0b}},
{0xb4, 1,{0x80}},
{0xb2, 3,{0xf0,0x12,0x30}},
{0xe3, 14,{0x07,0x07,0x0b,0x0b,0x03,0x0b,0x00,0x00,0x00,0x00,0xff,0x00,0xc0,0x10}},
{0xc1, 12,{0x54,0x00,0x1e,0x1e,0x77,0xf1,0xff,0xff,0xcc,0xcc,0x77,0x77}},
{0xb5, 2,{0x07,0x07}},
{0xb6, 2,{0x30,0x30}},
{0xbf, 3,{0x02,0x11,0x00}},
{0xe9, 63,{0x02,0x00,0x10,0x05,0xb6,0x0a,0xa0,0x12,0x31,0x23,
		   0x37,0x13,0x40,0xa0,0x27,0x38,0x0c,0x00,0x03,0x00,
		   0x00,0x00,0x0c,0x00,0x03,0x00,0x00,0x00,0x75,0x75,
		   0x31,0x88,0x88,0x88,0x88,0x88,0x88,0x13,0x88,0x64,
		   0x64,0x20,0x88,0x88,0x88,0x88,0x88,0x88,0x02,0x88,
		   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xea, 61,{0x02,0x21,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		   0x00,0x00,0x02,0x46,0x02,0x88,0x88,0x88,0x88,0x88,
		   0x88,0x64,0x88,0x13,0x57,0x13,0x88,0x88,0x88,0x88,
		   0x88,0x88,0x75,0x88,0x23,0x10,0x00,0x00,0x30,0x00,
		   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		   0x00,0x00,0x00,0x00,0x03,0x0a,0xa0,0x00,0x00,0x00,0x00}},
{0xe0, 34,{0x00,0x07,0x0f,0x30,0x35,0x3c,0x43,0x37,0x06,0x0d,
		   0x0d,0x10,0x11,0x0f,0x12,0x0f,0x14,0x00,0x07,0x0f,
		   0x30,0x35,0x3c,0x43,0x37,0x06,0x0d,0x0d,0x10,0x11,0x0f,0x12,0x0f,0x14}},

  
{0x35,01,{0x00}}, 
{0x11,1,{0x00}},        // Sleep-Out
{REGFLAG_DELAY, 120, {}},
{0x29,1, {0x00}},       // Display On
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};
static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
  // Display off sequence
  {0x28, 0, {0x00}},
  {REGFLAG_DELAY, 120, {}},

  // Sleep Mode On
  {0x10, 0, {0x00}},
  {REGFLAG_DELAY, 200, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
  unsigned int i;

  for(i = 0; i < count; i++)
  {

    unsigned cmd;
    cmd = table[i].cmd;

    switch (cmd)
    {

      case REGFLAG_DELAY :
        MDELAY(table[i].count);
        break;

      case REGFLAG_END_OF_TABLE :
        break;

      default:
        dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        //MDELAY(2);
    }
  }

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, sizeof(LCM_PARAMS));

  params->type = LCM_TYPE_DSI;

  params->width = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  // enable tearing-free
  params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    //params->dsi.mode   = SYNC_PULSE_VDO_MODE;
  params->dsi.mode = SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_FOUR_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

  params->dsi.packet_size=256;
  params->dsi.intermediat_buffer_num = 2;

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_sync_active				= 6;
    params->dsi.vertical_backporch				= 15; //12
    params->dsi.vertical_frontporch				= 16; //6	
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 10; //10
    params->dsi.horizontal_backporch				= 70; //70
  params->dsi.horizontal_frontporch				= 60; //70;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


params->dsi.PLL_CLOCK = 258 ;//260;//212;//208;	
params->dsi.ssc_disable = 1;	
}

static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(5);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(5);
  SET_RESET_PIN(1);
  MDELAY(20);
  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
  lcm_init();
}

static unsigned int lcm_compare_id(void)
{

 	unsigned int array[4];
	unsigned short device_id;
	unsigned char buffer[4];

  SET_RESET_PIN(1);
  MDELAY(20);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);
  /*
     array[0]=0x00063902;
     array[1]=0x000177ff;
     array[2]=0x00001000;
     dsi_set_cmdq(array, 3, 1);
   */
  array[0] = 0x00033700;
  dsi_set_cmdq(array, 1, 1);
	MDELAY(10); 

  read_reg_v2(0x04, buffer, 2);	
  device_id = (buffer[0]<<8) | (buffer[1]);

#if defined(BUILD_LK)
	printf("st7701_ivo50_hongzhan_fwvga  %s,line=%d, id = 0x%x, buffer[0]=0x%08x,buffer[1]=0x%08x\n",__func__,__LINE__, device_id, buffer[0],buffer[1]);
#else
	printk("st7701_ivo50_hongzhan_fwvga  %s,line=%d, id = 0x%x, buffer[0]=0x%08x,buffer[1]=0x%08x\n",__func__,__LINE__, device_id, buffer[0],buffer[1]);
#endif
  return (LCM_ID == device_id)?1:0;


}

LCM_DRIVER st7703_ivo50_brj_lhd_lcm_drv =
{
  .name			= "st7703_ivo50_brj_lhd",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,
  .compare_id     = lcm_compare_id,
  .init_power     = lcm_init_power,
  .resume_power   = lcm_resume_power,
  .suspend_power  = lcm_suspend_power,
};
