/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#if defined(BUILD_LK)
#ifdef MTK_ROUND_CORNER_SUPPORT
#include "data_rgba8888_roundedpattern.h"
#endif
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
/*#include <mach/mt_pm_ldo.h>*/
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define printk(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define printk(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID	 0x9911

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

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>


#define I2C_I2C_LCD_BIAS_CHANNEL 0
#ifndef CONFIG_FPGA_EARLY_PORTING
#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL	/* for I2C channel 0 */
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info tps65132_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
  {.compatible = "mediatek,I2C_LCD_BIAS"},
  {},
};
#endif

/*static struct i2c_client *tps65132_i2c_client;*/
struct i2c_client *tps65132_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev {
  struct i2c_client *client;

};

static const struct i2c_device_id tps65132_id[] = {
  {I2C_ID_NAME, 0},
  {}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver tps65132_iic_driver = {
  .id_table = tps65132_id,
  .probe = tps65132_probe,
  .remove = tps65132_remove,
  /* .detect               = mt6605_detect, */
  .driver = {
    .owner = THIS_MODULE,
    .name = "tps65132",
#if !defined(CONFIG_MTK_LEGACY)
    .of_match_table = lcm_of_match,
#endif
  },
};

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  printk("tps65132_iic_probe\n");
  printk("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
  tps65132_i2c_client = client;
  return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
  printk("tps65132_remove\n");
  tps65132_i2c_client = NULL;
  i2c_unregister_device(client);
  return 0;
}

/*static int tps65132_write_bytes(unsigned char addr, unsigned char value)*/
#if !defined(CONFIG_ARCH_MT6797)
int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
  int ret = 0;
  struct i2c_client *client = tps65132_i2c_client;
  char write_data[2] = { 0 };

  write_data[0] = addr;
  write_data[1] = value;
  ret = i2c_master_send(client, write_data, 2);
  if (ret < 0)
    printk("tps65132 write data fail !!\n");
  return ret;
}
#endif

static int __init tps65132_iic_init(void)
{
  printk("tps65132_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
  i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
#endif
  printk("tps65132_iic_init2\n");
  i2c_add_driver(&tps65132_iic_driver);
  printk("tps65132_iic_init success\n");
  return 0;
}

static void __exit tps65132_iic_exit(void)
{
  printk("tps65132_iic_exit\n");
  i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif
/* static unsigned char lcd_id_pins_value = 0xFF; */
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)

#ifndef CONFIG_FPGA_EARLY_PORTING
//#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE  (1)
#endif

//#define GPIO_LCM_ID_PIN GPIO_LCD_ID_PIN
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  
#define GPIO_LCM_EN5V         (13+343)
#define GPIO_LCM_EN5V_1        (GPIO13 | 0x80000000)
#define GPIO_LCD_ID_PIN       (0x80 | 0x80000000)
#define GPIO_TP_RST_1         (GPIO8 | 0x80000000)
#define GPIO_LCM_RST_1        (GPIO83 | 0x80000000)
#define GPIO_LCM_RST          (83+343)
#define GPIO_LCM_ENP_1        (GPIO117 | 0x80000000)
#define GPIO_LCM_ENN_1        (GPIO118 | 0x80000000)
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

#ifdef BUILD_LK
static void lcm_set_enp_bias_lk(int output)
{
  mt_set_gpio_mode(GPIO_LCM_EN5V_1, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_EN5V_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_EN5V_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
static void lcm_set_rst_lk(int output)
{
  mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);

  mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

#endif
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

struct LCM_setting_table
{
  unsigned char cmd;
  unsigned char count;
  unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] ={
  //-------------  Display Initial Code Setting  -------------------------
  {0xF0,2 ,{0x5A,0x5A}},
  {0xF1,2 ,{0xA5,0xA5}},
  {0xB0,16,{0x21,0x54,0x76,0x54,0x66,0x66,0x33,0x33,0x0C,0x03,0x03,0x8C,0x03,0x03,0x0F,0x00}},
  {0xB1,16,{0x13,0xD4,0x02,0x86,0x00,0x01,0x01,0x88,0x01,0x01,0x53,0x00,0x00,0x00,0x00,0x00}},
  {0xB2,16,{0x67,0x2A,0x05,0x8A,0x65,0x02,0x08,0x20,0x30,0x91,0x22,0x33,0x44,0x00,0x18,0xA1}},
  {0xB3,16,{0x01,0x00,0x00,0x33,0x00,0x26,0x26,0xC0,0x3F,0xAA,0x33,0xC3,0xAA,0x30,0xC3,0xAA}},
  {0xB6,16,{0x0A,0x02,0x14,0x15,0x1B,0x02,0x02,0x02,0x02,0x13,0x11,0x02,0x02,0x0F,0x0D,0x05}},
  {0xB4,16,{0x0B,0x02,0x14,0x15,0x1B,0x02,0x02,0x02,0x02,0x12,0x10,0x02,0x02,0x0E,0x0C,0x04}},
  {0xBB,16,{0x00,0x00,0x00,0x00,0x02,0xFF,0xFC,0x0B,0x23,0x01,0x73,0x44,0x44,0x00,0x00,0x00}},
  {0xBC,10,{0x61,0x03,0xFF,0x5E,0x72,0xE0,0x2E,0x04,0x88,0x3E}},
  {0xBD,16,{0x6E,0x0E,0x65,0x65,0x15,0x15,0x50,0x41,0x14,0x66,0x23,0x02,0x00,0x00,0x00,0x00}},
  {0xBE, 5,{0x60,0x60,0x50,0x60,0x77}},
  {0xC1,16,{0x70,0x68,0x0C,0x7C,0x04,0x0C,0x10,0x04,0x2A,0x31,0x00,0x07,0x10,0x10,0x00,0x00}},
  {0xC2, 1,{0x00}},
  {0xC3, 8,{0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x0D}},
  {0xC4, 8,{0xB4,0xA3,0xEE,0x41,0x04,0x2F,0x00,0x00}},
  {0xC5,12,{0x07,0x1F,0x42,0x26,0x52,0x44,0x14,0x1A,0x04,0x00,0x0A,0x08}},
  {0xC6,16,{0x81,0x01,0x67,0x01,0x33,0xA0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
  {0xC7,16,{0x7C,0x62,0x51,0x62,0x54,0x3F,0x31,0x36,0x23,0x3F,0x41,0x32,0x4E,0x47,0x56,0x48}},
  {0xC8, 5,{0x35,0x5F,0x38,0x28,0x0C}},
  {0xC9,16,{0x7C,0x62,0x51,0x62,0x54,0x3F,0x31,0x36,0x23,0x3F,0x41,0x32,0x4E,0x47,0x56,0x48}},
  {0xCA, 5,{0x35,0x5F,0x38,0x28,0x0C}},
  {0xCB,11,{0x00,0x00,0x00,0x01,0x6C,0x00,0x33,0x00,0x17,0xFF,0xEF}},
  {0xF0,2 ,{0xB4,0x4B}},
  {0xD0,8 ,{0x80,0x0D,0xFF,0x0F,0x63,0x2B,0x08,0x08}},
  {0xD2,10,{0x43,0x0C,0x00,0x01,0x80,0x26,0x04,0x00,0x16,0x42}},
  {0xd5,1 ,{0x0f}},
  {0x35,1 ,{0x00}},
  {0xF0,2 ,{0xA5,0xA5}},
  {0xF1,2 ,{0x5A,0x5A}},
  {0x26,1 ,{0x01}},

  {0x11,1,{0x00}},
  {REGFLAG_DELAY,120,{}},

  {0x29,1,{0x00}},
  {REGFLAG_DELAY,10,{}},
  {REGFLAG_END_OF_TABLE,0x00,{}}
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
static void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, sizeof(LCM_PARAMS));

  params->type   = LCM_TYPE_DSI;

  params->width  = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  // enable tearing-free
  params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

  //params->dsi.mode   = SYNC_PULSE_VDO_MODE;
  params->dsi.mode   = BURST_VDO_MODE;	

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_FOUR_LANE;

  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;

  params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

  params->dsi.vertical_sync_active				= 4;
  params->dsi.vertical_backporch				= 12;
  params->dsi.vertical_frontporch				= 124;
  params->dsi.vertical_active_line				= FRAME_HEIGHT;

  params->dsi.horizontal_sync_active			= 4;
  params->dsi.horizontal_backporch				= 45;//55
  params->dsi.horizontal_frontporch				= 45;//55
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

  params->dsi.PLL_CLOCK = 251;


  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 0;
  params->dsi.noncont_clock = TRUE;
  params->dsi.noncont_clock_period = 1;
  params->dsi.ssc_disable = 1;
  /*
     params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
     params->dsi.lcm_esd_check_table[0].count = 4;
     params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
     params->dsi.lcm_esd_check_table[0].para_list[1] = 0x00;
     params->dsi.lcm_esd_check_table[0].para_list[2] = 0x0a;
     params->dsi.lcm_esd_check_table[0].para_list[3] = 0x9c;

     params->dsi.lcm_esd_check_table[1].cmd = 0x0c;
     params->dsi.lcm_esd_check_table[1].count = 1;
     params->dsi.lcm_esd_check_table[1].para_list[0] = 0x07;
     params->dsi.lcm_esd_check_table[2].cmd = 0x0e;
     params->dsi.lcm_esd_check_table[2].count = 1;
     params->dsi.lcm_esd_check_table[2].para_list[0] = 0x00;
   */
#if defined(BUILD_LK)
#ifdef MTK_ROUND_CORNER_SUPPORT
  params->round_corner_params.w = ROUND_CORNER_W;
  params->round_corner_params.h = ROUND_CORNER_H;
  params->round_corner_params.lt_addr= left_top;
  params->round_corner_params.rt_addr = right_top;
  params->round_corner_params.lb_addr = left_bottom;
  params->round_corner_params.rb_addr = right_bottom;
#endif
#else
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
  params->round_corner_en = 1;
  params->corner_pattern_width = 18;
  params->corner_pattern_height = 18;
#endif
#endif
}

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

  TPS65132_i2c.id = 0; //I2C_I2C_LCD_BIAS_CHANNEL;	/* I2C2; */
  /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
  TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
  TPS65132_i2c.mode = ST_MODE;
  TPS65132_i2c.speed = 100;
  len = 2;

  ret_code = i2c_write(&TPS65132_i2c, write_data, len);
  /* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

  return ret_code;
}

#else

/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data) */
/* #define TPS65132_read_byte(add)  mt8193_i2c_read(add) */

#endif
#endif

static void lcm_init(void)
{
  unsigned char cmd = 0x0;
  unsigned char data = 0x11;
#ifndef CONFIG_FPGA_EARLY_PORTING
  int ret = 0;
#endif
#ifdef BUILD_LK
  lcm_set_enp_bias_lk(1);
#else
  //set_gpio_lcd_enp(1);
  gpio_set_value_cansleep(GPIO_LCM_EN5V, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_EN5V, 1);
#endif

  cmd = 0x00;
  data = 0x11;
#ifdef BUILD_LK
  //lcm_set_rst_lk(0);
#else
  //gpio_set_value_cansleep(GPIO_LCM_RST, 0);
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
  //	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
  //	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
  //	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
  //#else
  //	set_gpio_lcd_enp(1);
#endif
  MDELAY(5);
#ifdef BUILD_LK
  ret = TPS65132_write_byte(cmd, data);
#else
#if !defined(CONFIG_ARCH_MT6797)
  ret = tps65132_write_bytes(cmd, data);
#endif
#endif

  if (ret < 0)
    printk("icnl9911----tps6132----cmd=%0x--i2c write error----\n", cmd);
  else
    printk("icnl9911----tps6132----cmd=%0x--i2c write success----\n", cmd);

  cmd = 0x01;
  data = 0x11;

#ifdef BUILD_LK
  ret = TPS65132_write_byte(cmd, data);
#else
#if !defined(CONFIG_ARCH_MT6797)
  ret = tps65132_write_bytes(cmd, data);
#endif
#endif

  if (ret < 0)
    printk("icnl9911----tps6132----cmd=%0x--i2c write error----\n", cmd);
  else
    printk("icnl9911----tps6132----cmd=%0x--i2c write success----\n", cmd);

#endif



#ifdef BUILD_LK
  lcm_set_rst_lk(1);
  MDELAY(15);
  lcm_set_rst_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(20);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(15);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(20);
#endif

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
#ifndef BUILD_LK
extern u8 tpd_proximity_flag;
#endif
static void lcm_suspend(void)
{
  unsigned int data_array[16];
  data_array[0] = 0x03261500;
  dsi_set_cmdq(data_array, 1, 1);

  data_array[0]=0x00280500; // Display Off
  dsi_set_cmdq(data_array, 1, 1); 

  MDELAY(10);
#ifndef BUILD_LK
  if(!tpd_proximity_flag)//proximity
#endif
  {
    data_array[0] = 0x00100500; // Sleep In
    dsi_set_cmdq(data_array, 1, 1); 
  }
  MDELAY(120);

#ifndef BUILD_LK
  if(!tpd_proximity_flag)//proximity
#endif
  {
#ifdef BUILD_LK
    lcm_set_enp_bias_lk(0);
    lcm_set_rst_lk(0);
#else
    printk("%s\n",__func__);
    //  set_gpio_lcd_enp(0);
    //gpio_set_value_cansleep(GPIO_LCM_EN5V, 0);
    //gpio_set_value_cansleep(GPIO_LCM_RST, 0);
#endif
  }
}

static void lcm_resume(void)
{
#ifndef BUILD_LK
  if(!tpd_proximity_flag)//proximity
#endif
	lcm_init();
#ifndef BUILD_LK
  else
  {
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
  }
#endif
}

static unsigned int lcm_compare_id(void)
{

  unsigned int array[4];
  unsigned short device_id;
  unsigned char buffer[4];

#ifdef BUILD_LK
  lcm_set_enp_bias_lk(1);
#else
  //set_gpio_lcd_enp(1);
  gpio_set_value_cansleep(GPIO_LCM_EN5V, 1);
#endif

#ifdef BUILD_LK
  lcm_set_rst_lk(1);
  MDELAY(15);
  lcm_set_rst_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(50);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(15);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(50);
#endif

  array[0]=0x00033902;
  array[1]=0x005A5AF0;
  dsi_set_cmdq(array, 2, 1);

  array[0]=0x00033902;
  array[1]=0x00A5A5F1;
  dsi_set_cmdq(array, 2, 1);

  read_reg_v2(0xfa, buffer, 2);	
  device_id = (buffer[0]<<8) | (buffer[1]);

  //printk("-------------wuxiaotian----------- device_id = 0x%4x\n",device_id);
  return (LCM_ID == device_id)?1:0;

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER icnl9911_ls55_hjc_lhd_lcm_drv = 
{
  .name          = "icnl9911_ls55_hjc_lhd",
  .set_util_funcs 	= lcm_set_util_funcs,
  .get_params     	= lcm_get_params,
  .init           	= lcm_init,
  .suspend        	= lcm_suspend,
  .resume         	= lcm_resume,
  .compare_id     	= lcm_compare_id,
};
