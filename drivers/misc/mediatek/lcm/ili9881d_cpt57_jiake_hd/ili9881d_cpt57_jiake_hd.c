/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

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

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1440)
#define LCM_PHYSICAL_WIDTH									(64800)
#define LCM_PHYSICAL_HEIGHT									(129600)
#define LCM_DENSITY											(480)

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

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

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

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

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xFF, 3,{0x98,0x81,0x03}},
{0x01, 1,{0x00}},
{0x02, 1,{0x00}},
{0x03, 1,{0x56}},
{0x04, 1,{0x13}},
{0x05, 1,{0x00}},
{0x06, 1,{0x06}},
{0x07, 1,{0x01}},
{0x08, 1,{0x00}},
{0x09, 1,{0x30}},
{0x0a, 1,{0x01}},
{0x0b, 1,{0x00}},
{0x0c, 1,{0x30}},
{0x0d, 1,{0x01}},
{0x0e, 1,{0x00}},
{0x0f, 1,{0x18}},
{0x10, 1,{0x18}},
{0x11, 1,{0x00}},
{0x12, 1,{0x00}},
{0x13, 1,{0x00}},
{0x14, 1,{0x00}},
{0x15, 1,{0x08}},
{0x16, 1,{0x08}},
{0x17, 1,{0x00}},
{0x18, 1,{0x08}},
{0x19, 1,{0x00}},
{0x1a, 1,{0x00}},
{0x1b, 1,{0x00}},
{0x1c, 1,{0x00}},
{0x1d, 1,{0x00}},
{0x1e, 1,{0x40}},
{0x1f, 1,{0xc0}},
{0x20, 1,{0x02}},
{0x21, 1,{0x05}},
{0x22, 1,{0x02}},
{0x23, 1,{0x00}},
{0x24, 1,{0x86}},
{0x25, 1,{0x85}},
{0x26, 1,{0x00}},
{0x27, 1,{0x00}},
{0x28, 1,{0x3b}},
{0x29, 1,{0x03}},
{0x2a, 1,{0x00}},
{0x2b, 1,{0x00}},
{0x2c, 1,{0x00}},
{0x2d, 1,{0x00}},
{0x2e, 1,{0x00}},
{0x2f, 1,{0x00}},
{0x30, 1,{0x00}},
{0x31, 1,{0x00}},
{0x32, 1,{0x00}},
{0x33, 1,{0x00}},
{0x34, 1,{0x00}},
{0x35, 1,{0x00}},
{0x36, 1,{0x00}},
{0x37, 1,{0x00}},
{0x38, 1,{0x00}},
{0x39, 1,{0x35}},
{0x3a, 1,{0x00}},
{0x3b, 1,{0x40}},
{0x3c, 1,{0x00}},
{0x3d, 1,{0x00}},
{0x3e, 1,{0x00}},
{0x3f, 1,{0x00}},
{0x40, 1,{0x38}},
{0x41, 1,{0x88}},
{0x42, 1,{0x00}},
{0x43, 1,{0x00}},
{0x44, 1,{0x3f}},
{0x45, 1,{0x20}},
{0x46, 1,{0x00}},

{0x50, 1,{0x01}},
{0x51, 1,{0x23}},
{0x52, 1,{0x45}},
{0x53, 1,{0x67}},
{0x54, 1,{0x89}},
{0x55, 1,{0xab}},
{0x56, 1,{0x01}},
{0x57, 1,{0x23}},
{0x58, 1,{0x45}},
{0x59, 1,{0x67}},
{0x5a, 1,{0x89}},
{0x5b, 1,{0xab}},
{0x5c, 1,{0xcd}},
{0x5d, 1,{0xef}},
{0x5e, 1,{0x11}},
{0x5f, 1,{0x01}},
{0x60, 1,{0x00}},
{0x61, 1,{0x08}},
{0x62, 1,{0x0f}},
{0x63, 1,{0x0e}},
{0x64, 1,{0x0d}},
{0x65, 1,{0x0c}},
{0x66, 1,{0x02}},
{0x67, 1,{0x02}},
{0x68, 1,{0x02}},
{0x69, 1,{0x02}},
{0x6a, 1,{0x02}},
{0x6b, 1,{0x02}},
{0x6c, 1,{0x02}},
{0x6d, 1,{0x02}},
{0x6e, 1,{0x06}},
{0x6f, 1,{0x02}},
{0x70, 1,{0x02}},
{0x71, 1,{0x02}},
{0x72, 1,{0x02}},
{0x73, 1,{0x02}},
{0x74, 1,{0x02}},
{0x75, 1,{0x01}},
{0x76, 1,{0x00}},
{0x77, 1,{0x06}},
{0x78, 1,{0x0f}},
{0x79, 1,{0x0e}},
{0x7a, 1,{0x0d}},
{0x7b, 1,{0x0c}},
{0x7c, 1,{0x02}},
{0x7d, 1,{0x02}},
{0x7e, 1,{0x02}},
{0x7f, 1,{0x02}},
{0x80, 1,{0x02}},
{0x81, 1,{0x02}},
{0x82, 1,{0x02}},
{0x83, 1,{0x02}},
{0x84, 1,{0x08}},
{0x85, 1,{0x02}},
{0x86, 1,{0x02}},
{0x87, 1,{0x02}},
{0x88, 1,{0x02}},
{0x89, 1,{0x02}},
{0x8a, 1,{0x02}},

{0xFF, 3,{0x98,0x81,0x04}},    
{0x68, 1,{0xDB}},
{0x6D, 1,{0x08}},
{0x70, 1,{0x00}},
{0x71, 1,{0x00}},
{0x66, 1,{0xFE}},
{0x6F, 1,{0x05}},
{0x82, 1,{0x14}}, //0f
{0x84, 1,{0x14}}, //0f
{0x85, 1,{0x10}},  //0f  
{0x32, 1,{0xAC}},
{0x8C, 1,{0x80}},
{0x3C, 1,{0xF5}},
{0x3A, 1,{0x24}},
{0xB5, 1,{0x02}},
{0x31, 1,{0x25}},
{0x88, 1,{0x33}},
{0x89, 1,{0xBA}},
    
{0xFF, 3,{0x98,0x81,0x01}},    
{0x22, 1,{0x0A}},
{0x31, 1,{0x00}},
{0x50, 1,{0x5b}},  //3c
{0x51, 1,{0x5b}},  //3b
{0x53, 1,{0x3F}},
{0x55, 1,{0x46}},
{0x60, 1,{0x28}},
{0x61, 1,{0x00}},
{0x62, 1,{0x0D}},
{0x63, 1,{0x00}},
{0x2E, 1,{0xF0}},
{0x2F, 1,{0x00}},

//============Gamma START=============
//Pos Register   
{0xA0, 1,{0x08}},	
{0xA1, 1,{0x4B}},	
{0xA2, 1,{0x59}},	
{0xA3, 1,{0x14}},	
{0xA4, 1,{0x17}},	
{0xA5, 1,{0x29}},	
{0xA6, 1,{0x1E}},	
{0xA7, 1,{0x1E}},	
{0xA8, 1,{0xD2}},	
{0xA9, 1,{0x1C}},	
{0xAA, 1,{0x28}},	
{0xAB, 1,{0xAF}},	
{0xAC, 1,{0x1A}},	
{0xAD, 1,{0x19}},	
{0xAE, 1,{0x4C}},	
{0xAF, 1,{0x21}},	
{0xB0, 1,{0x28}},	
{0xB1, 1,{0x5F}},	
{0xB2, 1,{0x6C}},	
{0xB3, 1,{0x34}},	

//Neg Register    
{0xC0, 1,{0x08}},	
{0xC1, 1,{0x4B}},	
{0xC2, 1,{0x59}},	
{0xC3, 1,{0x14}},	
{0xC4, 1,{0x17}},	
{0xC5, 1,{0x29}},	
{0xC6, 1,{0x1E}},	
{0xC7, 1,{0x1E}},	
{0xC8, 1,{0xD2}},	
{0xC9, 1,{0x1C}},	
{0xCA, 1,{0x28}},	
{0xCB, 1,{0xAF}},	
{0xCC, 1,{0x1A}},	
{0xCD, 1,{0x19}},	
{0xCE, 1,{0x4C}},	
{0xCF, 1,{0x21}},	
{0xD0, 1,{0x28}},	
{0xD1, 1,{0x5F}},	
{0xD2, 1,{0x6C}},	
{0xD3, 1,{0x34}},	
   
{0xFF, 3,{0x98,0x81,0x00}},    
{0x35,1, {0x00}},  

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

  for(i = 0; i < count; i++) {

    unsigned cmd;
    cmd = table[i].cmd;

    switch (cmd) {

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
/*
  params->physical_width = LCM_PHYSICAL_WIDTH/1000;
  params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
  params->physical_width_um = LCM_PHYSICAL_WIDTH;
  params->physical_height_um = LCM_PHYSICAL_HEIGHT;
  params->density            = LCM_DENSITY;
*/
  // enable tearing-free
  params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

  params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM = LCM_FOUR_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

  params->dsi.intermediat_buffer_num = 2;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count = 720 * 3;

  params->dsi.vertical_sync_active                = 4;// 3    2
  params->dsi.vertical_backporch                  = 18;// 20   1
  params->dsi.vertical_frontporch                 = 10; // 1  12
  params->dsi.vertical_active_line                = FRAME_HEIGHT;

  params->dsi.horizontal_sync_active              = 20;// 50  2
  params->dsi.horizontal_backporch                = 120;//90
  params->dsi.horizontal_frontporch               = 72;//90
  params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

  params->dsi.ssc_disable                         = 1;
  params->dsi.ssc_range                         = 4;
  params->dsi.HS_TRAIL                             = 15;
  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 0;
  params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
  params->dsi.lcm_esd_check_table[0].count        = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

  params->dsi.PLL_CLOCK =250;
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
static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
  lcm_init();
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


static unsigned int lcm_compare_id(void)
{

  int array[4];
  char buffer[3];
  char id_high=0;
  char id_low=0;
  int id=0;

  SET_RESET_PIN(1);
  MDELAY(20);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);

  //{0x39, 0xFF, 5, { 0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1 CMD
  array[0] = 0x00043902;
  array[1] = 0x068198FF;
  dsi_set_cmdq(array, 2, 1);

  array[0] = 0x00013700;
  dsi_set_cmdq(array, 1, 1);
  read_reg_v2(0xF2, &buffer[0], 1);  //0xOd

  id = buffer[0];
#if defined(BUILD_LK)
  printf("%s, [ili9881d_ivo50_txd_hd]  buffer[0] = [0x%x]  ID = [0x%x]\n",__func__,buffer[0], id);
#else
  printk("%s, [ili9881d_ivo50_txd_hd]  buffer[0] = [0x%x]  ID = [0x%x]\n",__func__,buffer[0], id);
#endif


  return (0x0d == id)?1:0;

}


LCM_DRIVER ili9881d_cpt57_jiake_hd_lcm_drv =
{
  .name			  = "ili9881d_cpt57_jiake_hd",
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
