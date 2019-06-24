/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

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

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
#else

#include <linux/proc_fs.h> //proc file use
#endif



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1440)


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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


struct LCM_setting_table
{
  unsigned cmd;
  unsigned char count;
  unsigned char para_list[64];
};

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static struct LCM_setting_table lcm_initialization_setting[] =
{
  {0xFF,03,{0x98,0x81,0x03}},    
  {0x01,01,{0x00}},    
  {0x02,01,{0x00}},    
  {0x03,01,{0x53}},   //52=3H_53=3.5H/4H
  {0x04,01,{0x13}},   //12=3H_13=3.5H/4H      
  {0x05,01,{0x00}},    
  {0x06,01,{0x04}},   //03=3H_04=3.5H/4H  
  {0x07,01,{0x00}},   //01=3H_02=3.5H_00=4H    
  {0x08,01,{0x00}},    
  {0x09,01,{0x14}},   //01=3H_28=3.5H_14=4H  
  {0x0a,01,{0x14}},   //00=3H_28=3.5H_14=4H  
  {0x0b,01,{0x00}},    
  {0x0c,01,{0x01}},    
  {0x0d,01,{0x00}},    
  {0x0e,01,{0x00}},    
  {0x0f,01,{0x14}},    //00=3H_16=3.5H_14=4H  
  {0x10,01,{0x14}},    //00=3H_16=3.5H_14=4H  
  {0x11,01,{0x00}},    
  {0x12,01,{0x00}},    
  {0x13,01,{0x00}},    
  {0x14,01,{0x00}},    
  {0x15,01,{0x08}},    
  {0x16,01,{0x08}},    
  {0x17,01,{0x00}},    
  {0x18,01,{0x08}},    
  {0x19,01,{0x00}},    
  {0x1a,01,{0x00}},    
  {0x1b,01,{0x00}},    
  {0x1c,01,{0x00}},    
  {0x1d,01,{0x00}},    
  {0x1e,01,{0x44}},  
  {0x1f,01,{0x80}},    
  {0x20,01,{0x02}},     //01=3H_02=3.5H/4H     
  {0x21,01,{0x03}},     //02_3.5H_03=4H  
  {0x22,01,{0x00}},    
  {0x23,01,{0x00}},    
  {0x24,01,{0x00}},    
  {0x25,01,{0x00}},    
  {0x26,01,{0x00}},    
  {0x27,01,{0x00}},    
  {0x28,01,{0x33}}, 
  {0x29,01,{0x03}},     //02=3H_03=3.5H/4H  
  {0x2a,01,{0x00}},    
  {0x2b,01,{0x00}},    
  {0x2c,01,{0x00}},    
  {0x2d,01,{0x00}},    
  {0x2e,01,{0x00}},    
  {0x2f,01,{0x00}},    
  {0x30,01,{0x00}},    
  {0x31,01,{0x00}},    
  {0x32,01,{0x00}},    
  {0x33,01,{0x00}},    
  {0x34,01,{0x04}},    
  {0x35,01,{0x00}},    
  {0x36,01,{0x00}},    
  {0x37,01,{0x00}},    
  {0x38,01,{0x3C}},    
  {0x39,01,{0x35}},
  {0x3A,01,{0x01}},
  {0x3B,01,{0x40}},
  {0x3C,01,{0x00}},
  {0x3D,01,{0x01}},
  {0x3E,01,{0x00}},
  {0x3F,01,{0x00}},
  {0x40,01,{0x00}},
  {0x41,01,{0x88}},
  {0x42,01,{0x00}},
  {0x43,01,{0x00}},
  {0x44,01,{0x3F}},     //1F TO 3F_ RESET KEEP LOW ALL GATE ON
  {0x45,01,{0x20}},     //LVD茦追摽ALL GATE ON祫VGH
  {0x46,01,{0x00}},

  //====================//   
  {0x50,01,{0x01}},    
  {0x51,01,{0x23}},    
  {0x52,01,{0x45}},    
  {0x53,01,{0x67}},    
  {0x54,01,{0x89}},     
  {0x55,01,{0xab}},    
  {0x56,01,{0x01}},    
  {0x57,01,{0x23}},    
  {0x58,01,{0x45}},    
  {0x59,01,{0x67}},    
  {0x5a,01,{0x89}},    
  {0x5b,01,{0xab}},    
  {0x5c,01,{0xcd}},    
  {0x5d,01,{0xef}},

  {0x5e,01,{0x11}}, 
  {0x5f,01,{0x01}},    
  {0x60,01,{0x00}},    
  {0x61,01,{0x15}},    
  {0x62,01,{0x14}},    
  {0x63,01,{0x0C}},    
  {0x64,01,{0x0D}},    
  {0x65,01,{0x0E}},    
  {0x66,01,{0x0F}},    
  {0x67,01,{0x06}},    
  {0x68,01,{0x02}},    
  {0x69,01,{0x02}},    
  {0x6a,01,{0x02}},    
  {0x6b,01,{0x02}},    
  {0x6c,01,{0x02}},    
  {0x6d,01,{0x02}},      
  {0x6e,01,{0x08}},
  {0x6f,01,{0x02}},    
  {0x70,01,{0x02}},    
  {0x71,01,{0x02}},    
  {0x72,01,{0x02}},    
  {0x73,01,{0x02}},    
  {0x74,01,{0x02}},    
  {0x75,01,{0x01}},    
  {0x76,01,{0x00}},    
  {0x77,01,{0x15}},    
  {0x78,01,{0x14}},    
  {0x79,01,{0x0C}},    
  {0x7a,01,{0x0D}},
  {0x7b,01,{0x0E}}, 
  {0x7c,01,{0x0F}},    
  {0x7D,01,{0x08}},  
  {0x7E,01,{0x02}},    
  {0x7F,01,{0x02}},    
  {0x80,01,{0x02}},    
  {0x81,01,{0x02}},    
  {0x82,01,{0x02}},    
  {0x83,01,{0x02}},    
  {0x84,01,{0x06}},   
  {0x85,01,{0x02}},    
  {0x86,01,{0x02}},    
  {0x87,01,{0x02}},    
  {0x88,01,{0x02}},    
  {0x89,01,{0x02}},    
  {0x8A,01,{0x02}},
  //====================//    
  {0xFF,03,{0x98,0x81,0x04}},  

  {0x00,01,{0x00}},     //3L

  {0x68,01,{0xDB}},     //nonoverlap 18ns (VGH and VGL)
  {0x6D,01,{0x08}},     //gvdd_isc[2:0]=0 (0.2uA) 可減少VREG1擾動
  {0x70,01,{0x00}},     //VGH_MOD and VGH_DC CLKDIV disable
  {0x71,01,{0x00}},     //VGL CLKDIV disable
  {0x66,01,{0xFE}},     //VGH 4X
  {0x3A,01,{0x24}},     //PS_EN OFF
  {0x82,01,{0x10}},     //VREF_VGH_MOD_CLPSEL 15V
  {0x84,01,{0x10}},     //VREF_VGH_CLPSEL 15V
  {0x85,01,{0x10}},     //VREF_VGL_CLPSEL -11V
  {0x32,01,{0xAC}},     //開啟負channel的power saving
{0x8C,01,{0x80}},     //sleep out Vcom disable以避免Vcom source不同步enable導致玻璃微亮
{0x3C,01,{0xF5}},     //開啟Sample & Hold Function
{0x3A,01,{0x24}},     //PS_EN OFF       
{0xB5,01,{0x02}},     //GAMMA OP 
{0x31,01,{0x25}},     //SOURCE OP 
{0x88,01,{0x33}},     //VSP/VSN LVD Disable   
{0x89,01,{0xBA}},     //VCI LVD ON

  //====================// 
{0xFF,03,{0x98,0x81,0x01}},    
{0x22,01,{0x0A}},      
{0x31,01,{0x00}},     //column inversion     
{0x50,01,{0x6B}},     //VREG10UT 4.7  
{0x51,01,{0x6B}},     //VREG20UT -4.7
{0x53,01,{0x8a}},     //VC0M1      
{0x55,01,{0x8A}},     //VC0M2        
{0x60,01,{0x16}},     //SDT=2.5us      
{0x61,01,{0x00}},     //CR    
{0x62,01,{0x0D}},     //EQ
{0x63,01,{0x00}},     //PC
{0x2E,01,{0xF0}},     //1440 GATE NL SEL  
{0x2F,01,{0x00}},     //480 SOURCE


  //====================//   GAMMA Positive
{0xA0,01,{0x00}},	
{0xA1,01,{0x17}},	
{0xA2,01,{0x25}},	
{0xA3,01,{0x15}},	
{0xA4,01,{0x19}},	
{0xA5,01,{0x2a}},	
{0xA6,01,{0x1e}},	
{0xA7,01,{0x1f}},	
{0xA8,01,{0x84}},	
{0xA9,01,{0x1d}},	
{0xAA,01,{0x28}},	
{0xAB,01,{0x75}},	
{0xAC,01,{0x1b}},	
{0xAD,01,{0x1a}},	
{0xAE,01,{0x4f}},	
{0xAF,01,{0x23}},	
{0xB0,01,{0x2a}},	
{0xB1,01,{0x50}},	
{0xB2,01,{0x5e}},	
{0xB3,01,{0x25}},	



  //====================//  GAMMA Negative
{0xC0,01,{0x00}},	
{0xC1,01,{0x16}},	
{0xC2,01,{0x26}},	
{0xC3,01,{0x16}},	
{0xC4,01,{0x19}},	
{0xC5,01,{0x2b}},	
{0xC6,01,{0x1e}},	
{0xC7,01,{0x1F}},	
{0xC8,01,{0x84}},	
{0xC9,01,{0x1d}},	
{0xCA,01,{0x28}},	
{0xCB,01,{0x76}},	
{0xCC,01,{0x1b}},	
{0xCD,01,{0x1a}},	
{0xCE,01,{0x4E}},	
{0xCF,01,{0x23}},	
{0xD0,01,{0x29}},	
{0xD1,01,{0x51}},	
{0xD2,01,{0x5d}},	
{0xD3,01,{0x25}},	


  //====================//      
{0xFF,03,{0x98,0x81,0x00}},    
{0x35,01,{0x00}}, 
{0x11,1,{0x00}},        // Sleep-Out
{REGFLAG_DELAY, 120, {}},
{0x29,1, {0x00}},       // Display On
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};
static struct LCM_setting_table lcm_sleep_mode_in_setting[] =
{
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
  params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

  params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM = LCM_THREE_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

  params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count = 720 * 3;

  params->dsi.vertical_sync_active				= 8;
  params->dsi.vertical_backporch					= 24;
  params->dsi.vertical_frontporch					= 16;
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 40;
  params->dsi.horizontal_backporch				= 100;
  params->dsi.horizontal_frontporch				= 70;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.HS_TRAIL=20; 
  params->dsi.PLL_CLOCK = 220;
}
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static int adc_read_vol(void)
{
	
  int adc[1];
  int data[4] ={0,0,0,0};
  int sum = 0;
  int adc_vol=0;
  int num = 0;

  for(num=0;num<10;num++)
  {
    IMM_GetOneChannelValue(12, data, adc);
    sum+=(data[0]*100+data[1]);
  }
  adc_vol = sum/10;

#if defined(BUILD_LK)
  printf("ili9881c adc_vol is %d\n",adc_vol);
#else
  printk("ili9881c adc_vol is %d\n",adc_vol);
#endif
 return (adc_vol < 60) ? 0: 1;
}
static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);
  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
  lcm_init();
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


  return (0x0d == ( id + adc_read_vol()))?1:0;

}

LCM_DRIVER ili9881d_hsd55_hongzhan_lhd_lcm_drv =
{
  .name			= "ili9881d_hsd55_hongzhan_lhd",
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
