/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_proximity.c
*
*    Author: luoguojin
*
*   Created: 2016-09-19
*
*  Abstract: close proximity function
*
*   Version: v1.0
*
* Revision History:
*        v1.0:
*            First release based on xiaguobin's solution. By luougojin 2016-08-19
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

#if FTS_PSENSOR_EN
//#include <hwmsen_dev.h>
//#include <sensors_io.h>

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct fts_proximity_st {
    u8      mode                : 1;    /* 1- proximity enable 0- disable */
    u8      detect              : 1;    /* 0-->close ; 1--> far away */
    u8      unused              : 4;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_proximity_st fts_proximity_data;
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
extern void touch_ps_data_report(int value);
int fts_enter_proximity_mode(int mode);
static ssize_t fts_touch_proximity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_touch_proximity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
/*****************************************************************************
* functions body
*****************************************************************************/


/* read and write proximity mode
*   read example: cat  fts_touch_proximity_mode---read  proximity mode
*   write example:echo 01 > fts_touch_proximity_mode ---write proximity mode to 01
*/
static DEVICE_ATTR (fts_touch_proximity_mode, S_IRUGO | S_IWUSR, fts_touch_proximity_show, fts_touch_proximity_store);
static struct attribute *fts_touch_proximity_attrs[] = {
    &dev_attr_fts_touch_proximity_mode.attr,
    NULL,
};

static struct attribute_group fts_touch_proximity_group = {
    .attrs = fts_touch_proximity_attrs,
};


static ssize_t fts_touch_proximity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    mutex_lock(&fts_data->input_dev->mutex);
    return snprintf(buf, PAGE_SIZE, "Proximity: %s\n", fts_proximity_data.mode ? "On" : "Off");
    mutex_unlock(&fts_data->input_dev->mutex);
}

static ssize_t fts_touch_proximity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    int ret;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&fts_data->input_dev->mutex);
    val = simple_strtoul(buf, 0, 10);
    if (val == 1) {
        if (!fts_proximity_data.mode) {
            fts_proximity_data.mode = 1;
            ret = fts_enter_proximity_mode(1);
        }
    } else {
        if (fts_proximity_data.mode) {
            fts_proximity_data.mode = 0;
            ret = fts_enter_proximity_mode(0);
        }
    }
    mutex_unlock(&fts_data->input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_enter_proximity_mode
* Brief:  change proximity mode
* Input:  proximity mode
* Output: no
* Return: success =0
***********************************************************************/
u8 tpd_proximity_flag                    = 0;
int fts_enter_proximity_mode(int mode)
{
    int ret = 0;
    u8 buf_addr = 0;
    u8 buf_value = 0;

    buf_addr = FTS_REG_FACE_DEC_MODE_EN;
    if (mode)
    {
        buf_value = 0x01;
        tpd_proximity_flag = 1;
    }
    else
    {
        buf_value = 0x00;
        tpd_proximity_flag = 0;
    }
    ret = fts_i2c_write_reg(fts_data->client, buf_addr, buf_value);
    if (ret < 0) {
        FTS_ERROR("[PROXIMITY] Write proximity register(0xB0) fail!");
        return ret;
    }

    fts_proximity_data.mode = buf_value ? 1 : 0;
    FTS_DEBUG("[PROXIMITY] proximity mode = %d", fts_proximity_data.mode);

    return ret ;
}

/*****************************************************************************
*  Name: fts_proximity_recovery
*  Brief: need call when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_proximity_recovery(struct i2c_client *client)
{
    int ret = 0;

    if (fts_proximity_data.mode)
        ret = fts_enter_proximity_mode(1);

    return ret;
}

/*****************************************************************************
*  Name: fts_proximity_readdata
*  Brief:
*  Input:
*  Output:
*  Return: 0 - need return in suspend
*****************************************************************************/
int fts_proximity_readdata(struct i2c_client *client)
{
    int ret;
    int proximity_status = 1;
    u8  regvalue;

    if (fts_proximity_data.mode == 0)
        return -EPERM;

    fts_i2c_read_reg(client, FTS_REG_FACE_DEC_MODE_STATUS, &regvalue);

    if (regvalue == 0xC0) {
        /* close. need lcd off */
        proximity_status = 0;
    } else if (regvalue == 0xE0) {
        /* far away */
        proximity_status = 1;
    }

    if (proximity_status != (int)fts_proximity_data.detect) 
    {
        FTS_DEBUG("[PROXIMITY] p-sensor state:%s", proximity_status ? "AWAY" : "NEAR");
        fts_proximity_data.detect = proximity_status ? 1 : 0;
        touch_ps_data_report(proximity_status);
        return 0;
    }

    return -1;
}

/*****************************************************************************
*  Name: fts_proximity_suspend
*  Brief: Run when tp enter into suspend
*  Input:
*  Output:
*  Return: 0 - need return in suspend
*****************************************************************************/
int fts_proximity_suspend_flag = 0;
int fts_proximity_suspend(void)
{
    if (fts_proximity_data.mode == 1)
    {
        fts_proximity_suspend_flag=1;
        return 0;
    }
    else
    {
        fts_proximity_suspend_flag=0;
        return -1;
    }
}

/*****************************************************************************
*  Name: fts_proximity_resume
*  Brief: Run when tp resume
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_proximity_resume(void)
{
    if (fts_proximity_data.mode == 1 && fts_proximity_suspend_flag == 1)
        return 0;
    else
        return -1;
}
#endif /* FTS_PSENSOR_EN */

