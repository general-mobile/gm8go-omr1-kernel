/*
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>

#include <linux/io.h>
#include "upmu_sw.h"
#include "upmu_common.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h>
#include <linux/sched.h>

#include <alsps.h>
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif

/*----------------------------------------------------------------------------*/
#define APS_TAG                 	  	"[DUMMY_PS] "
#define APS_FUN(f)              	  	printk(KERN_ERR APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_APS_ERR(fmt, args...)    	    printk(KERN_ERR fmt, ##args)

extern struct alsps_context *alsps_context_obj;
extern int ps_data_report(int value, int status);

static int alsps_local_init(void);
static int alsps_remove(void);

static struct alsps_init_info touch_ps_sensor_init_info = {
		.name = "touch_ps",
		.init = alsps_local_init,
		.uninit = alsps_remove,	
};

static int enable_ps(int enable)
{
  printk("avoid operate null pointer due to the undetected touchpanel!");
  return 0;
}

int (*touchpanel_enable_ps)(int enable) = &enable_ps;

void touch_ps_data_report(int value)
{
    ps_data_report(value, SENSOR_STATUS_ACCURACY_MEDIUM);
}

static ssize_t touch_ps_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
    /*get ps value from tp driver*/
	res = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", res);
}

static DRIVER_ATTR(ps, S_IWUSR | S_IRUGO, touch_ps_show_ps,NULL);

static struct driver_attribute *touch_ps_attr_list[] = {
    &driver_attr_ps,
};

static int touch_ps_sensor_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(touch_ps_attr_list)/sizeof(touch_ps_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, touch_ps_attr_list[idx])))
        {
            printk("driver_create_file (%s) = %d\n", touch_ps_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
#if 0
static int touch_ps_sensor_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(touch_ps_attr_list)/sizeof(touch_ps_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, touch_ps_attr_list[idx]);
    }

    return err;
}
#endif
/*--------------------------------------------------------------------------------*/
static int ps_open_report_data(int open)
{
    APS_FUN();
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_enable_nodata(int en)
{
    APS_FUN();
    printk("proximity enable %d\n",en);
    (*touchpanel_enable_ps)(en);
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
    APS_FUN();
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{
    
    APS_FUN();
    
    *value = 0;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;    
	return 0;
}
/*----------------------------------------------------------------------------*/

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;
	/*FIX ME*/

	APS_LOG("epl259x ps setdelay = (%d) ok.\n", value);
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}
static int register_touch_alsps(void)
{
    int err;
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};

    ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
    ps_ctl.batch = ps_batch;
    ps_ctl.flush = ps_flush;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
	ps_ctl.is_support_batch = false;
#endif

    printk("%s line %d\n",__func__,__LINE__);
    
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		return -1;
	}
    
    printk("%s line %d\n",__func__,__LINE__);
	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
    
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		return -1;
	}
    
    printk("%s line %d\n",__func__,__LINE__);
    
	//err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	//if(err)
	//{
	//	printk("register proximity batch support err = %d\n", err);
	//	return -1;
	//}
    
    printk("%s line %d\n",__func__,__LINE__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
    int ret;
    if((ret = touch_ps_sensor_create_attr(&touch_ps_sensor_init_info.platform_diver_addr->driver)))
    {
        printk("create attribute err = %d\n", ret);
        //goto exit_create_attr_failed;
    }

    ret = register_touch_alsps();
    
	return ret;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
    APS_FUN();
    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init touch_ps_sensor_init(void)
{
    APS_FUN();
    alsps_driver_add(&touch_ps_sensor_init_info);
    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit touch_ps_sensor_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(touch_ps_sensor_init);
module_exit(touch_ps_sensor_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yangzhihong@vanzotec.com");
MODULE_DESCRIPTION("Touch ps Sensor driver");
MODULE_LICENSE("GPL");
