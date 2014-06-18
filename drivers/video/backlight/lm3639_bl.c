/* drivers/video/backlight/lm3639_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/leds.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>

#include <mach/board_lge.h>
#include <linux/earlysuspend.h>

//                                                                
#define MAX_BRIGHTNESS_LM3639 			0xFF
//                                                                
#define DEFAULT_BRIGHTNESS 				0x73
#define DEFAULT_FTM_BRIGHTNESS			0x2B

#define I2C_BL_NAME "lm3639_bled"
#define I2C_FLASH_NAME "lm3639_flash"
#define I2C_TORCH_NAME "lm3639_torch"

#define BL_ON	1
#define BL_OFF	0

static struct i2c_client *lm3639_i2c_client;
//                                                  
#if 0//def CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
#endif
//                                                  

struct backlight_platform_data {
   void (*platform_init)(void);
   int gpio;
   unsigned int mode;
   int max_current;
   int init_on_boot;
   int min_brightness;
   int max_brightness;
   int default_brightness;
   int factory_brightness;
};

struct lm3639_device {
	struct i2c_client *client;
	struct backlight_device *bl_dev;
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	int gpio;
	int max_current;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int factory_brightness;
	struct mutex bl_mutex;
};

static const struct i2c_device_id lm3639_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};

static int cur_main_lcd_level = DEFAULT_BRIGHTNESS;
//                                                  
//static int saved_main_lcd_level = DEFAULT_BRIGHTNESS;
//                                                  
static int backlight_status = BL_OFF;
static struct lm3639_device *main_lm3639_dev;

static int lm3639_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *data)
{
	int err;
	struct i2c_msg msg[] = {
                        {
		client->addr, 0, 1, &reg
                        },
                        {
		client->addr, I2C_M_RD, 1, data
                        },
	};

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0) {
		dev_err(&client->dev, "i2c read error\n");
		return err;
	}
	return 0;
}

static int lm3639_write_reg(struct i2c_client *client, unsigned char reg, unsigned char data)
{
	int err;
	unsigned char buf[2];
	struct i2c_msg msg = {
		client->addr, 0, 2, buf
	};

	buf[0] = reg;
	buf[1] = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0) {
		dev_err(&client->dev, "i2c write error\n");
	}
	return 0;
}

//                                                  
//static int exp_min_value = 150;
//                                                  
static int cal_value;

static char mapped_value[] = {
                10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                10, 10, 10, 11, 11, 11, 11, 12, 12, 12,
                13, 13, 13, 14, 14, 15, 15, 16, 16, 17,
                17, 18, 19, 19, 20, 21, 21, 22, 23, 24,
                24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
                34, 35, 36, 38, 39, 40, 41, 43, 44, 45,
                47, 48, 49, 51, 52, 54, 55, 57, 59, 60,
                62, 64, 65, 67, 69, 71, 73, 74, 76, 78,
                80, 82, 84, 86, 89, 91, 93, 95, 97, 100,
                102, 104, 106, 109, 111, 114, 116, 119, 121, 124,
                127
};

static void lm3639_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm3639_device *dev;
	enum lge_boot_mode_type bootmode = lge_get_boot_mode();
	unsigned int bright_per = 0;


	dev = (struct lm3639_device *)i2c_get_clientdata(client);

	if(bootmode == LGE_BOOT_MODE_FACTORY2 || bootmode == LGE_BOOT_MODE_PIFBOOT)
		level = dev->factory_brightness;

//                                                                                             
	if (level == -1)
		level = cur_main_lcd_level; //dev->default_brightness;
//                                                                                             

	cur_main_lcd_level = level;
	dev->bl_dev->props.brightness = cur_main_lcd_level;

	mutex_lock(&main_lm3639_dev->bl_mutex);
	if (level != 0) {
//                                           
                        //Gamma 2.2 Table adapted
//                        bright_per = (level - (unsigned int)20) *(unsigned int)100 / (unsigned int)235;
                        if( (level -20) > 0) {
                                bright_per = (level - (unsigned int)20) *(unsigned int)100 / (unsigned int)235;
                        } else {
                                bright_per = 0;
                        }
//                                           
                        cal_value = mapped_value[bright_per];

                        if (backlight_status == BL_OFF) {
//                                                          
                                lm3639_write_reg(main_lm3639_dev->client, 0x02, 0x54);
//                                                          
//                                                                                    
                                lm3639_write_reg(main_lm3639_dev->client, 0x03, 0x01);
//                                                                                    
                                lm3639_write_reg(main_lm3639_dev->client, 0x05, cal_value);
                                lm3639_write_reg(main_lm3639_dev->client, 0x0A, 0x11);
                        }
                        lm3639_write_reg(main_lm3639_dev->client, 0x05, cal_value);

	} else {
		lm3639_write_reg(main_lm3639_dev->client, 0x05, 0x00);
		lm3639_write_reg(main_lm3639_dev->client, 0x0A, 0x00);
	}
	mutex_unlock(&main_lm3639_dev->bl_mutex);
}

void lm3639_backlight_on(int level)
{
        int gpio = main_lm3639_dev->gpio;

        if (backlight_status == BL_OFF) {
                gpio_direction_output(gpio, 1);
//                                                     
                mdelay(10);
//                                                     

        }

        lm3639_set_main_current_level(main_lm3639_dev->client, level);
        backlight_status = BL_ON;

        return;
}

void lm3639_backlight_off(void)
{
	int gpio = main_lm3639_dev->gpio;

	if (backlight_status == BL_OFF)
		return;

//                                                  
//	saved_main_lcd_level = cur_main_lcd_level;
//                                                  
	lm3639_set_main_current_level(main_lm3639_dev->client, 0);
	backlight_status = BL_OFF;

	gpio_direction_output(gpio, 0);
//                                                  
//	msleep(6);
//                                                  

	return;
}

void lm3639_lcd_backlight_set_level(int level)
{
	if (level > MAX_BRIGHTNESS_LM3639)
		level = MAX_BRIGHTNESS_LM3639;

	if (lm3639_i2c_client != NULL) {
		if (level == 0) {
			lm3639_backlight_off();
		} else {
			lm3639_backlight_on(level);
		}
	} else {
		pr_err("%s(): No client\n", __func__);
	}
}
EXPORT_SYMBOL(lm3639_lcd_backlight_set_level);


static int bl_set_intensity(struct backlight_device *bd)
{

	struct i2c_client *client = to_i2c_client(bd->dev.parent);

	lm3639_set_main_current_level(client, bd->props.brightness);
	cur_main_lcd_level = bd->props.brightness;

	return 0;
}

static int bl_get_intensity(struct backlight_device *bd)
{
    unsigned char val = 0;

    val &= 0x1f;
    return (int)val;
}

//                                                  
#if 0
static ssize_t lcd_backlight_show_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n", cal_value);

	return r;
}

static ssize_t lcd_backlight_store_level(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int level;
	struct i2c_client *client = to_i2c_client(dev);

	if (!count)
		return -EINVAL;

	level = simple_strtoul(buf, NULL, 10);

	if (level > MAX_BRIGHTNESS_LM3639)
		level = MAX_BRIGHTNESS_LM3639;

	lm3639_set_main_current_level(client, level);
	cur_main_lcd_level = level;

	return count;
}

static int lm3639_bl_resume(struct i2c_client *client)
{
    lm3639_backlight_on(saved_main_lcd_level);
    return 0;
}

static int lm3639_bl_suspend(struct i2c_client *client, pm_message_t state)
{
    printk(KERN_INFO"%s: new state: %d\n", __func__, state.event);

    lm3639_lcd_backlight_set_level(saved_main_lcd_level);

    return 0;
}

static ssize_t lcd_backlight_show_on_off(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	pr_info("%s received (prev backlight_status: %s)\n", __func__, backlight_status ? "ON" : "OFF");

	return r;
}

static ssize_t lcd_backlight_store_on_off(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int on_off;
	struct i2c_client *client = to_i2c_client(dev);

	if (!count)
		return -EINVAL;

	pr_info("%s received (prev backlight_status: %s)\n", __func__, backlight_status ? "ON" : "OFF");

	on_off = simple_strtoul(buf, NULL, 10);

	printk(KERN_ERR "%d", on_off);

	if (on_off == 1) {
                        lm3639_bl_resume(client);
	} else if (on_off == 0)
                        lm3639_bl_suspend(client, PMSG_SUSPEND);

	return count;

}
static ssize_t lcd_backlight_show_exp_min_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight  : %d\n", exp_min_value);

	return r;
}

static ssize_t lcd_backlight_store_exp_min_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (!count)
		return -EINVAL;

	value = simple_strtoul(buf, NULL, 10);
	exp_min_value = value;

	return count;
}

DEVICE_ATTR(lm3639_level, 0644, lcd_backlight_show_level, lcd_backlight_store_level);
DEVICE_ATTR(lm3639_backlight_on_off, 0644, lcd_backlight_show_on_off, lcd_backlight_store_on_off);
DEVICE_ATTR(lm3639_exp_min_value, 0644, lcd_backlight_show_exp_min_value, lcd_backlight_store_exp_min_value);
#endif
//                                                  

static struct backlight_ops lm3639_bl_ops = {
	.update_status = bl_set_intensity,
	.get_brightness = bl_get_intensity,
};

//                                                  
#if 0//def CONFIG_HAS_EARLYSUSPEND
static void lm3639_early_suspend(struct early_suspend *h)
{
        int err = 0;
        unsigned char data;
        int gpio = main_lm3639_dev->gpio;

        data = 0x00; //backlight2 brightness 0
        err = lm3639_write_reg(main_lm3639_dev->client, 0x05, data);

        data = 0x00; //backlight disable
        err = lm3639_write_reg(main_lm3639_dev->client, 0x0A, data);

        gpio_direction_output(gpio, 0);
}
static void lm3639_late_resume(struct early_suspend *h)
{
        int err = 0;
        unsigned char data1;
        int gpio = main_lm3639_dev->gpio;

        gpio_direction_output(gpio, 1); //0 -> 1
        mdelay(50);

        err = lm3639_write_reg(main_lm3639_dev->client, 0x05, cur_main_lcd_level);

        data1 =0x11;//backlight enable 0x19 -> 0x11
        err = lm3639_write_reg(main_lm3639_dev->client, 0x0A, data1);
}
#endif
//                                                  

/* Flash control for L9II */
/*                                */

/*	 Flash Current
	 0000 : 46.875 mA		1000 : 421.875 mA
	 0001 : 93.75 mA 		1001 : 468.25 mA
	 0010 : 140.625 mA 	1010 : 515.625 mA
	 0011 : 187.5 mA  		1011 : 562.5 mA
	 0100 : 234.375 mA	1100 : 609.375 mA
	 0101 : 281.25 mA		1101 : 659.25 mA
	 0110 : 328.125 mA	1110 : 703.125 mA
	 0111 : 375 mA		1111 : 750 mA

	 Torch Current
	 000 : 28.125 mA		100 : 140.625 mA
	 001 : 56.25 mA		101 : 168.75 mA
	 010 : 84.375 mA		110 : 196.875 mA
	 011 : 112.5 mA		111 : 225mA

	 Frequency [7]
	 0 : 2 Mhz (default)
	 1 : 4 Mhz

	 Current Limit [6:5]
	 00 : 1.7 A			10 : 2.5 A (default)
	 01 : 1.9 A			11 : 3.1 A

	 Time out [4:0]
	 00000 : 32 ms
	 01111 : 512 ms (default)
	 11111 : 1024 ms
*/

#define LM3639_POWER_OFF        0
#define LM3639_POWER_ON         1
#define REG_ENABLE				0x0A
#define REG_IO_CTRL				0x09
#define REG_FL_CONF_1			0x06
#define REG_FL_CONF_2			0x07

static int lm3639_onoff_state = LM3639_POWER_OFF;

enum led_status {
	LM3639_LED_OFF,
	LM3639_LED_LOW,
	LM3639_LED_HIGH,
	LM3639_LED_MAX
};

unsigned char strobe_ctrl;
unsigned char flash_ctrl;



void lm3639_enable_torch_mode(enum led_status state)
{
		int err = 0;

        unsigned char data1;
        unsigned char torch_curr = 0;
        unsigned char strobe_curr = 0;

        unsigned char data2;
        unsigned char fled_sw_freq = 0x00; //default 2Mhz
        unsigned char fled_curr_limit = 0x00; //1.7A
        unsigned char strobe_timeout = 0x0F; //1024ms

        pr_err("%s: state = %d\n", __func__, state);

        if (state == LM3639_LED_LOW) {
                torch_curr = 0x30; // 112.5 mA
                strobe_curr = 0x01; // 93.75 mA
        } else {
                torch_curr = 0x00;
                strobe_curr = 0x00;
        }
		/* Configuration of frequency, current limit and timeout */
		data2 =  fled_sw_freq | fled_curr_limit | strobe_timeout;
        err = lm3639_write_reg(main_lm3639_dev->client, REG_FL_CONF_2, data2);

		/* Configuration of current */
		data1 = torch_curr | strobe_curr;
        err = lm3639_write_reg(main_lm3639_dev->client, REG_FL_CONF_1, data1);

		/* Control of I/O register */
        strobe_ctrl=0;
        err = lm3639_read_reg(main_lm3639_dev->client, REG_IO_CTRL, &strobe_ctrl);
        strobe_ctrl &= 0xED; /* 1110 1101 */
        err = lm3639_write_reg(main_lm3639_dev->client, REG_IO_CTRL, strobe_ctrl);

		/* Enable */
        flash_ctrl=0;
        err = lm3639_read_reg(main_lm3639_dev->client, REG_ENABLE, &flash_ctrl);
        flash_ctrl |= 0x42; /* 0100 0010 */
        err = lm3639_write_reg(main_lm3639_dev->client, REG_ENABLE, flash_ctrl);

}



void lm3639_enable_flash_mode(enum led_status state)
{
        int err = 0;

        unsigned char data1;
        unsigned char torch_curr = 0;
        unsigned char strobe_curr = 0;

        unsigned char data2;
        unsigned char fled_sw_freq = 0x00; //default 2Mhz
        unsigned char fled_curr_limit = 0x10; //2.5A
        unsigned char strobe_timeout = 0x0F; //1024ms

        pr_err("%s: state = %d\n", __func__, state);

        if (state == LM3639_LED_LOW) {
                torch_curr = 0x10; // 56.25 mA
                strobe_curr = 0x01; // 93.75 mA
        } else {
                torch_curr = 0x70; // 225mA
                strobe_curr = 0x0F; // 750 mA
        }

		/* Configuration of frequency, current limit and timeout */
		data2 =  fled_sw_freq | fled_curr_limit | strobe_timeout;
        err = lm3639_write_reg(main_lm3639_dev->client, REG_FL_CONF_2, data2);

		/* Configuration of current */
		data1 = torch_curr | strobe_curr;
        err = lm3639_write_reg(main_lm3639_dev->client, REG_FL_CONF_1, data1);

		/* Control of I/O register */
        strobe_ctrl=0;
        err = lm3639_read_reg(main_lm3639_dev->client, REG_IO_CTRL, &strobe_ctrl);
        strobe_ctrl &= 0xED; /* 1110 1101 */
        err = lm3639_write_reg(main_lm3639_dev->client, REG_IO_CTRL, strobe_ctrl);

		/* Enable */
        flash_ctrl=0;
        err = lm3639_read_reg(main_lm3639_dev->client, REG_ENABLE, &flash_ctrl);
        flash_ctrl |= 0x66; /* 0110 0110*/
        err = lm3639_write_reg(main_lm3639_dev->client, REG_ENABLE, flash_ctrl);

}

void lm3639_config_gpio_on(void)
{
        pr_err("%s: Start\n", __func__);
}

void lm3639_config_gpio_off(void)
{
        pr_err("%s: Start\n", __func__);
}

void lm3639_led_enable(void)
{
        pr_err("%s: Start\n", __func__);

        lm3639_onoff_state = LM3639_POWER_ON;
}

void lm3639_led_disable(void)
{
        int err = 0;

        pr_err("%s: Start\n", __func__);

        flash_ctrl = 0;
        strobe_ctrl = 0;

        err = lm3639_read_reg(main_lm3639_dev->client, REG_IO_CTRL, &strobe_ctrl);
        strobe_ctrl &= 0xCF;
        err = lm3639_write_reg(main_lm3639_dev->client, REG_IO_CTRL, strobe_ctrl);


        err = lm3639_read_reg(main_lm3639_dev->client, REG_ENABLE, &flash_ctrl);
        flash_ctrl &= 0x99;
        err = lm3639_write_reg(main_lm3639_dev->client, REG_ENABLE, flash_ctrl);

        lm3639_onoff_state = LM3639_POWER_OFF;
}

int lm3639_flash_set_led_state(int led_state)
{
	int rc = 0;
	int batt_temp = 0;

	pr_err("%s: led_state = %d\n", __func__, led_state);

	switch (led_state) {
	case MSM_CAMERA_LED_OFF:
		lm3639_led_disable();
		break;
	case MSM_CAMERA_LED_LOW:
		lm3639_led_enable();
		lm3639_enable_torch_mode(LM3639_LED_LOW);
		break;
	case MSM_CAMERA_LED_HIGH:
		lm3639_led_enable();
		batt_temp = pm8921_batt_temperature();
		if(batt_temp > -100) {
			pr_err("%s: Working on LED_HIGH Flash mode (Battery temperature = %d)\n", __func__, batt_temp);
			lm3639_enable_flash_mode(LM3639_LED_HIGH);
		} else {
			pr_err("%s: Working on LED_LOW Flash mode (Battery temperature = %d)\n", __func__, batt_temp);
			lm3639_enable_flash_mode(LM3639_LED_LOW);
		}
		break;
	case MSM_CAMERA_LED_INIT:
		lm3639_config_gpio_on();
		break;
	case MSM_CAMERA_LED_RELEASE:
		lm3639_led_disable();
		//lm3639_config_gpio_off();
		break;
	default:
		rc = -EFAULT;
		break;
	}

	return rc;
}
EXPORT_SYMBOL(lm3639_flash_set_led_state);

/* torch */
static void lm3639_torch_led_set(struct led_classdev *led_cdev,
					enum led_brightness value)
{
        pr_err("%s: led_cdev->brightness[%d]\n", __func__, value);

        led_cdev->brightness = value;

        if(value)
                lm3639_enable_torch_mode(LM3639_LED_LOW);
        else
                lm3639_led_disable();
}

/* flash */
static void lm3639_flash_led_set(struct led_classdev *led_cdev,
					enum led_brightness value)
{
        pr_err("%s: led_cdev->brightness[%d]\n", __func__, value);

        led_cdev->brightness = value;

        if(value)
                lm3639_enable_flash_mode(LM3639_LED_LOW);
        else
                lm3639_led_disable();
}

static int lm3639_probe(struct i2c_client *i2c_dev, const struct i2c_device_id *id)
{
	struct backlight_platform_data *pdata;
	struct lm3639_device *dev;
	struct backlight_device *bl_dev;
	struct backlight_properties props;

	int err;

	pr_info("%s: i2c probe start\n", __func__);

	pdata = i2c_dev->dev.platform_data;
	lm3639_i2c_client = i2c_dev;

	dev = kzalloc(sizeof(struct lm3639_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&i2c_dev->dev, "fail alloc for lm3639_device\n");
		return 0;
	}

	pr_info("%s: gpio = %d\n", __func__,pdata->gpio);

	gpio_tlmm_config(GPIO_CFG(pdata->gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (pdata->gpio && gpio_request(pdata->gpio, "lm3639_bl_en") != 0) {
		return -ENODEV;
	}

	main_lm3639_dev = dev;

	memset(&props, 0, sizeof(struct backlight_properties));

	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS_LM3639;
	bl_dev = backlight_device_register(I2C_BL_NAME, &i2c_dev->dev, NULL, &lm3639_bl_ops, &props);
	bl_dev->props.max_brightness = MAX_BRIGHTNESS_LM3639;
	bl_dev->props.brightness = DEFAULT_BRIGHTNESS;
	bl_dev->props.power = FB_BLANK_UNBLANK;

	dev->bl_dev = bl_dev;
	dev->client = i2c_dev;
	dev->gpio = pdata->gpio;
	dev->max_current = pdata->max_current;
	dev->min_brightness = pdata->min_brightness;
	dev->default_brightness = pdata->default_brightness;
	dev->max_brightness = pdata->max_brightness;
	i2c_set_clientdata(i2c_dev, dev);

	if(pdata->factory_brightness <= 0)
	        dev->factory_brightness = DEFAULT_FTM_BRIGHTNESS;
	else
	        dev->factory_brightness = pdata->factory_brightness;


	mutex_init(&dev->bl_mutex);

//                                                  
#if 0
	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3639_level);
	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3639_backlight_on_off);
	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3639_exp_min_value);
#endif
//                                                  

	/* flash */
	dev->cdev_flash.name = I2C_FLASH_NAME;
	dev->cdev_flash.max_brightness = 16;
	dev->cdev_flash.brightness_set = lm3639_flash_led_set;
	err = led_classdev_register((struct device *)&i2c_dev->dev, &dev->cdev_flash);

	/* torch */
	dev->cdev_torch.name = I2C_TORCH_NAME;
	dev->cdev_torch.max_brightness = 8;
	dev->cdev_torch.brightness_set = lm3639_torch_led_set;
	err = led_classdev_register((struct device *)&i2c_dev->dev, &dev->cdev_torch);

	mutex_init(&dev->bl_mutex);

//                                                  
#if 0 //def CONFIG_HAS_EARLYSUSPEND
	early_suspend.suspend = lm3639_early_suspend;
	early_suspend.resume = lm3639_late_resume;

	register_early_suspend(&early_suspend);
#endif
//                                                  

	return 0;

}

static int lm3639_remove(struct i2c_client *i2c_dev)
{
	struct lm3639_device *dev = (struct lm3639_device *)i2c_get_clientdata(i2c_dev);
	int gpio = main_lm3639_dev->gpio;

	led_classdev_unregister(&dev->cdev_torch);
	led_classdev_unregister(&dev->cdev_flash);

//                                                  
#if 0
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3639_level);
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3639_backlight_on_off);
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3639_exp_min_value);
#endif
//                                                  

	backlight_device_unregister(dev->bl_dev);

	i2c_set_clientdata(i2c_dev, NULL);

	if (gpio_is_valid(gpio))
		gpio_free(gpio);
	return 0;
}

static struct i2c_driver main_lm3639_driver = {
	.probe = lm3639_probe,
	.remove = lm3639_remove,
	.suspend = NULL,
	.resume = NULL,
	.id_table = lm3639_bl_id,
	.driver = {
		.name = I2C_BL_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lcd_backlight_init(void)
{
	static int err;

	err = i2c_add_driver(&main_lm3639_driver);

	return err;
}

module_init(lcd_backlight_init);

MODULE_DESCRIPTION("LM3639 Backlight Control");
MODULE_AUTHOR("Jaeseong Gim <jaeseong.gim@lge.com>");
MODULE_LICENSE("GPL");
