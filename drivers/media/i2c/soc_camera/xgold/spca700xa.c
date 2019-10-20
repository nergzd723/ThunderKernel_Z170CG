/*
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * */


#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/freezer.h>

#include "spca700xa.h"
#include "app_i2c_lib_icatch.h"


#define SPCA700XA_SPI_BITS_PER_WORD	8
#define SPCA700XA_CMD_NORMAL		0x55


struct spca700xa *isp;


struct spca700xa_spi_rd {
	struct spi_transfer	spi_xfer;
	u32			spi_tx;
	u32			spi_rx;
};

struct spca700xa {
	struct spi_device	*spi;
	struct spi_message  spi_read_msg;
	struct mutex		mutex;
	spinlock_t		lock;
};


static int spca700xa_cmd(struct spca700xa *isp, UINT8 *cmd, UINT32 length)
{
	UINT8 *tx = cmd;
	struct spi_transfer xfer = {
		.tx_buf		= tx,
		.len		= length,
		.bits_per_word	= 8,
		.cs_change	= 1,
	};
	struct spi_message msg;
	int error;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	error = spi_sync(isp->spi, &msg);
	if (error) {
		printk("%s: error\n", __func__);
		dev_err(&isp->spi->dev, "%s: failed, command: %x, error: %d\n",
			__func__, *cmd, error);
		return error;
	}

	return 0;
}

#define BYTES_PER_XFER 16

static int spca700xa_read(struct spca700xa *isp, UINT8 *value, UINT32 ulTransByteCnt)
{
        //struct spca700xa_spi_rd spi_rd;
	struct spi_message msg;
	int error, i = 0, remaining = ulTransByteCnt;
        u8 *tx;
        u8 *rx = value;
        tx = kzalloc(ulTransByteCnt, GFP_KERNEL);
        printk("In spca700xa_read, initial is value: %x:\n", *value);

	isp->spi->mode |= SPI_CPOL;
	isp->spi->mode |= SPI_CPHA;
	spi_setup(isp->spi);
#if 0
        //ASUS_BSP+++, for calibration
        struct spi_transfer xfer = {
               .tx_buf = tx,
               .rx_buf = rx,
               .len = ulTransByteCnt,
               .bits_per_word = 8,
               .cs_change = 0,
        };
#endif
        printk("In %s:\n", __func__);
        //ASUS_BSP---, for calibration

	for (i = 0; remaining > 0;remaining -= BYTES_PER_XFER) {
		struct spi_transfer xfer = {
		       .tx_buf = tx + i * BYTES_PER_XFER,
		       .rx_buf = rx + i * BYTES_PER_XFER,
		       .len = (remaining > BYTES_PER_XFER) ? BYTES_PER_XFER :remaining,
		       .bits_per_word = 8,
		       .cs_change = 0,
		};

		spi_message_init(&msg);
        	spi_message_add_tail(&xfer, &msg);//ASUS_BSP+++, for calibration
        	//spi_message_add_tail(&spi_rd.spi_xfer, &msg);
		error = spi_sync(isp->spi, &msg);
		i++;
	}
	
        printk("In %s: spi_syn done\n", __func__);
        printk("In spca700xa_read, after is value: %x:\n", *value);
        printk("In spca700xa_read, Error is %d:\n", error);
        kfree(tx);
        if (error) { //ASUS_BSP+++, for calibration
                dev_err(&isp->spi->dev, "%s: failed, command: %x, error: %d\n",
                __func__, *value, error);
                return error;
        }

	return 0;
}

#define testsize 65536
#define read_spi_test 0
int spca700xa_SPI_write(UINT8 *ucStartAddr, UINT32 ulTransByteCnt)
{
	int size,i;
	UINT8 pattern[testsize];
	UINT16 p2 = 0;
	pattern[0]=0x01;
	pattern[1]=0x01;
	pattern[2]=0x02;
	pattern[3]=0x02;
	pattern[4]=0x03;
	int sum=0;

	size=sizeof(pattern);

	//printk("%s: ulTransByteCnt = %d\n", __func__, ulTransByteCnt);
#if read_spi_test
	for (i=0; i<testsize; i++) {
		pattern[i]=i;
		sum+=pattern[i];
		/*if (((i mod 2) && (i > 0)))
		{
			p2+= i << 8 + i-1;
		}*/
	}

	//for (i=0; i<10; i++) {
		spca700xa_cmd(isp, &pattern[0], testsize);
	//}
	printk("========== XXXX ====  %s: sum=%x  testsize=%d p2=%x\n", __func__, sum, testsize, p2);
#else
	spca700xa_cmd(isp, ucStartAddr, ulTransByteCnt);
/*
	for (i=0;i<ulTransByteCnt;i++){
		spca700xa_cmd_byte(isp, ucStartAddr[i]);
	}
*/
#endif





	return 0;
}

EXPORT_SYMBOL(spca700xa_SPI_write);

int spca700xa_SPI_read(UINT8 *ucStartAddr, UINT32 ulTransByteCnt)
{
        printk("In %s: ulTransByteCnt = %d, FW Address: 0x%x\n", __func__, ulTransByteCnt, *ucStartAddr);
	spca700xa_read(isp, ucStartAddr, ulTransByteCnt);
/*
	for (i = 0x1390; i < 0x13b0; i++) {
		printk("%s: FW Address for read [%d]: %x\n", __func__, i, ucStartAddr[i]);
	}
*/
	return 0;
}

EXPORT_SYMBOL(spca700xa_SPI_read);

//For Wesley test +++ 2 function
int spca700xa_SPI_clk_control(bool reset)
{
	printk("WesleySPI_clk_control ==== in\n");
	spi_bus_reset(isp->spi, reset);
	printk("WesleySPI_clk_control ==== out\n");
	return 0;
}

EXPORT_SYMBOL(spca700xa_SPI_clk_control);
//For Wesley test --- 2 function

int spca700xa_SPI_off(bool off){
    spi_bus_off(isp->spi, off);
    return 0;
}

EXPORT_SYMBOL(spca700xa_SPI_off);

extern unsigned int entry_mode;

static int spca700xa_probe(struct spi_device *spi)
{
	int error=0;

        if (entry_mode != 1) {
                printk("%s: return because entry_mode!=1\n", __func__);
                return error;
        }

	printk("==== WesleySPI probe ==== %s:\n", __func__);
	/* Set up SPI*/
	spi->bits_per_word = SPCA700XA_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	spi->chip_select = 0; //Wesley, default 0
	//spi->inter_character_pause = 1;
	error = spi_setup(spi);
	if (error < 0) {
		dev_err(&spi->dev, "%s: SPI setup error %d\n",
			__func__, error);
		return error;
	}else{
		printk("%s success\n", __func__);
	}
	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	isp->spi = spi;

	spi_set_drvdata(spi, isp);

	return 0;
}

static int spca700xa_remove(struct spi_device *spi)
{
	return 0;
}

#ifdef CONFIG_PM

static int spca700xa_suspend(struct spi_device *spi, pm_message_t state)
{
	return 0;
}

static int spca700xa_resume(struct spi_device *spi)
{
	return 0;
}

#else
#define spca700xa_suspend NULL
#define spca700xa_resume  NULL
#endif

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "icatch,spca700spi" },
	{},
};

MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static struct spi_driver spca700xa_driver = {
	.driver = {
		.name		= "spca700xa",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},

	.probe		= spca700xa_probe,
	.remove		= spca700xa_remove,
	.suspend	= spca700xa_suspend,
	.resume		= spca700xa_resume,
};

static int __init spca700xa_init(void)
{
	return spi_register_driver(&spca700xa_driver);
}
module_init(spca700xa_init);

static void __exit spca700xa_exit(void)
{
	spi_unregister_driver(&spca700xa_driver);
}
module_exit(spca700xa_exit);

MODULE_DESCRIPTION("SPCA700XA driver");
MODULE_AUTHOR("ASUS SW3");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spca700xa");
