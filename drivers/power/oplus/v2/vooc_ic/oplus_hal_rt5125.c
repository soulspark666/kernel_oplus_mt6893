// SPDX-License-Identifier: GPL-2.0-only
/*
 * RT5125 VOOC ASIC HAL for the v2 charger framework (MT6893/Cupida).
 *
 * WP-B of V2_CHARGER_PORT_PLAN. Ported from
 * drivers/power/oplus/v1/vooc_ic/oplus_rt5125.c; interface skeleton follows
 * drivers/power/oplus/v2/vooc_ic/oplus_hal_rk826.c.
 *
 * LAYERING (v2): this HAL owns ONLY the FW/I2C path (page program, CRC16
 * verify, version read, upgrade/check state). All GPIO/bit-bang sequencing
 * (clock/data/reset/switch/eint) lives in hal/oplus_virtual_asic.c and is
 * reached through the parent-delegating shims below — the timing-critical
 * bodies were NOT re-ported here, they are consumed verbatim from the
 * virtual-asic layer fed by the WP-A DTS.
 *
 * DELIBERATE OMISSIONS (unimplemented v2 funcs return -ENOTSUPP by design):
 * - VOOC_USER_FW_UPGRADE / _FW_CHECK_THEN_RECOVER_FIX: v1 rt5125 has no
 *   source (ops slots NULL). oplus_chg_vooc.c:3281 calls _FIX and logs on
 *   error — tolerated, watch at WP-E.
 * - VOOC_CHECK_FW_STATUS: RT5125 has no ELDI-style status register; the
 *   v2 caller (oplus_hal_vooc.c:467) treats -ENOTSUPP as pass=true.
 * - Multi-variant FW arrays (vooc_fw_type, 7 blobs): single DTS blob
 *   (oplus,firmware_data on rt5125_fastcg) covers 21127. Revisit if another
 *   project needs a second variant.
 * - v1 plat_ops/gauge caching (the_bat, 400ms temp/soc work): gauge is
 *   owned by WP-C's virtual_gauge binding, not the VOOC HAL.
 * - v1 shutdown parking (switch-to-normal + clock/reset park): rk826 has
 *   an empty shutdown; parking belongs to the virtual-asic exit path.
 * - No separate .h (matches rk826/op10 precedent: vooc_ic HALs are .c-only;
 *   everything is static except the module init/exit pair).
 */

#define pr_fmt(fmt) "[RT5125]([%s][%d]): " fmt, __func__, __LINE__

#define VOOC_ASIC_RT5125

#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/version.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>

#include <linux/module.h>
#include <soc/oplus/device_info.h>

#else
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <soc/oplus/device_info.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_chg_ic.h>
#include <oplus_hal_vooc.h>

struct rt5125_chip {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;

	const unsigned char *firmware_data;
	unsigned int fw_data_count;
	int fw_data_version;
	int fw_mcu_version;

	bool upgrading;		/* ex v1 mcu_update_ing */
	bool boot_by_gpio;	/* ex v1 mcu_boot_by_gpio */
	bool vooc_fw_check;
	bool vooc_fw_update_newmethod;
	int reset_status;	/* ex v1 the_bat.reset_status */
	struct wakeup_source *update_wake_lock;
};

extern int charger_abnormal_log;

/* #ifdef CONFIG_OPLUS_CHARGER_MTK */
#define I2C_MASK_FLAG (0x00ff)
#define I2C_ENEXT_FLAG (0x0200)
/* #endif */

#define GTP_DMA_MAX_TRANSACTION_LENGTH 255 /* for DMA mode */
#define GTP_SUPPORT_I2C_DMA 0

#define FW_CHECK_FAIL 0
#define FW_CHECK_SUCCESS 1

/* ---- I2C transport (verbatim from v1 oplus_rt5125.c:216-284, MTK branch,
 * GTP_SUPPORT_I2C_DMA=0; file-static = per-HAL state) ---- */
#ifdef CONFIG_OPLUS_CHARGER_MTK
DEFINE_MUTEX(dma_wr_access_rt5125);

static char gpDMABuf_pa[GTP_DMA_MAX_TRANSACTION_LENGTH] = { 0 };

static int oplus_i2c_dma_read(struct i2c_client *client, u16 addr, s32 len,
			      u8 *rxbuf)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1] = { 0 };
	struct i2c_msg msg[2] = {
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = 0,
		 .buf = buffer,
		 .len = 1,
		 },
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = I2C_M_RD,
		 .buf = (__u8 *) gpDMABuf_pa,
		 .len = len,
		 },
	};
	mutex_lock(&dma_wr_access_rt5125);
	buffer[0] = (u8) (addr & 0xFF);
	if (rxbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}

	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(rxbuf, gpDMABuf_pa, len);
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}

static int oplus_i2c_dma_write(struct i2c_client *client, u16 addr, s32 len,
			       u8 const *txbuf)
{
	int ret = 0;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_pa;
	struct i2c_msg msg = {
		.addr = (client->addr & I2C_MASK_FLAG),
		.flags = 0,
		.buf = (__u8 *) gpDMABuf_pa,
		.len = 1 + len,
	};
	mutex_lock(&dma_wr_access_rt5125);
	wr_buf[0] = (u8) (addr & 0xFF);
	if (txbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}
	memcpy(wr_buf + 1, txbuf, len);

	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			continue;
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}
#endif /*CONFIG_OPLUS_CHARGER_MTK */

static int oplus_vooc_i2c_read(struct i2c_client *client, u8 addr, s32 len,
			       u8 *rxbuf)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if GTP_SUPPORT_I2C_DMA
	return oplus_i2c_dma_read(client, addr, len, rxbuf);
#else
	return i2c_smbus_read_i2c_block_data(client, addr, len, rxbuf);
#endif
#else
	return i2c_smbus_read_i2c_block_data(client, addr, len, rxbuf);
#endif
}

static int oplus_vooc_i2c_write(struct i2c_client *client, u8 addr, s32 len,
				u8 const *txbuf)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if GTP_SUPPORT_I2C_DMA
	return oplus_i2c_dma_write(client, addr, len, txbuf);
#else
	return i2c_smbus_write_i2c_block_data(client, addr, len, txbuf);
#endif
#else
	return i2c_smbus_write_i2c_block_data(client, addr, len, txbuf);
#endif
}

/*****************************************************************/
/* ---- RT5125 MTP protocol (verbatim v1:388-553) ---- */
#define DEFAULT_MAX_BINSIZE	(16 * 1024)
#define DEFAULT_MAX_DATALEN	(128)
#define DEFAULT_MAX_PAGELEN	(128)
#define DEFAULT_MAX_PAGEIDX	(DEFAULT_MAX_BINSIZE / DEFAULT_MAX_PAGELEN)
#define DEFAULT_VERINFO_LEN	(11)
#define DEFAULT_PAGEWR_RETRY	(110)
#define DEFAULT_I2C_RETRY	(5)
/* cmd 1 + data 128 + crc8 */
#define DEFAULT_MAX_BUFFLEN	(1 + DEFAULT_MAX_DATALEN + 1)
#define RT5125_CRC16_INIT	(0xffff)

#define RT5125_PAGE_IDX		(0x10)
#define RT5125_ACCESS_CTL	(0x12)
#define RT5125_STATUS		(0x13)
#define RT5125_DATA_BUF		(0x80)
#define RT5125_FW_CRC16_INFO	(0x90)
#define RT5125_CMD_CRC8_INFO	(0x91)

#define RT5125_MTP_MODE		BIT(0)
#define RT5125_FW_CRC16_RSLT	BIT(3)
#define RT5125_OPSTATUS_MASK	(0x7)
#define RT5125_OPSTATUS_FAIL	(0x4)
#define RT5125_OPSTATUS_SUCCESS	(0x2)
#define RT5125_OPSTATUS_ONGOING	(0x1)
#define RT5125_OPSTATUS_IDLE	(0x0)
#define RT5125_OPSTATUS_FAILMSK	BIT(2)
#define RT5125_WT_PAGE		BIT(7)
#define RT5125_RD_PAGE		BIT(6)
#define RT5125_WT_FW_CRC16	BIT(3)
#define RT5125_FW_CRC16_VRFY	BIT(2)
#define RT5125_WT_KEY		BIT(1)
#define CRC8_TABLE_SIZE	256

static u8 crc8_table[CRC8_TABLE_SIZE];
static DEFINE_MUTEX(data_lock);
static u8 data_buff[DEFAULT_MAX_BUFFLEN];

static u16 const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static u8 crc8(const u8 table[CRC8_TABLE_SIZE], u8 *pdata, size_t nbytes,
	       u8 crc)
{
	/* loop over the buffer data */
	while (nbytes-- > 0)
		crc = table[(crc ^ *pdata++) & 0xff];
	return crc;
}

static void crc8_populate_msb(u8 table[CRC8_TABLE_SIZE], u8 polynomial)
{
	int i, j;
	const u8 msbit = 0x80;
	u8 t = msbit;

	table[0] = 0;
	for (i = 1; i < CRC8_TABLE_SIZE; i *= 2) {
		t = (t << 1) ^ (t & msbit ? polynomial : 0);
		for (j = 0; j < i; j++)
			table[i + j] = table[j] ^ t;
	}
}

static inline u16 crc16_byte(u16 crc, const u8 data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

static u16 crc16(u16 crc, u8 const *buffer, size_t len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
}

static int rt5125_i2c_block_read(struct rt5125_chip *chip, u8 cmd, u8 *data,
				 s32 len)
{
	u8 crc;
	int retry = 0, ret;

	if (len > DEFAULT_MAX_DATALEN || len <= 0)
		return -EINVAL;
	mutex_lock(&data_lock);
retry_read:
	if (retry++ >= DEFAULT_I2C_RETRY) {
		ret = -EIO;
		goto out_read;
	}
	ret = oplus_i2c_dma_read(chip->client, cmd, len + 1, data_buff + 1);
	if (ret < 0)
		goto out_read;
	data_buff[0] = cmd;
	/* verify crc 8 : cmd + data */
	crc = crc8(crc8_table, data_buff, len + 1, 0);
	if (crc != data_buff[len + 1])
		goto retry_read;
	memcpy(data, data_buff + 1, len);
out_read:
	mutex_unlock(&data_lock);
	return ret;
}

static int rt5125_i2c_block_write(struct rt5125_chip *chip, u8 cmd,
				  const u8 *data, s32 len)
{
	int retry = 0, ret;

	if (len > DEFAULT_MAX_DATALEN || len <= 0)
		return -EINVAL;
	mutex_lock(&data_lock);
retry_write:
	if (retry++ >= DEFAULT_I2C_RETRY) {
		ret = -EIO;
		goto out_write;
	}
	data_buff[0] = cmd;
	memcpy(data_buff + 1, data, len);
	data_buff[len + 1] = crc8(crc8_table, data_buff, len + 1, 0);
	ret = oplus_i2c_dma_write(chip->client, cmd, len + 1, data_buff + 1);
	if (ret < 0)
		goto out_write;
	ret = oplus_i2c_dma_read(chip->client, RT5125_CMD_CRC8_INFO, 1,
				 data_buff);
	if (ret < 0)
		goto out_write;
	if (data_buff[0] != (~data_buff[len + 1] & 0xff))
		goto retry_write;
out_write:
	mutex_unlock(&data_lock);
	return ret;
}

/* ---- parent-delegating GPIO shims (pattern: rk826:467-560).
 * FW sequences below call these; virtual_asic serves them from WP-A DTS. ---- */
static int rt5125_set_clock_active(struct rt5125_chip *chip)
{
	struct oplus_chg_ic_dev *parent = chip->ic_dev->parent;
	int rc;

	if (parent == NULL) {
		chg_err("parent not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(parent, OPLUS_IC_FUNC_VOOC_SET_CLOCK_ACTIVE);
	if (rc < 0) {
		chg_err("set clock active error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int rt5125_set_clock_sleep(struct rt5125_chip *chip)
{
	struct oplus_chg_ic_dev *parent = chip->ic_dev->parent;
	int rc;

	if (parent == NULL) {
		chg_err("parent not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(parent, OPLUS_IC_FUNC_VOOC_SET_CLOCK_SLEEP);
	if (rc < 0) {
		chg_err("set clock sleep error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int rt5125_set_reset_active(struct rt5125_chip *chip)
{
	struct oplus_chg_ic_dev *parent = chip->ic_dev->parent;
	int rc;

	if (parent == NULL) {
		chg_err("parent not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(parent, OPLUS_IC_FUNC_VOOC_RESET_ACTIVE);
	if (rc < 0) {
		chg_err("set reset active error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int rt5125_set_reset_active_force(struct rt5125_chip *chip)
{
	struct oplus_chg_ic_dev *parent = chip->ic_dev->parent;
	int rc;

	if (parent == NULL) {
		chg_err("parent not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(parent, OPLUS_IC_FUNC_VOOC_RESET_ACTIVE_FORCE);
	if (rc < 0) {
		chg_err("set reset active force error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int rt5125_set_reset_sleep(struct rt5125_chip *chip)
{
	struct oplus_chg_ic_dev *parent = chip->ic_dev->parent;
	int rc;

	if (parent == NULL) {
		chg_err("parent not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(parent, OPLUS_IC_FUNC_VOOC_RESET_SLEEP);
	if (rc < 0) {
		chg_err("set reset sleep error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

/* ---- FW update: page program + CRC16 + keyword (verbatim v1:555-799).
 * Only adaptations: chip type, shim calls, argless power queries. ---- */
static int rt5125_fw_update(struct rt5125_chip *chip)
{
	const u8 *data = chip->firmware_data;
	s32 len = chip->fw_data_count;
	u8 rwdata = 0, status, fwdata[DEFAULT_MAX_PAGELEN];
	u8 last_page[DEFAULT_MAX_PAGELEN];
	s32 elapsed, wr_len;
	u32 fw_info;
	int i, idx, retry, ret;
	u32 keyword = 0;

	/* poly = x^8 + x^2 + x^1 + 1 */
	crc8_populate_msb(crc8_table, 0x7);
	/* wirte every single page */
	for (i = 0; i < len; i += DEFAULT_MAX_PAGELEN) {
		idx = i / DEFAULT_MAX_PAGELEN;
		elapsed = len - i;
		wr_len = (elapsed > DEFAULT_MAX_PAGELEN)
		    ? DEFAULT_MAX_PAGELEN : elapsed;
		memset(fwdata, 0xff, DEFAULT_MAX_PAGELEN);
		memcpy(fwdata, data + i, wr_len);
		/* page idx */
		rwdata = (u8) idx;
		ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX, &rwdata,
					     sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] wrpage idx fail\n", idx);
			goto update_fw_err;
		}
		/* page data */
		ret = rt5125_i2c_block_write(chip, RT5125_DATA_BUF, fwdata,
					     DEFAULT_MAX_PAGELEN);
		if (ret < 0) {
			chg_err("[%d] wrpage data fail\n", idx);
			goto update_fw_err;
		}
		/* access to write page from buffer to mtp */
		rwdata = (u8) RT5125_WT_PAGE;
		ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
					     sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] wrpage access fail\n", idx);
			goto update_fw_err;
		}
		/* wait 128ms for mtp write */
		msleep(128);
		retry = 0;
busy_check:
		if (retry++ > DEFAULT_PAGEWR_RETRY) {
			chg_err("[%d] wrpage over retrycnt\n", idx);
			goto update_fw_err;
		}
		msleep(5);
		/* check page wr status */
		ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
					    sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] read status fail\n", idx);
			goto update_fw_err;
		}
		status = rwdata & RT5125_OPSTATUS_MASK;
		if (status & RT5125_OPSTATUS_FAILMSK) {
			chg_err("[%d] wrpage fail 0x%02x\n", idx, status);
			goto update_fw_err;
		} else if (status == RT5125_OPSTATUS_ONGOING) {
			chg_err("[%d] wrpage ongoing\n", idx);
			goto busy_check;
		} else if (status == RT5125_OPSTATUS_SUCCESS) {
			chg_err("[%d] wrpage success\n", idx);
		} else {
			chg_err("[%d] wrpage unknown 0x%02x\n", idx, status);
			goto update_fw_err;
		}
	}
	/* FWINFO[31:16] = CRC16_H:CRC16_L */
	fw_info = crc16(RT5125_CRC16_INIT, data, len) << 16;
	/* FWINFO[15:0] = FWSIZE_H:FWSIZE_L */
	fw_info |= (u16) len;
	ret = rt5125_i2c_block_write(chip, RT5125_FW_CRC16_INFO,
				     (void *)&fw_info, sizeof(fw_info));
	if (ret < 0) {
		chg_err("write fw info fail\n");
		goto update_fw_err;
	}
	/* write access for crc16 write */
	rwdata = (u8) RT5125_WT_FW_CRC16;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
				     sizeof(rwdata));
	if (ret < 0) {
		chg_err("wr_fw_crc access fail\n");
		goto update_fw_err;
	}
	/* wait 20ms for CRC write */
	msleep(20);
	/* check wr_fw_crc status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
				    sizeof(rwdata));
	if (ret < 0) {
		chg_err("wr_fw_crc read status fail\n");
		goto update_fw_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("wr_fw_crc status fail\n");
		goto update_fw_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("wr_fw_crc success\n");
	} else {
		chg_err("wr_fw_crc unknown 0x%02x\n", status);
		goto update_fw_err;
	}
	retry = 0;
	/* write access for crc16 verify */
	rwdata = (u8) RT5125_FW_CRC16_VRFY;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
				     sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy access fail\n");
		goto update_fw_err;
	}
	/* wait 200ms for CRC verify */
crc_busy_retry:
	msleep(200);
	/* check fw_crc_vrfy status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
				    sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy read status fail\n");
		goto update_fw_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_ONGOING) {
		dev_err(chip->dev, "fw_crc_vrfy ongoing\n");
		retry++;
		if (retry < 10)
			goto crc_busy_retry;
		else {
			dev_err(chip->dev, "fw_crc_vrfy busy retry fail\n");
			goto update_fw_err;
		}
	} else if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("fw_crc_vrfy status fail\n");
		goto update_fw_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("fw_crc_vrfy success\n");
	} else {
		chg_err("fw_crc_vrfy unknown 0x%02x\n", status);
		goto update_fw_err;
	}
	if (!(rwdata & RT5125_FW_CRC16_RSLT)) {
		chg_err("fw_crc_vrfy crc result fail\n");
		goto update_fw_err;
	}
	chg_err("fw_crc_vrfy OK\n");
	/* write access for keyword */
	retry = 0;
keyword_retry:
	rwdata = (u8) RT5125_WT_KEY;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
				     sizeof(rwdata));
	if (ret < 0) {
		chg_err("key_write access fail\n");
		goto update_fw_err;
	}
	/* wait keyword write success */
	msleep(10);
	/* check keyword write status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
				    sizeof(rwdata));
	if (ret < 0) {
		chg_err("key_write read status fail\n");
		goto update_fw_err;
	}
	retry++;
	if (retry > 10) {
		dev_err(chip->dev, "Key_write fail by retry over %d\n", retry);
		return -EFAULT;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("key_write status fail retry 0x%02x\n", status);
		goto keyword_retry;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("key_write success\n");
	} else {
		chg_err("key_write unknown status: 0x%02x, retry\n", status);
		goto keyword_retry;
	}

	/* check keyword correctly */
	idx = DEFAULT_MAX_PAGEIDX - 1;
	rwdata = (u8)idx;
	ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX,
				     &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] rdpage idx fail\n", idx);
		goto update_fw_err;
	}

	/* access to read page from mtp to buffer */
	rwdata = (u8)RT5125_RD_PAGE;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL,
				     &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d]rdpage access fail\n", idx);
		goto update_fw_err;
	}

	/* wait 5ms for mtp read */
	msleep(5);

	/* check page rd status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS,
				    &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] read status fail\n", idx);
		goto update_fw_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("[%d] rdpage fail 0x%02x\n", idx, status);
		goto keyword_retry;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("[%d] rdpage success\n", idx);
	} else {
		chg_err("[%d]rdpage unknown 0x%02x\n", idx, status);
		goto keyword_retry;
	}

	/* page data */
	ret = rt5125_i2c_block_read(chip, RT5125_DATA_BUF,
				    last_page, DEFAULT_MAX_PAGELEN);
	if (ret < 0) {
		chg_err("[%d] rdpage data fail\n", idx);
		goto update_fw_err;
	}

	/* get keyword */
	keyword = (last_page[127] << 8) + last_page[126];
	if (keyword != 0x5125) {
		dev_err(chip->dev, "key_write [%d] check fail\n", keyword);
		goto keyword_retry;
	} else {
		chg_err("key_write success\n");
	}

	chip->fw_mcu_version = chip->fw_data_version;
	chg_debug("success\n");
	return 0;
update_fw_err:
	charger_abnormal_log = CRITICAL_LOG_VOOC_FW_UPDATE_ERR;
	chg_err("fail\n");
	return 1;
}

/* ---- FW check (verbatim v1:801-972) ---- */
static bool rt5125_fw_update_check(struct rt5125_chip *chip)
{
	const u8 *data = chip->firmware_data;
	s32 len = chip->fw_data_count;
	u8 rwdata = 0, status, fwdata[DEFAULT_MAX_PAGELEN * 2];
	u8 last_page[DEFAULT_MAX_PAGELEN];
	u8 verinfo[DEFAULT_VERINFO_LEN];
	u32 fw_info = 0;
	u32 keyword = 0;
	int i, idx, ret, retry = 0;

	if (!chip->firmware_data) {
		chg_err("rt5125_fw_data Null, Return\n");
		return FW_CHECK_FAIL;
	}
	/* poly = x^8 + x^2 + x^1 + 1 */
	crc8_populate_msb(crc8_table, 0x7);
	/* write access for crc16 verify */
	rwdata = (u8) RT5125_FW_CRC16_VRFY;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
				     sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy access fail\n");
		goto fw_update_check_err;
	}
	/* wait 200ms for CRC verify */
crc_busy_retry:
	msleep(200);
	/* check fw_crc_vrfy status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
				    sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy read status fail\n");
		goto fw_update_check_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_ONGOING) {
		dev_err(chip->dev, "fw_crc_vrfy ongoing\n");
		retry++;
		if (retry < 10)
			goto crc_busy_retry;
		else {
			dev_err(chip->dev, "fw_crc_vrfy busy retry fail\n");
			goto fw_update_check_err;
		}
	} else if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("fw_crc_vrfy status fail\n");
		goto fw_update_check_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("fw_crc_vrfy success\n");
	} else {
		chg_err("fw_crc_vrfy unknown 0x%02x\n", status);
		goto fw_update_check_err;
	}
	if (!(rwdata & RT5125_FW_CRC16_RSLT)) {
		chg_err("fw_crc_vrfy crc result fail\n");
		goto fw_update_check_err;
	}
	chg_err("fw_crc_vrfy OK\n");
	/* read orig fw info */
	idx = DEFAULT_MAX_PAGEIDX - 1;
	rwdata = (u8) idx;
	ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX, &rwdata,
				     sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] rdpage idx fail\n", idx);
		goto fw_update_check_err;
	}
	/* access to read page from mtp to buffer */
	rwdata = (u8) RT5125_RD_PAGE;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
				     sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d]rdpage access fail\n", idx);
		goto fw_update_check_err;
	}
	/* wait 5ms for mtp read */
	msleep(5);
	/* check page rd status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
				    sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] read status fail\n", idx);
		goto fw_update_check_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("[%d] rdpage fail 0x%02x\n", idx, status);
		goto fw_update_check_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("[%d] rdpage success\n", idx);
	} else {
		chg_err("[%d]rdpage unknown 0x%02x\n", idx, status);
		goto fw_update_check_err;
	}
	/* page data */
	ret = rt5125_i2c_block_read(chip, RT5125_DATA_BUF, last_page,
				    DEFAULT_MAX_PAGELEN);
	if (ret < 0) {
		chg_err("[%d] rdpage data fail\n", idx);
		goto fw_update_check_err;
	}
	/* get fw size */
	fw_info = (last_page[123] << 8) + last_page[122];
	if (fw_info < DEFAULT_VERINFO_LEN) {
		chg_err("size [%d] smaller than verinfo\n", fw_info);
		goto fw_update_check_err;
	}

	/* get keyword */
	keyword = (last_page[127] << 8) + last_page[126];
	if (keyword != 0x5125) {
		chg_err("keyword [%d] check fail\n", keyword);
		goto fw_update_check_err;
	}

	/* always read last two single page */
	idx = fw_info / DEFAULT_MAX_PAGELEN - 1;
	for (i = 0; i < 2; i++) {
		/* page idx */
		if ((idx + i) < 0 || (idx + i) >= DEFAULT_MAX_PAGEIDX)
			continue;
		rwdata = (u8) (idx + i);
		ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX, &rwdata,
					     sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] rdpage idx fail\n", idx + i);
			goto fw_update_check_err;
		}
		/* access to read page from mtp to buffer */
		rwdata = (u8) RT5125_RD_PAGE;
		ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata,
					     sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d]rdpage access fail\n", idx + i);
			goto fw_update_check_err;
		}
		/* wait 5ms for mtp read */
		msleep(5);
		/* check page rd status */
		ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata,
					    sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] read status fail\n", idx + i);
			goto fw_update_check_err;
		}
		status = rwdata & RT5125_OPSTATUS_MASK;
		if (status & RT5125_OPSTATUS_FAILMSK) {
			chg_err("[%d] rdpage fail 0x%02x\n", idx + i, status);
			goto fw_update_check_err;
		} else if (status == RT5125_OPSTATUS_SUCCESS) {
			chg_err("[%d] rdpage success\n", idx + i);
		} else {
			chg_err("[%d]rdpage unknown 0x%02x\n", idx + i, status);
			goto fw_update_check_err;
		}
		/* page data */
		ret = rt5125_i2c_block_read(chip, RT5125_DATA_BUF,
					    fwdata + i * DEFAULT_MAX_PAGELEN,
					    DEFAULT_MAX_PAGELEN);
		if (ret < 0) {
			chg_err("[%d] rdpage data fail\n", idx + i);
			goto fw_update_check_err;
		}
	}
	idx = DEFAULT_MAX_PAGELEN + fw_info % DEFAULT_MAX_PAGELEN -
		DEFAULT_VERINFO_LEN;
	memcpy(verinfo, fwdata + idx, DEFAULT_VERINFO_LEN);
	chip->fw_mcu_version = verinfo[DEFAULT_VERINFO_LEN - 4];
	chg_err("update before: fw_mcu_version=%x,%x\n", chip->fw_mcu_version,
		verinfo[7]);
	idx = len - DEFAULT_VERINFO_LEN;
	ret = memcmp(verinfo, data + idx, DEFAULT_VERINFO_LEN);
	if (ret != 0) {
		chg_err("verinfo not equal\n");
		goto fw_update_check_err;
	}
	return FW_CHECK_SUCCESS;
fw_update_check_err:
	chg_err("rt5125_fw_data check fail\n");
	return FW_CHECK_FAIL;
}

/******************************************************************/
/* ---- FW version handshake (verbatim v1:975-1016, shim-adapted) ---- */
static int rt5125_get_fw_verion_from_ic(struct rt5125_chip *chip)
{
	unsigned char addr_buf[2] = { 0x3B, 0xF0 };
	unsigned char data_buf[4] = { 0 };
	int rc = 0;
	int update_result = 0;

	chg_debug("in\n");

	if (oplus_is_power_off_charging() == true ||
	    oplus_is_charger_reboot() == true) {
		chip->upgrading = true;
		update_result = rt5125_fw_update(chip);
		chip->upgrading = false;
		if (update_result) {
			msleep(30);
			rt5125_set_clock_sleep(chip);
			rt5125_set_reset_active_force(chip);
		}
	} else {
		rt5125_set_clock_active(chip);
		chip->boot_by_gpio = true;
		msleep(10);
		rt5125_set_reset_active_force(chip);
		chip->upgrading = true;
		msleep(2500);
		chip->boot_by_gpio = false;
		rt5125_set_clock_sleep(chip);
/*		first:set address*/
		rc = oplus_vooc_i2c_write(chip->client, 0x01, 2, &addr_buf[0]);
		if (rc < 0) {
			chg_err(" i2c_write 0x01 error\n");
			return FW_CHECK_FAIL;
		}
		msleep(2);
		oplus_vooc_i2c_read(chip->client, 0x03, 4, data_buf);

		chg_err("data:%x %x %x %x, fw_ver:%x\n", data_buf[0],
			data_buf[1], data_buf[2], data_buf[3], data_buf[0]);
		chip->upgrading = false;
		msleep(5);
		rt5125_set_reset_active_force(chip);
	}
	return data_buf[0];
}

/* ---- check+recover with 5-try update loop (verbatim v1:1018-1076) ---- */
static int __rt5125_fw_check_then_recover(struct rt5125_chip *chip)
{
	int update_result = 0;
	int try_count = 5;
	int ret = 0;

	if (!chip->firmware_data) {
		chg_err("rt5125_fw_data Null, Return\n");
		return FW_ERROR_DATA_MODE;
	} else {
		chg_debug("begin\n");
	}
	if (oplus_is_power_off_charging() == true ||
	    oplus_is_charger_reboot() == true) {
		chip->upgrading = false;
		rt5125_set_clock_sleep(chip);
		rt5125_set_reset_sleep(chip);
		chip->reset_status = 0;
		ret = FW_NO_CHECK_MODE;
	} else {
		chip->boot_by_gpio = false;
		rt5125_set_clock_active(chip);
		chip->boot_by_gpio = true;
		msleep(10);
		rt5125_set_reset_active_force(chip);
		chip->upgrading = true;
		msleep(2500);
		chip->boot_by_gpio = false;
		rt5125_set_clock_sleep(chip);
		__pm_stay_awake(chip->update_wake_lock);
		if (rt5125_fw_update_check(chip) == FW_CHECK_FAIL) {
			chg_debug("firmware update start\n");
			do {
				rt5125_set_clock_active(chip);
				chip->boot_by_gpio = true;
				msleep(10);
				chip->upgrading = false;
				rt5125_set_reset_active(chip);
				chip->upgrading = true;
				msleep(2500);
				chip->boot_by_gpio = false;
				rt5125_set_clock_sleep(chip);
				update_result = rt5125_fw_update(chip);
				if (!update_result)
					break;
			} while ((update_result) && (--try_count > 0));
			chg_debug("firmware update end, retry times %d\n",
				  5 - try_count);
		} else {
			chip->vooc_fw_check = true;
			chg_debug("fw check ok\n");
		}
		__pm_relax(chip->update_wake_lock);
		chip->upgrading = false;
		msleep(5);
		rt5125_set_reset_active(chip);
		ret = FW_CHECK_MODE;
	}
	rt5125_set_reset_sleep(chip);
	chip->reset_status = 0;
	return ret;
}

/* ---- v2 registration (pattern: rk826:1576-1728) ---- */
static void register_vooc_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;

	version = "rt5125";
	manufacture = "ROCKCHIP";

	ret = register_device_proc("vooc", version, manufacture);
	if (ret) {
		chg_err(" fail\n");
	}
}

static void rt5125_shutdown(struct i2c_client *client)
{
}

static ssize_t vooc_fw_check_read(struct file *filp, char __user *buff,
				  size_t count, loff_t *off)
{
	struct rt5125_chip *chip = PDE_DATA(file_inode(filp));
	char page[256] = { 0 };
	char read_data[32] = { 0 };
	int len = 0;

	if (chip->vooc_fw_check) {
		read_data[0] = '1';
	} else {
		read_data[0] = '0';
	}
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations vooc_fw_check_proc_fops = {
	.read = vooc_fw_check_read,
	.llseek = noop_llseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops vooc_fw_check_proc_fops = {
	.proc_read = vooc_fw_check_read,
	.proc_lseek = noop_llseek,
};
#endif

static int init_proc_vooc_fw_check(struct rt5125_chip *chip)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create_data("vooc_fw_check", 0444, NULL,
			     &vooc_fw_check_proc_fops, chip);
	if (!p) {
		chg_err("proc_create vooc_fw_check_proc_fops fail!\n");
	}
	return 0;
}

/* Liveness probe: RT5125 has no ID register (v1 used GPIO strapping, which
 * in v2 belongs to virtual_asic). After the standard clock/reset bracketing,
 * a raw STATUS read that completes without a FAIL bit means the ASIC
 * answers on this bus. Returns false -> INIT returns -ENODEV ("not used"). */
static bool rt5125_is_used(struct i2c_client *client)
{
	struct rt5125_chip *chip = i2c_get_clientdata(client);
	u8 status = 0;
	int rc = 0;

	rt5125_set_clock_active(chip);
	msleep(10);
	rt5125_set_reset_active_force(chip);
	msleep(2500);
	rt5125_set_clock_sleep(chip);

	rc = oplus_i2c_dma_read(chip->client, RT5125_STATUS, 1, &status);
	if (rc < 0) {
		chg_err("rt5125 read status fail, rc = %d\n", rc);
		goto not_used;
	}
	chg_err("rt5125 status: 0x%x\n", status);
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("rt5125 status fail bits set\n");
		goto not_used;
	}
	msleep(5);
	rt5125_set_reset_active(chip);
	rt5125_set_reset_sleep(chip);
	return true;

not_used:
	msleep(5);
	rt5125_set_reset_active(chip);
	rt5125_set_reset_sleep(chip);
	return false;
}

static int rt5125_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct rt5125_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!rt5125_is_used(chip->client)) {
		chg_info("rt5125 not used\n");
		return -ENODEV;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	chip->update_wake_lock =
		wakeup_source_register("rt5125_update_wake_lock");
#else
	chip->update_wake_lock =
		wakeup_source_register(NULL, "rt5125_update_wake_lock");
#endif

	register_vooc_devinfo();
	init_proc_vooc_fw_check(chip);
	ic_dev->online = true;

	return OPLUS_VOOC_IC_RT5125;
}

static int rt5125_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct rt5125_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ic_dev->online = false;
	wakeup_source_remove(chip->update_wake_lock);

	return 0;
}

static int rt5125_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[],
			   int len)
{
	return 0;
}

static int rt5125_fw_upgrade(struct oplus_chg_ic_dev *ic_dev)
{
	struct rt5125_chip *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip->firmware_data == NULL) {
		chg_err("firmware_data is NULL");
		return -ENODATA;
	}

	rc = rt5125_fw_update(chip);
	if (rc != 0) {
		chg_err("firmware upgrade error, rc=%d\n", rc);
		return -EINVAL;
	}

	return 0;
}

static int rt5125_fw_check_then_recover(struct oplus_chg_ic_dev *ic_dev)
{
	struct rt5125_chip *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = __rt5125_fw_check_then_recover(chip);
	if (rc != FW_CHECK_MODE) {
		chg_err("fw_check_then_recover error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int rt5125_get_fw_version(struct oplus_chg_ic_dev *ic_dev,
				 u32 *version)
{
	struct rt5125_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*version = rt5125_get_fw_verion_from_ic(chip);

	return 0;
}

static int rt5125_get_upgrade_status(struct oplus_chg_ic_dev *ic_dev,
				     bool *upgrading)
{
	struct rt5125_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*upgrading = chip->upgrading;

	return 0;
}

static int rt5125_boot_by_gpio(struct oplus_chg_ic_dev *ic_dev,
			       bool *boot_by_gpio)
{
	struct rt5125_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*boot_by_gpio = chip->boot_by_gpio;

	return 0;
}

static void *rt5125_get_func(struct oplus_chg_ic_dev *ic_dev,
			     enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       rt5125_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       rt5125_exit);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       rt5125_smt_test);
		break;
	case OPLUS_IC_FUNC_VOOC_FW_UPGRADE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOC_FW_UPGRADE,
					       rt5125_fw_upgrade);
		break;
	case OPLUS_IC_FUNC_VOOC_FW_CHECK_THEN_RECOVER:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_VOOC_FW_CHECK_THEN_RECOVER,
			rt5125_fw_check_then_recover);
		break;
	case OPLUS_IC_FUNC_VOOC_GET_FW_VERSION:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_VOOC_GET_FW_VERSION,
			rt5125_get_fw_version);
		break;
	case OPLUS_IC_FUNC_VOOC_UPGRADING:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOC_UPGRADING,
					       rt5125_get_upgrade_status);
		break;
	case OPLUS_IC_FUNC_VOOC_BOOT_BY_GPIO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOC_BOOT_BY_GPIO,
					       rt5125_boot_by_gpio);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq rt5125_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
};

static int rt5125_parse_fw_data_by_dts(struct rt5125_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	const char *data;
	int len = 0;

	data = of_get_property(node, "oplus,firmware_data", &len);
	if (!data) {
		chg_err("%s: parse vooc fw failed\n", __func__);
		return -ENOMEM;
	}

	chip->firmware_data = data;
	chip->fw_data_count = len;
	chip->fw_data_version = data[len - 4];
	chg_info("version: 0x%x, count: %d\n", chip->fw_data_version,
		 chip->fw_data_count);

	return 0;
}

static int rt5125_driver_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct rt5125_chip *chip;
	struct device_node *node = client->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc;

	chip = devm_kzalloc(&client->dev, sizeof(struct rt5125_chip),
			    GFP_KERNEL);
	if (!chip) {
		chg_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	/* FW always comes from the WP-A DTS blob on this project; the flag
	 * is recorded for the core (no request_firmware path needed). */
	chip->vooc_fw_update_newmethod =
		of_property_read_bool(node, "oplus,vooc_fw_update_newmethod");
	rc = rt5125_parse_fw_data_by_dts(chip);
	if (rc < 0)
		goto error;

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto error;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto error;
	}
	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	sprintf(ic_cfg.manu_name, "rt5125");
	sprintf(ic_cfg.fw_id, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = rt5125_get_func;
	ic_cfg.virq_data = rt5125_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(rt5125_virq_table);
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto error;
	}
	chg_info("register %s\n", node->name);

	chg_info("rt5125 probe success\n");
	return 0;

error:
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);
	return rc;
}

static int rt5125_driver_remove(struct i2c_client *client)
{
	struct rt5125_chip *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	if (chip->ic_dev->online)
		rt5125_exit(chip->ic_dev);
	devm_oplus_chg_ic_unregister(&client->dev, chip->ic_dev);
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
static const struct of_device_id rt5125_match[] = {
	{ .compatible = "oplus,rt5125-fastcg" },
	{},
};

static const struct i2c_device_id rt5125_id[] = {
	{ "rt5125-fastcg", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, rt5125_id);

struct i2c_driver rt5125_i2c_driver = {
	.driver = {
		.name = "rt5125-fastcg",
		.owner        = THIS_MODULE,
		.of_match_table = rt5125_match,
	},
	.probe = rt5125_driver_probe,
	.remove	= rt5125_driver_remove,
	.shutdown = rt5125_shutdown,
	.id_table = rt5125_id,
};

int rt5125_driver_init(void)
{
	int ret = 0;

	chg_debug("init start\n");

	if (i2c_add_driver(&rt5125_i2c_driver) != 0) {
		chg_err(" failed to register rt5125 i2c driver.\n");
	} else {
		chg_debug(" Success to register rt5125 i2c driver.\n");
	}

	return ret;
}

void rt5125_driver_exit(void)
{
	i2c_del_driver(&rt5125_i2c_driver);
}
oplus_chg_module_register(rt5125_driver);
