// SPDX-License-Identifier: GPL-2.0-only

#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/unaligned.h>

#include "eeprom_driver.h"
#include "eeprom_i2c_common_driver.h"
#include "kd_imgsensor.h"
#include "x6871_cam_cal.h"

#define X6871_OTP_HEADER_OFFSET	0x001
#define X6871_OTP_HEADER_ID	0x010b00ff
#define X6871_AWB_SIZE		6
#define X6871_PDAF_FIRST_SIZE	0x1f0
#define X6871_PDAF_SECOND_SIZE	0x3ec

struct x6871_otp_layout {
	const char *node;
	unsigned int sensor_id;
	unsigned int i2c_addr;
	unsigned int awb;
	unsigned int awb_checksum;
	unsigned int af;
	unsigned int lsc;
	unsigned int pdaf;
};

/* Offsets from the X6871 stock camera_eeprom_v4l2 layout and read paths. */
static const struct x6871_otp_layout x6871_layouts[] = {
	{
		.node = "camera_eeprom0",
		.sensor_id = S5KHM6SX_SENSOR_ID,
		.i2c_addr = 0x50,
		.awb = 0x0c81,
		.awb_checksum = 0x0c8f,
		.af = 0x0c91,
		.lsc = 0x0ca1,
		.pdaf = 0x13ef,
	}, {
		.node = "camera_eeprom1",
		.sensor_id = GC32E1_SENSOR_ID,
		.i2c_addr = 0x51,
		.awb = 0x0a8c,
		.lsc = 0x0a9c,
	},
};

static const struct x6871_otp_layout *
x6871_find_layout(const struct EEPROM_DRV_FD_DATA *pdata, unsigned int sensor_id)
{
	const struct device_node *node = pdata->pdrv->pi2c_client->dev.of_node;
	u32 addr;
	unsigned int i;

	if (of_property_read_u32(node, "reg", &addr))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(x6871_layouts); i++) {
		const struct x6871_otp_layout *layout = &x6871_layouts[i];

		if (sensor_id == layout->sensor_id && addr == layout->i2c_addr &&
		    of_node_name_eq(node, layout->node))
			return layout;
	}

	return NULL;
}

bool x6871_cam_cal_matches(const struct EEPROM_DRV_FD_DATA *pdata, unsigned int sensor_id)
{
	return x6871_find_layout(pdata, sensor_id) != NULL;
}

static bool x6871_read(struct i2c_client *client, unsigned int offset, u8 *buf,
		       unsigned int len)
{
	return Common_read_region(client, offset, buf, len) == len;
}

static u8 x6871_sum(const u8 *buf, unsigned int len)
{
	u8 sum = 0;

	while (len--)
		sum += *buf++;
	return sum;
}

static bool x6871_read_awb(struct i2c_client *client, const struct x6871_otp_layout *layout,
			  struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT *out)
{
	u8 awb[2 * X6871_AWB_SIZE], checksum;
	u16 unit_r, unit_g, unit_b, gold_r, gold_g, gold_b;
	unsigned int unit_max, gold_max;
	struct STRUCT_CAM_CAL_PREGAIN_STRUCT *gain = &out->S2aAwb;

	if (!x6871_read(client, layout->awb, awb, sizeof(awb)))
		return false;
	if (layout->awb_checksum &&
	    (!x6871_read(client, layout->awb_checksum, &checksum, 1) ||
	     checksum != x6871_sum(awb, sizeof(awb))))
		return false;

	/* EEPROM stores little-endian R, B, G, first unit then golden. */
	unit_r = get_unaligned_le16(awb);
	unit_b = get_unaligned_le16(awb + 2);
	unit_g = get_unaligned_le16(awb + 4);
	gold_r = get_unaligned_le16(awb + 6);
	gold_b = get_unaligned_le16(awb + 8);
	gold_g = get_unaligned_le16(awb + 10);
	if (!unit_r || !unit_g || !unit_b || !gold_r || !gold_g || !gold_b ||
	    (unit_r == U16_MAX && unit_g == U16_MAX && unit_b == U16_MAX) ||
	    (gold_r == U16_MAX && gold_g == U16_MAX && gold_b == U16_MAX))
		return false;

	unit_max = max3(unit_r, unit_g, unit_b) * 512U;
	gold_max = max3(gold_r, gold_g, gold_b) * 512U;
	gain->rUnitGainu4R = DIV_ROUND_CLOSEST(unit_max, (unsigned int)unit_r);
	gain->rUnitGainu4G = DIV_ROUND_CLOSEST(unit_max, (unsigned int)unit_g);
	gain->rUnitGainu4B = DIV_ROUND_CLOSEST(unit_max, (unsigned int)unit_b);
	gain->rGoldGainu4R = DIV_ROUND_CLOSEST(gold_max, (unsigned int)gold_r);
	gain->rGoldGainu4G = DIV_ROUND_CLOSEST(gold_max, (unsigned int)gold_g);
	gain->rGoldGainu4B = DIV_ROUND_CLOSEST(gold_max, (unsigned int)gold_b);
	gain->rGainSetNum = 1;
	gain->rValueR = unit_r;
	gain->rValueGr = unit_g;
	gain->rValueGb = unit_g;
	gain->rValueB = unit_b;
	gain->rGoldenR = gold_r;
	gain->rGoldenGr = gold_g;
	gain->rGoldenGb = gold_g;
	gain->rGoldenB = gold_b;
	out->S2aBitEn |= CAM_CAL_AWB_BITEN;
	return true;
}

static bool x6871_read_2a(struct i2c_client *client, const struct x6871_otp_layout *layout,
			 struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT *out)
{
	u8 af[12];

	memset(out, 0, sizeof(*out));
	if (!x6871_read_awb(client, layout, out))
		return false;
	if (layout->af) {
		if (!x6871_read(client, layout->af, af, sizeof(af)))
			return false;
		out->S2aAF_t.AF_infinite_pattern_distance = get_unaligned_le16(af);
		out->S2aAF_t.AF_Macro_pattern_distance = get_unaligned_le16(af + 4);
		out->S2aAf[0] = get_unaligned_le16(af + 6);
		out->S2aAf[1] = get_unaligned_le16(af + 10);
		out->S2aBitEn |= CAM_CAL_AF_BITEN;
		out->S2aAfBitflagEn = 0x0c;
	}
	out->S2aVer = 1;
	return true;
}

static bool x6871_read_lsc(struct i2c_client *client, const struct x6871_otp_layout *layout,
			  struct STRUCT_CAM_CAL_SINGLE_LSC_STRUCT *out)
{
	struct STRUCT_CAM_CAL_LSC_MTK_TYPE *lsc = &out->LscTable.MtkLcsData;
	u8 *table = out->LscTable.Data + offsetof(struct STRUCT_CAM_CAL_LSC_MTK_TYPE, SlimLscType);
	u8 marker[2], checksum;

	memset(out, 0, sizeof(*out));
	if (!x6871_read(client, layout->lsc - 1, marker, sizeof(marker)) ||
	    !get_unaligned_le16(marker) ||
	    !x6871_read(client, layout->lsc, table, CAM_CAL_SINGLE_LSC_SIZE) ||
	    !x6871_read(client, layout->lsc + CAM_CAL_SINGLE_LSC_SIZE, &checksum, 1) ||
	    checksum != x6871_sum(table, CAM_CAL_SINGLE_LSC_SIZE))
		return false;
	lsc->MtkLscType = 2;
	lsc->PixId = 8;
	lsc->TableSize = CAM_CAL_SINGLE_LSC_SIZE;
	return true;
}

static bool x6871_read_pdaf(struct i2c_client *client, const struct x6871_otp_layout *layout,
			   struct STRUCT_CAM_CAL_PDAF_STRUCT *out)
{
	const unsigned int size = X6871_PDAF_FIRST_SIZE + X6871_PDAF_SECOND_SIZE;
	u8 valid, checksum;

	memset(out, 0, sizeof(*out));
	if (!x6871_read(client, layout->pdaf - 1, &valid, 1) || valid != 1 ||
	    !x6871_read(client, layout->pdaf, out->Data, size) ||
	    !x6871_read(client, layout->pdaf + size, &checksum, 1) ||
	    checksum != x6871_sum(out->Data, size))
		return false;
	/* The stock HAL expects a 0x5de-byte result (two trailing zero bytes). */
	out->Size_of_PDAF = size + 2;
	return true;
}

static unsigned int x6871_command_error(const struct x6871_otp_layout *layout,
				      unsigned int command)
{
	switch (command) {
	case CAMERA_CAM_CAL_DATA_3A_GAIN:
	case CAMERA_CAM_CAL_DATA_SHADING_TABLE:
		return CAM_CAL_ERR_NO_ERR;
	case CAMERA_CAM_CAL_DATA_PDAF:
		return layout->pdaf ? CAM_CAL_ERR_NO_ERR : CAM_CAL_ERR_NO_PDAF;
	default:
		return command < CAMERA_CAM_CAL_DATA_LIST ? CamCalReturnErr[command] :
			CAM_CAL_ERR_NO_CMD;
	}
}

static bool x6871_read_header(struct i2c_client *client, const struct x6871_otp_layout *layout)
{
	u8 id[4];

	return client->addr == layout->i2c_addr &&
	       x6871_read(client, X6871_OTP_HEADER_OFFSET, id, sizeof(id)) &&
	       get_unaligned_le32(id) == X6871_OTP_HEADER_ID;
}

unsigned int x6871_cam_cal_read(struct EEPROM_DRV_FD_DATA *pdata,
			       struct STRUCT_CAM_CAL_DATA_STRUCT *data)
{
	const struct x6871_otp_layout *layout = x6871_find_layout(pdata, data->sensorID);
	struct i2c_client *client = pdata->pdrv->pi2c_client;
	unsigned int ret;
	bool ok = false;

	/* Fields populated here have the same offsets in the stock ioctl. */
	static_assert(offsetof(struct STRUCT_CAM_CAL_DATA_STRUCT, SingleLsc) == 0x28);
	static_assert(offsetof(struct STRUCT_CAM_CAL_DATA_STRUCT, Single2A) == 0x874);
	static_assert(offsetof(struct STRUCT_CAM_CAL_DATA_STRUCT, PDAF) == 0x924);
	static_assert(sizeof(*data) <= X6871_CAM_CAL_READ_SIZE);

	if (!layout)
		return CAM_CAL_ERR_NO_DEVICE;
	ret = x6871_command_error(layout, data->Command);
	if (ret)
		return ret;

	mutex_lock(&pdata->pdrv->eeprom_mutex);
	if (!x6871_read_header(client, layout)) {
		mutex_unlock(&pdata->pdrv->eeprom_mutex);
		return CAM_CAL_ERR_NO_DEVICE;
	}

	switch (data->Command) {
	case CAMERA_CAM_CAL_DATA_3A_GAIN:
		ok = x6871_read_2a(client, layout, &data->Single2A);
		break;
	case CAMERA_CAM_CAL_DATA_SHADING_TABLE:
		ok = x6871_read_lsc(client, layout, &data->SingleLsc);
		break;
	case CAMERA_CAM_CAL_DATA_PDAF:
		ok = x6871_read_pdaf(client, layout, &data->PDAF);
		break;
	default:
		break;
	}
	mutex_unlock(&pdata->pdrv->eeprom_mutex);
	data->DataVer = CAM_CAL_SINGLE_EEPROM_DATA;
	return ok ? CAM_CAL_ERR_NO_ERR : CamCalReturnErr[data->Command];
}

unsigned int x6871_cam_cal_need_power(struct EEPROM_DRV_FD_DATA *pdata,
				    struct STRUCT_CAM_CAL_NEED_POWER_ON *data)
{
	const struct x6871_otp_layout *layout = x6871_find_layout(pdata, data->sensorID);

	if (!layout)
		return CAM_CAL_ERR_NO_DEVICE;
	if (data->Command >= CAMERA_CAM_CAL_DATA_LIST)
		return CAM_CAL_ERR_NO_CMD;

	/* The EEPROM may not be powered until after the HAL asks this. */
	data->needPowerOn = !x6871_command_error(layout, data->Command);
	return CAM_CAL_ERR_NO_ERR;
}
