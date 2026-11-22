/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __X6871_CAM_CAL_H
#define __X6871_CAM_CAL_H

#include "cam_cal_format.h"

struct EEPROM_DRV_FD_DATA;

/* The X6871 HAL uses a larger payload; only the common fields are used. */
#define X6871_CAM_CAL_READ_SIZE	0x2c30
#define X6871_CAM_CAL_READ	_IOC(_IOC_READ | _IOC_WRITE, CAM_CALAGIC, 15, \
				     X6871_CAM_CAL_READ_SIZE)
#define X6871_CAM_CAL_QUERY	_IOC(_IOC_READ, CAM_CALAGIC, 20, X6871_CAM_CAL_READ_SIZE)

bool x6871_cam_cal_matches(const struct EEPROM_DRV_FD_DATA *pdata, unsigned int sensor_id);
unsigned int x6871_cam_cal_read(struct EEPROM_DRV_FD_DATA *pdata,
			       struct STRUCT_CAM_CAL_DATA_STRUCT *data);
unsigned int x6871_cam_cal_need_power(struct EEPROM_DRV_FD_DATA *pdata,
				    struct STRUCT_CAM_CAL_NEED_POWER_ON *data);

#endif
