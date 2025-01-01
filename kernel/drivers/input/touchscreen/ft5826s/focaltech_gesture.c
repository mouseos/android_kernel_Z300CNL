/*
 *
 * FocalTech fts TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

 /*******************************************************************************
*
* File Name: Focaltech_Gestrue.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-07-06
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define GESTURE_DOUBLECLICK	    	0x24
#define GESTURE_W		    	0x31
#define GESTURE_S		    	0x46
#define GESTURE_E		    	0x33
#define GESTURE_C		    	0x34
#define GESTURE_Z		    	0x65
#define GESTURE_V			0x54
#define FTS_GESTURE_POINTS 		255
#define FTS_GESTURE_POINTS_HEADER 	8
#define FTS_GESTURE_OUTPUT_ADRESS 	0xD3

#define FTS_WAKEUP_TIME			25

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/

/*******************************************************************************
* Name: fts_Gesture_init
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_Gesture_init(struct input_dev *input_dev)
{
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);

	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);

	return 0;
}

/*******************************************************************************
* Name: fts_check_gesture
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static void fts_check_gesture(struct input_dev *input_dev, int gesture_id)
{
	switch(gesture_id)
	{
	        case GESTURE_DOUBLECLICK:
			pr_info("[Focal][Touch] gesture_id = 0x%x (DCLICK)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_DOUBLE_CLICK, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_DOUBLE_CLICK, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_W:
			pr_info("[Focal][Touch] gesture_id = 0x%x (W)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_W, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_W, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_S:
			pr_info("[Focal][Touch] gesture_id = 0x%x (S)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_S, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_S, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_E:
			pr_info("[Focal][Touch] gesture_id = 0x%x (E)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_E, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_E, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_C:
			pr_info("[Focal][Touch] gesture_id = 0x%x (C)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_C, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_C, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_Z:
			pr_info("[Focal][Touch] gesture_id = 0x%x (Z)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_Z, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_Z, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_V:
			pr_info("[Focal][Touch] gesture_id = 0x%x (V)\n",
								gesture_id);
	                input_report_key(input_dev, KEY_GESTURE_V, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_V, 0);
	                input_sync(input_dev);
	                break;
	        default:
	                break;
	}
}

 /************************************************************************
* Name: fts_read_Gesturedata
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
int fts_read_Gesturedata(void)
{
	unsigned char buf[FTS_GESTURE_POINTS * 3] = { 0 };
	int ret = -1;
	int retry = 2;
	int gesture_id = 0;

	wake_lock(&fts_wq_data->fts_wake_lock);

	buf[0] = FTS_GESTURE_OUTPUT_ADRESS;

retry_gesture_read:
	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf,
				FTS_GESTURE_POINTS_HEADER);
	if (ret < 0) {
		if (retry > 0) {
			dev_dbg(&fts_wq_data->client->dev,
				"%s: i2c retry\n", __func__);
			msleep(FTS_WAKEUP_TIME);
			retry--;
			goto retry_gesture_read;
		} else {
			dev_err(&fts_wq_data->client->dev,
				"%s read touchdata failed.\n", __func__);
			wake_unlock(&fts_wq_data->fts_wake_lock);
			return ret;
		}
	}

	gesture_id = buf[0];
	dev_dbg(&fts_i2c_client->dev, "gesture ID is 0x%x\n", gesture_id);
	fts_check_gesture(fts_input_dev, gesture_id);

	wake_unlock(&fts_wq_data->fts_wake_lock);
	return 0;
}
