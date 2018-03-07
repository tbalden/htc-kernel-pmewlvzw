#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_feature_htc.h"

int siw_set_cover_mode(struct siw_ts *ts, u16 enable)
{
//	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
//	struct device *dev = ts->dev;
	return 0;
}

int siw_set_glove_mode(struct siw_ts *ts, u16 enable)
{
//	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
//	struct device *dev = ts->dev;
	return 0;
}

int siw_set_edge_filter(struct siw_ts *ts, u16 enable)
{
//	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
//	struct device *dev = ts->dev;
	return 0;
}

int siw_set_status(struct siw_ts *ts, u16 value)
{
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
	struct device *dev = ts->dev;

	if (!mode_ctrl->support_glove && !mode_ctrl->support_cover) {
		return 0;
	}

	if (mode_ctrl->bus_to_mcu) {
		t_dev_info(dev, "%s: bus is already switched to MCU\n", __func__);
		return 0;
	}

	if (value & 0x02) {
		siw_set_glove_mode(ts, 0);
		siw_set_cover_mode(ts, value & 0x02);
		t_dev_info(dev, "mode %d (cover)\n", value);
	}
	else if (value & 0x01) {
		siw_set_cover_mode(ts, 0);
		siw_set_glove_mode(ts, value & 0x01);
		t_dev_info(dev, "mode %d (glove)\n", value);
	}
	else {
		siw_set_cover_mode(ts, 0);
		siw_set_glove_mode(ts, 0);
		t_dev_info(dev, "mode %d (normal)\n", value);
	}

	t_dev_dbg(DBG_BASE, dev, " %s: done\n", __func__);
	return 0;
}
/*
void siw_set_chip_mode(struct siw_ts *ts, u16 value)
{
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
	struct device *dev = ts->dev;

	if (!mode_ctrl->support_glove && !mode_ctrl->support_cover) {
		return;
	}

	if (mode_ctrl->bus_to_mcu) {
		t_dev_info(dev, "%s: bus is already switched to MCU\n", __func__);
		return 0;
	}

	siw_set_edge_filter(siw, mode_ctrl->cover_mode);
	if (mode_ctrl->cover_mode) {
		siw_set_status(siw, mode_ctrl->cover_mode << 1);
	}
	else if (mode_ctrl->glove_setting & 0x01) {
		siw_set_status(siw, 1);
		mode_ctrl->glove_enable = 1;
	}
	else {
		siw_set_status(siw, 0);
		mode_ctrl->glove_enable = 0;
	}
}*/
