#ifndef __SIW_TOUCH_FEATURE_HTC_H
#define __SIW_TOUCH_FEATURE_HTC_H

int siw_set_cover_mode(struct siw_ts *ts, u16 enable);
int siw_set_glove_mode(struct siw_ts *ts, u16 enable);
int siw_set_edge_filter(struct siw_ts *ts, u16 enable);
int siw_set_status(struct siw_ts *ts, u16 value);
/*void siw_set_chip_mode(struct siw_ts *ts, u16 value);*/
#endif
