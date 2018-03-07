/*
 * include/linux/htc_mcuroute.h - The mcuroute header
 * Copyright (C) 2009-2016  HTC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __HTC_MCUROUTE_H
#define __HTC_MCUROUTE_H

#ifdef __KERNEL__
#ifdef CONFIG_HTC_MCUROUTE
extern void mcuroute_lock(void);
extern int mcuroute_trylock(void);
extern void mcuroute_unlock(void);
extern int mcuroute_get_master_active(void);
extern void mcuroute_set_master_active(bool);
#else
static inline void mcuroute_lock(void)
{
	return;
}

static inline int mcuroute_trylock(void)
{
	return 1;
}

static inline void mcuroute_unlock(void)
{
	return;
}

static inline int mcuroute_get_master_active(void)
{
	return true;
}

static inline void mcuroute_set_master_active(bool en)
{
	return;
}
#endif
#endif

#endif
