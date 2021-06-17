/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME net_openthread_alarm
#define LOG_LEVEL CONFIG_OPENTHREAD_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <kernel.h>
#include <string.h>
#include <inttypes.h>

#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread-system.h>

#include <stdio.h>

#include "platform-zephyr.h"

static bool timer_ms_fired, timer_us_fired;

static void ot_timer_ms_fired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	timer_ms_fired = true;
	otSysEventSignalPending();
}

static void ot_timer_us_fired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	timer_us_fired = true;
	otSysEventSignalPending();
}

K_TIMER_DEFINE(ot_ms_timer, ot_timer_ms_fired, NULL);
K_TIMER_DEFINE(ot_us_timer, ot_timer_us_fired, NULL);

void platformAlarmInit(void)
{
	/* Intentionally empty */
}

void platformAlarmProcess(otInstance *aInstance)
{
	if (timer_ms_fired) {
		timer_ms_fired = false;
		otPlatAlarmMilliFired(aInstance);
	}
#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
	if (timer_us_fired) {
		timer_us_fired = false;
		otPlatAlarmMicroFired(aInstance);
	}
#endif
}

uint32_t otPlatAlarmMilliGetNow(void)
{
	return k_uptime_get_32();
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
	ARG_UNUSED(aInstance);

	int64_t reftime = (int64_t)aT0 + (int64_t)aDt;
	int64_t delta = -k_uptime_delta(&reftime);

	if (delta > 0) {
		k_timer_start(&ot_ms_timer, K_MSEC(delta), K_NO_WAIT);
	} else {
		ot_timer_ms_fired(NULL);
	}
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	k_timer_stop(&ot_ms_timer);
}

void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
	ARG_UNUSED(aInstance);

	uint64_t reftime = aT0 + aDt;
	uint64_t curtime = k_ticks_to_us_floor64(k_uptime_ticks());
	int64_t delta = reftime - curtime;

	if (delta > 0) {
		k_timer_start(&ot_us_timer, K_USEC(delta), K_NO_WAIT);
	} else {
		ot_timer_us_fired(NULL);
	}
}

void otPlatAlarmMicroStop(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	k_timer_stop(&ot_us_timer);
}

uint32_t otPlatAlarmMicroGetNow(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}
