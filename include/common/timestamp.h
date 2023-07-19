/**
 * @file
 * @brief  Interface to timestamp handling.
 *
 * All rights reserved. Reproduction in whole or in part without the
 * written consent of CNGIC UVC is prohibited.
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H


#include <stdbool.h>


#if defined(__cplusplus)
extern "C" {
#endif


/*------------------------------------------------------------------------*
 *  global definitions                                                    *
 *------------------------------------------------------------------------*/

/**
 * @brief  A timestamp with microsecond resolution.
 *
 * Please note that negative timestamps are not supported, and their
 * use has undefined behavior.
 */
typedef long long int timestamp_t;

/**
 * @brief  Difference between variables of type timestamp_t.
 */
typedef long long int timestamp_diff_t;


/**
 * @brief  Possible return values for timestamp_get_clock_type().
 */
enum eClockType
{
	 TS_TSC_CLOCK           = 1     ///< TSC-based time
	,TS_REALTIME_CLOCK      = 2     ///< system time with CLOCK_REALTIME clock
	,TS_MONOTONIC_CLOCK     = 3     ///< system time with CLOCK_MONOTONIC clock
	,TS_RAW_MONOTONIC_CLOCK = 4     ///< system time with CLOCK_MONOTONIC_RAW clock
};

/*------------------------------------------------------------------------*
 *  global functions                                                      *
 *------------------------------------------------------------------------*/

/**
 * @brief   Initialize high-precision timer.
 *
 * This function should be called once upon application startup,
 * before any timestamps are being acquired.
 */
void timestamp_init(void);

/**
 * @brief   Return whether the timestamp generation uses the raw processor TSC.
 *
 * A raw processor TSC (Time Stamp Counter) is usually a single processor
 * instruction ("rdtsc" on IA32-compatible processors) and is very fast.
 *
 * On multi-core systems, these raw TSCs are not necessarily in sync with
 * each other, so unless all threads using these timestamps are running
 * on the same core, there can be big jumps (forward as well as backwards)
 * in two timestamps taken immediately afterwards, depending on which core
 * is currently running the thread.
 *
 * For this reason, this library can be configured (at compile-time) to
 * use timestamps that are being fetched with clock_gettime(), so they are
 * kept in sync by Linux with all cores.
 *
 * The mode for which a given library instance has been configured can be
 * queried with this function.
 *
 * @retval  true  if using TSC directly
 * @retval  false  if using a syscall-based timestamp
 */
bool timestamp_is_raw(void);

/**
 * @brief   Return the type of clock used.
 *
 * See timestamp_is_raw() for an explanation of the differences between
 * TSC-based and system clocks. A system clock can again be configured
 * to use different representation. See clock_gettime(3) for an
 * explanation of these.
 *
 * The clock type of this module is configured during compile time
 * and cannot be changed during runtime.
 *
 * @return  clock type used by this module
 */
int timestamp_get_clock_type(void);

/**
 * @brief   Return timestamp.
 *
 * The timestamps returned with this function are always positive and
 * monotonic - that is, no timestamp retrieved with this function
 * is smaller than any of the previously fetched timestamps since
 * the last processor reset.
 *
 * The behavior of negative timestamps is undefined.
 *
 * @return  timestamp in microseconds
 */
timestamp_t xtime_microseconds(void);

/**
 * @brief   Return resolution of timestamps from xtime_microseconds().
 *
 * While xtime_microseconds() always returns a timestamp as a number
 * of microseconds, the real resolution of these timestamps is not
 * necessarily in the order of microseconds.
 *
 * This function returns the real resolution of implementation's timestamp
 * generator. It is returned as the smallest possible step between
 * two successive timestamps.
 *
 * The minimum resolution that can be returned here is 1 us; if the actual
 * clock resolution is smaller than that, it is still returned as 1.
 *
 * The value 0 is returned when the implementation does not support this
 * operation; this could happen, for instance, when the system has no
 * POSIX realtime support. If this is the case, it is highly likely that
 * any timestamp that is returned from xtime_microseconds() is invalid
 * as well.
 *
 * @note
 * The accuracy reported by this function may be off by an arbitrary
 * amount, due to deficiencies in the Linux kernel. In particular, it
 * may report the resolution of Linux' clock timer, which usually runs
 * with something between 100 Hz and 1000 Hz, which results in a reported
 * resolution of 1 to 10 ms. In reality, most current Linux systems
 * support a much higher resolution in the nanoseconds range.@n
 * For the time being, the only useful thing that can be done with this
 * function is to check whether it returns 0.
 *
 * @return  timestamp resolution in us, or 0
 */
timestamp_diff_t xtime_microseconds_resolution(void);


#if defined(__cplusplus)
}
#endif


#endif  // TIMESTAMP_H
