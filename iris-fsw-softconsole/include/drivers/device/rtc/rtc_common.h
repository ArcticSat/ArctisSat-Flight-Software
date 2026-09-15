#ifndef RTC_COMMON_H_
#define RTC_COMMON_H_
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Common definitions and macros for all RTC types.
//
// History
// 2019-04-17 by Tamkin Rahman
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <firmware/hal/CortexM3/GNU/cpu_types.h>
#include "drivers/mss_rtc/mss_rtc.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define MIN_SECONDS 0u
#define MAX_SECONDS 59u

#define MIN_MINUTES 0u
#define MAX_MINUTES 59u

#define MIN_HOURS 0u
#define MAX_HOURS 23u

#define MIN_DAYS 1u
#define MAX_DAYS 31u

#define MIN_MONTHS 1u
#define MAX_MONTHS 12u

#define MIN_YEARS  0u
#define MAX_YEARS  99u
#define YEAR_OFFSET 2000u

// The internal RTC runs off of the 50 MHz RC clock.
#define TICKS_TO_SECONDS 50000000u

// Rough conversions for the helper macro below.
#define SECONDS_IN_MINUTE 60
#define MINUTES_IN_HOUR   60
#define HOURS_IN_DAY      24
#define DAYS_IN_MONTH     31
#define MONTHS_IN_YEAR    12

#define SECONDS_IN_HOUR   (SECONDS_IN_MINUTE * MINUTES_IN_HOUR)
#define SECONDS_IN_MONTH  (SECONDS_IN_HOUR * HOURS_IN_DAY * DAYS_IN_MONTH)
#define SECONDS_IN_YEAR   (SECONDS_IN_MONTH * MONTHS_IN_YEAR)

// Helper macro used to convert a Calendar_t object to an unsigned long (for comparison operations). Note that it
// does not correlate one to one into seconds since epoch.
#define CALENDAR_TO_LONG(time) (time)->second \
                               + (time)->minute * SECONDS_IN_MINUTE \
							   + (time)->hour   * SECONDS_IN_HOUR \
							   + (time)->month  * SECONDS_IN_MONTH \
							   + (time)->year   * SECONDS_IN_YEAR

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTURES AND STRUCTURE TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
 *
 * The calendar type will have the following restrictions:
	typedef struct
	{
		uint8_t second;  // The current seconds (0-59).
		uint8_t minute;  // The current minutes (0-59).
		uint8_t hour;    // The current hour (0-23).
		uint8_t day;     // The current day (1-31).
		uint8_t month;   // The current month (1-12).
		uint8_t year;    // The current year (70 or greater, with 70 being 1970).
		uint8_t weekday; // The current weekday (1-7, with 1 being Sunday).
		uint8_t week;    // Not used. Kept for backward compatibility with MSS_RTC. MUST BE 1 or greater.
	} Calendar_t;
*/

typedef mss_rtc_calendar_t Calendar_t;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Compare two calendar times, and check which is later than the other. The times are assumed to be valid.
//
// Returns:
//  -1, if time1 is earlier than time2
//  0, if time1 is equal to time2
//  1, if time1 is later than time2
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static inline int compare_time(Calendar_t * time1, Calendar_t * time2)
{
	unsigned long time1_long = CALENDAR_TO_LONG(time1);
	unsigned long time2_long = CALENDAR_TO_LONG(time2);

	if (time1_long < time2_long)
	{
		return -1;
	}
	else if (time1_long > time2_long)
	{
		return 1;
	}
	return 0;
}

#endif /* RTC_COMMON_H_ */
