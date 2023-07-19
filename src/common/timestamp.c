/**
 * @file
 * @brief  Implementation of timestamp handling.
 *
 * All rights reserved. Reproduction in whole or in part without the
 * written consent of CNGIC UVC is prohibited.
 */

#include <common/timestamp.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <time.h>
#include <assert.h>

#ifdef WIN32
#include <Windows.h>
#endif


/*------------------------------------------------------------------------*
 *  local definitions                                                     *
 *------------------------------------------------------------------------*/

// May have set the preprocessor symbol WITH_FAST_TIMESTAMP on the compiler command-line.
#ifndef WITH_FAST_TIMESTAMP
#if defined(__i386__)
// Default behavior is not to use fast rdtsc-based timestamp generation,
// because it does not work well on multi-core systems.
# define WITH_FAST_TIMESTAMP  0
#elif defined(__arm__)
// ARM does not have a timestamp counter - fall-back to syscall-based solution.
# define WITH_FAST_TIMESTAMP  0
#else
// Unknown system - fall-back to syscall-based solution.
# define WITH_FAST_TIMESTAMP  0
#endif
#endif


/* type for high precision timer */
typedef unsigned long long int hp_timing_t;

/*------------------------------------------------------------------------*
 *  local variables                                                       *
 *------------------------------------------------------------------------*/

#if (WITH_FAST_TIMESTAMP - 0) == 1
/* the processor's clock frequency, read once from /proc/cpuinfo in timestamp_init() */
static hp_timing_t g_clockfreq;
#else

# ifdef WIN32
// Nothing to be done here; raw clocks currently not supported on Windows.
# else
// The Linux system clock to use.

#  if (WITH_MONOTONIC_CLOCK - 0) == 1

#   ifdef CLOCK_MONOTONIC_RAW
// CLOCK_MONOTONIC_RAW is available since Linux 2.6.28.
// Cannot jump; cannot slew.
// This is the best clock available for precise measuring of time invervals.
static int const g_clockId = CLOCK_MONOTONIC_RAW;
#   else
// CLOCK_MONOTONIC should be available on all Linux kernels since 2.6
// Cannot jump, but can slew (because of ntp).
static int const g_clockId = CLOCK_MONOTONIC;
#   endif  // CLOCK_MONOTONIC_RAW

#  else
// Always available.
// Can both jump and slew.
// This one implements legacy behavior and should not be used in new code.
static int const g_clockId = CLOCK_REALTIME;
#  endif

# endif

#endif

/*------------------------------------------------------------------------*
 *  local functions                                                       *
 *------------------------------------------------------------------------*/

#if (WITH_FAST_TIMESTAMP - 0) == 1
/*
 * inspired by glibc's __get_clockfreq(void)
 */
static hp_timing_t get_clockfreq(void)
{
	/* We read the information from the /proc filesystem.  It contains at
	   least one line like
	     cpu MHz         : 497.840237
	   or also
	     cpu MHz         : 497.841
	   We search for this line and convert the number in an integer.
	*/

	static hp_timing_t result;
	int fd;

	/* If this function was called before, we know the result.  */
	if (result != 0)
		return result;

	fd = open("/proc/cpuinfo", O_RDONLY);
	if (__builtin_expect(fd != -1, 1)) {
		/* XXX AFAIK the /proc filesystem can generate "files" only up
		   to a size of 4096 bytes.  */
		char buf[4097];
		ssize_t n;

		buf[4096] = '\0';
		n = read(fd, buf, sizeof buf);
		if (__builtin_expect(n, 1) > 0) {
			char *mhz = strstr(buf, "cpu MHz");

			if (__builtin_expect(mhz != NULL, 1)) {
				char *endp = buf + n;
				int seen_decpoint = 0;
				int ndigits = 0;

				/* Search for the beginning of the string.  */
				while (mhz < endp && (*mhz < '0' || *mhz > '9') && *mhz != '\n')
					mhz += 1;

				while (mhz < endp && *mhz != '\n') {
					if (*mhz >= '0' && *mhz <= '9') {
						result *= 10;
						result += *mhz - '0';
						if (seen_decpoint)
							ndigits += 1;
					} else if (*mhz == '.')
						seen_decpoint = 1;

					mhz += 1;
				}

				/* Compensate for missing digits at the end.  */
				while (ndigits++ < 6)
					result *= 10;
			}
		}
		close(fd);
	}

	return result;
}

/*
 * Return processor's time-stamp counter.
 * The processor increments the time-stamp counter MSR every clock cycle
 * and resets it to 0 whenever the processor is reset.
 * While the returned value is an unsigned 64-bit one, I treat it as signed
 * for my convenience, since even on a 10 GHz processor, the top bit will not
 * be set for approx. 30 years.
 */
static inline long long int nanotime(void)
{
	long long int val;

#if defined __i386__
// FIXME: would rather have to check for >=Pentium 2
//        or do some test that catches illegal instructions
// FIXME: maybe rather use rdtscp instruction? (serializing variant of rdtsc)
// FIXME: or use some serializing instruction (eg. cpuid) before rdtsc?
// FIXME: check this out: http://lkml.org/lkml/2005/11/4/173

// (This instruction is available since Intel's Pentium 2 processor.)

// The x86-specific constraint "A" means %edx:%eax for High-Word:Low-Word,
// which is exactly what 'rdtsc' uses as destination. So we don't even
// need to specify %edx and %eax as being clobbered.
// The constraint modifier "=" defines it to be a write-only operand.
	__asm__ __volatile__ ("rdtsc"
			      : "=A" (val)
			      /* no input registers */
			      /* no clobbered registers */
			     );
	// FIXME: I've read that some processors (eg. Pentium II) have a bug in their rdtsc
	//        handling that leads to more than the expected registers being touched
	//        by this instruction (%eax and %ebx have been mentioned).
	//        For these, we would probably have to save some registers manually
	//        before executing rdtsc (or use the clobbered list).
	//        ref: http://www.codeproject.com/datetime/ccputicker.asp?df=100&forumid=429&exp=0&select=5653
#elif defined __x86_64__
	// The result of the rdtsc instruction is returned in %edx:%eax as well; their upper 32 bits are cleared.
	// Cannot use "A" though, since that would read a single 64-bit register.
	unsigned long a, d;
	__asm__ __volatile__ ("rdtsc"
			      : "=a" (a), "=d" (d)
			      /* no input registers */
			      /* no clobbered registers */
			     );
	val = a | (d << 32);
#else
	#error "Clock cycle timer not supported on this platform."
#endif

	return val;
}
#endif  // WITH_FAST_TIMESTAMP == 1

/*------------------------------------------------------------------------*
 *  global functions                                                      *
 *------------------------------------------------------------------------*/

void timestamp_init(void)
{
#if (WITH_FAST_TIMESTAMP - 0) == 1
	if (!g_clockfreq)
		g_clockfreq = get_clockfreq();
#endif

#if 0
	struct timespec timespec;
	if (clock_getres(CLOCK_MONOTONIC, &timespec) == 0) {
		fprintf(stderr, "Monotonic clock resolution is %ld.%09lds.\n", timespec.tv_sec, timespec.tv_nsec);

		double clockstep = timespec.tv_nsec;
		clockstep /= 1*1000.0*1000.0*1000.0;
		clockstep += timespec.tv_sec;
		fprintf(stderr, "Monotonic clock step is %.12fs.\n", clockstep);

		const double clockfreq = 1.0 / clockstep;
		fprintf(stderr, "Monotonic clock freq is %.12f Hz\n", clockfreq);


		fprintf(stderr, "Kernel clock freq is %lld Hz\n", g_clockfreq);
		fprintf(stderr, "Kernel clock step is %.12fs.\n", 1.0 / g_clockfreq);

		// Now test it.
		const long long nt = nanotime();

		// Using kernel freq.
		const double kCPS = (double)g_clockfreq;
		const double kiCPS = 1000.0 * 1000.0 * 1.0 / kCPS;
		fprintf(stderr, "kernel: nanotime %lld -> %lld us\n", nt, llrint(kiCPS * nt));

		// Using clock freq.
		const double cCPS = (double)clockfreq;
		const double ciCPS = 1000.0 * 1000.0 * 1.0 / cCPS;
		fprintf(stderr, "clock:  cCPS = %.12f, ciCPS = %.12f\n", cCPS, ciCPS);
		long long dd = llrint(ciCPS * nt);
		fprintf(stderr, "clock:  nanotime %lld -> %lld us\n", nt, llrint(ciCPS * nt));
	} else {
		if (errno == EINVAL)
			fprintf(stderr, "Monotonic clock not supported.\n");
		else
			fprintf(stderr, "Error on retrieving resolution of monotonic clock: %s.\n", strerror(errno));
	}
#endif
}

int timestamp_get_clock_type(void)
{
#if (WITH_FAST_TIMESTAMP - 0) == 1
	return TS_TSC_CLOCK;
#else
#ifdef WIN32
	// Currently not really supported on Windows.
	return -1;
#else
	switch (g_clockId) {
	case CLOCK_REALTIME:
		return TS_REALTIME_CLOCK;
	case CLOCK_MONOTONIC:
		return TS_MONOTONIC_CLOCK;
#ifdef CLOCK_MONOTONIC_RAW
	case CLOCK_MONOTONIC_RAW:
		return TS_RAW_MONOTONIC_CLOCK;
#endif
	default:
		assert(!"Invalid clock configured");
		return -1;
	}
#endif
#endif
}

bool timestamp_is_raw(void)
{
#if (WITH_FAST_TIMESTAMP - 0) == 1
	return true;
#else
	return false;
#endif
}

timestamp_t xtime_microseconds(void)
{
#if (WITH_FAST_TIMESTAMP - 0) == 1
	static unsigned initialized = false;
	static double iCPS;

	if (__builtin_expect(!initialized, 0)) {
		if (!g_clockfreq)
			g_clockfreq = get_clockfreq();
		double CPS = (double)g_clockfreq;
		iCPS = 1000.0 * 1000.0 * 1.0 / CPS;
		initialized = true;
	}
	// see also arch/i386/kernel/timers/timer_tsc.c

	return (timestamp_t)llrint(iCPS * nanotime());
#else
	#ifdef WIN32
		// Quick'n'dirty usage of Windows HPT
		static bool have;
		static __int64 pcFrequency;
		if (!have) {
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			pcFrequency = freq.QuadPart;
		}

		LARGE_INTEGER li;
		li.QuadPart = 0;
		QueryPerformanceCounter(&li);
		return (li.QuadPart * 1000000LL / pcFrequency);
	#else
		// Simulate with clock_gettime()
		// This is actually a better approach anyway (since it is synchronized
		// with gettimeofday()), but presumably slower, since it needs a context switch.
		struct timespec currtime;
		clock_gettime(g_clockId, &currtime);
		long long timestamp_us = (long long)currtime.tv_sec * 1000000LL;
		timestamp_us += (long long)currtime.tv_nsec / 1000LL;
		return timestamp_us;
	#endif
#endif
}

timestamp_diff_t xtime_microseconds_resolution(void)
{
#if (WITH_FAST_TIMESTAMP - 0) == 1
	// But with our current processors with > 800 MHz, a resolution
	// of 1us seems doable.
	return 1;
#else
	#ifdef WIN32
		// Quick'n'dirty usage of Windows HPT
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		__int64 pcFrequency = freq.QuadPart;
		__int64 resolution_us = 1000000LL / pcFrequency;
		if (resolution_us == 0)
			resolution_us = 1;
		return resolution_us;
	#else
		struct timespec clockres;
		const int ret = clock_getres(g_clockId, &clockres);
		if (ret == -1)
			return 0;
		long long resolution_us = (long long)clockres.tv_sec * 1000000LL;
		resolution_us += (long long)clockres.tv_nsec / 1000LL;
		// If the reported clock resolution is less than 1 us, round up to 1 us.
		if (resolution_us == 0)
			resolution_us = 1;
		return resolution_us;
	#endif
#endif
}
