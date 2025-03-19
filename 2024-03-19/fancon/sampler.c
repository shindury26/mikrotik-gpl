#include <linux/kernel.h>
#include <linux/sort.h>
#include "sampler.h"

#define DEBUG_SAMPLER	0

unsigned sampler_get_rpm(struct sampler *ss, unsigned long usec) {
    if (usec - ss->usec > 1000000) {		// no recent data
	ss->last = 0;
#if DEBUG_SAMPLER
	printk("sampler: old data: now %lu last %lu\n", usec, ss->usec);
#endif
    }
#if DEBUG_SAMPLER
    printk("sampler: calc rpm from last val %d\n", ss->last);
#endif
    if (ss->last <= 0) return 0;		// no data at all
    return (60 * 1000000) / ss->last;		// calculare rpm from usec
}

void reset_sampler(struct sampler *ss) {
    ss->last = 0;
    ss->pulse = 0;
    ss->sum = 0;
    ss->min = 350;	// limits RPM at max 42k
}

int do_sample(struct sampler *ss, unsigned long usec, int stable_edge) {
    int sample = usec - ss->usec;
    ss->usec = usec;
    if (sample < 0 || sample > 500000 || ss->sum > 500000) {
	// something bad - reset & restart
	reset_sampler(ss);
	return 0;
    }

    // wait for long pulse before stable edge; account & ignore everything else
    ss->sum += sample;
    if (stable_edge == 0 || sample < ss->min) return 0;

    // pulse length should be >= 3/4 from previous sample length
    // it is needed on CCR1009-8G-1S at low rpm to filter out noise
    ss->min = sample * 3 / 4;

    // pulse 0 - wait for first edge, initial state
    // pulse 1 - got 1st edge, start measuring
    // pulse 2 - fan has rotated 180 degrees
    // pulse 3 - fan has rotated 360 degrees, first sample (ignore if bad)
    // pulse 4 - fan has rotated 180 degrees, 2nd rev
    // pulse 5 - fan has rotated 360 degrees, 2nd sample (ignore if bad)
    // pulse 6 - fan has rotated 180 degrees, 3rd rev
    // pulse 7 - fan has rotated 360 degrees, 3rd sample (use as it is)
    ++ss->pulse;
    if ((ss->pulse & 1) && ss->pulse > 2) {
	int deltaFromLast = abs(ss->sum - ss->last);
	if (deltaFromLast < ss->last / 4 || ss->last == 0 || ss->pulse > 6) {
#if DEBUG_SAMPLER
	    if (ss->pulse > 6) printk("sampler: use %d\n", ss->sum);
#endif
	    ss->last = ss->sum;		// good sample
	    ss->pulse = 1;
	}
	else {
#if DEBUG_SAMPLER
	    printk("sampler:ignore %d\n", ss->sum);
#endif
	}
	ss->sum = 0;
    }
    return 1;
}
