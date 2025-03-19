#ifndef __sampler_h__
#define __sampler_h__

struct sampler {
    unsigned long usec;	// time of last sense irq
    int sum;		// accounted usec in current pulse
    int min;		// min pulse width to account valid end of pulse
    int last;		// last revolution time
    int pulse;		// counter of accounted pulses
};

unsigned sampler_get_rpm(struct sampler *ss, unsigned long usec);
void reset_sampler(struct sampler *ss);
int do_sample(struct sampler *ss, unsigned long usec, int stable_edge);

#endif
