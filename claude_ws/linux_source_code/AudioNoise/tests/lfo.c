#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SAMPLES_PER_SEC (48000.0)

#include "../audio/util.h"
#include "../audio/lfo.h"

static struct lfo_state lfo;

int main(int argc, char **argv)
{
	float maxerr = 0;
	u32 maxidx = 0;

	// This checks every 2**32 value and can be very slow.
	// Only run on a fast machine
	lfo.step = 1;
	do {
		u32 idx = lfo.idx;
		float val = lfo_step(&lfo, lfo_sinewave);
		float s = (idx / TWO_POW_32) * 2 * M_PI;
		float exact = sin(s);
		float err = fabs(val - exact);
		if (err > maxerr) {
			maxerr = err;
			maxidx = idx;
		}
	} while (lfo.idx);

	printf("Max LFO sinewave error %.8f at %u\n", maxerr, maxidx);
	return 0;
}
