/*
 *	junjun@2012 initial version
 *	This file is used to provide flash read write store function
 *
 */

#ifndef __FILTER_H_
#define __FILTER_H_

struct filter_s {
	long b0, b1, bn; //num
	long a0, a1, an; //den
	long xn_1, yn_1;
};

void filter_init(struct filter_s *f);
long filt(struct filter_s *f, long xn);


#endif

