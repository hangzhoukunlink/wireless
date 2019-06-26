#include "filter.h"

void filter_init(struct filter_s *f)
{
	f->bn = 13;
	f->an = 13;

#if 0  //80Hz
	f->b0 = (long)(0.43417375120630208 * (1 << f->bn));
	f->b1 = (long)(0.43417375120630208 * (1 << f->bn));
	f->a0 = (long)(1.00000000000000000 * (1 << f->an));
	f->a1 = (long)(-0.13165249758739583 * (1 << f->an));
#else	//20Hz
	f->b0 = (long)(0.21132486540518713 * (1 << f->bn));
	f->b1 = (long)(0.21132486540518713 * (1 << f->bn));
	f->a0 = (long)(1.00000000000000000 * (1 << f->an));
	f->a1 = (long)(-0.57735026918962573 * (1 << f->an));
#endif

	f->xn_1 = 0;
	f->yn_1 = 0;
}

long filt(struct filter_s *f, long xn)
{
	long vb, va;
	vb = f->b0 * xn;
	vb += f->b1 * f->xn_1;
	vb >>= f->bn;

	va = f->a1 * f->yn_1;
	va = 0 - va;
	va >>= f->an;

	vb += va;

	f->xn_1 = xn;
	f->yn_1 = vb;
	return vb;
}
