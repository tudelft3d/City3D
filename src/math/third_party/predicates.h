
#ifndef _MATH_PREDICATES_H_
#define _MATH_PREDICATES_H_

#include "../math_common.h"


// Liangliang: It seems float may not work properly for some of my data, so I always use double.
//#define SINGLE
#ifdef SINGLE
#define REAL float
#else
#define REAL double                      /* float or double */
#endif

/*    orient2d(pa, pb, pc)                                                   */
/*    orient2dfast(pa, pb, pc)                                               */
/*    orient3d(pa, pb, pc, pd)                                               */
/*    orient3dfast(pa, pb, pc, pd)                                           */
/*    incircle(pa, pb, pc, pd)                                               */
/*    incirclefast(pa, pb, pc, pd)                                           */
/*    insphere(pa, pb, pc, pd, pe)                                           */
/*    inspherefast(pa, pb, pc, pd, pe)                                       */

void exactinit(int, int, int, REAL, REAL, REAL);

REAL orient2d(REAL *pa, REAL *pb, REAL *pc);

REAL orient2dfast(REAL *pa, REAL *pb, REAL *pc);

REAL orient3d(REAL *pa, REAL *pb, REAL *pc, REAL *pd);

REAL orient3dfast(REAL *pa, REAL *pb, REAL *pc, REAL *pd);

REAL orient3dexact(REAL *pa, REAL *pb, REAL *pc, REAL *pd);

REAL incircle(REAL *pa, REAL *pb, REAL *pc, REAL *pd);

REAL incirclefast(REAL *pa, REAL *pb, REAL *pc, REAL *pd);

REAL insphere(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL *pe);

REAL inspherefast(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL *pe);

REAL orient4d(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL *pe, REAL ah, REAL bh, REAL ch, REAL dh, REAL eh);

#endif
