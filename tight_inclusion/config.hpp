#pragma once

// WARNING: Do not modify config.hpp directly. Instead, modify config.hpp.in.

#define TIGHT_INCLUSION_NAME "ccdTest"
#define TIGHT_INCLUSION_VER ""
#define TIGHT_INCLUSION_VER_MAJOR ""
#define TIGHT_INCLUSION_VER_MINOR ""
#define TIGHT_INCLUSION_VER_PATCH ""

/* #undef TIGHT_INCLUSION_WITH_RATIONAL */
/* #undef TIGHT_INCLUSION_WITH_TIMER */
/* #undef TIGHT_INCLUSION_WITH_DOUBLE_PRECISION */
/* #undef TIGHT_INCLUSION_LIMIT_QUEUE_SIZE */
/* #undef TIGHT_INCLUSION_FLOAT_WITH_DOUBLE_INPUT */

// #define TIGHT_INCLUSION_CHECK_QUEUE_SIZE
// #define TIGHT_INCLUSION_USE_MAX_ABS_TOL

#ifdef TIGHT_INCLUSION_LIMIT_QUEUE_SIZE
#define MAX_QSIZE 1000
#endif
