#ifndef _ASSERT_H_
#define _ASSERT_H_

#define assert(x) ((void)0)

#ifndef static_assert
#define static_assert(expr, msg) _Static_assert(expr, msg)
#endif

#endif
