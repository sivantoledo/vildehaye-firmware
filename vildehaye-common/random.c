/* -------------------------------------------------------------------------
 * This is an ANSI C library for multi-stream random number generation.
 * The use of this library is recommended as a replacement for the ANSI C
 * rand() and srand() functions, particularly in simulation applications
 * where the statistical 'goodness' of the random number generator is
 * important.  The library supplies 256 streams of random numbers; use
 * SelectStream(s) to switch between streams indexed s = 0,1,...,255.
 *
 * The generator used in this library is a so-called 'Lehmer random number
 * generator' which returns a pseudo-random number uniformly distributed
 * 0.0 and 1.0.  The period is (m - 1) where m = 2,147,483,647 and the
 * smallest and largest possible values are (1 / m) and 1 - (1 / m)
 * respectively.  For more details see:
 *
 *       "Random Number Generators: Good Ones Are Hard To Find"
 *                   Steve Park and Keith Miller
 *              Communications of the ACM, October 1988
 *
 * Name            : rngs.c  (Random Number Generation - Multiple Streams)
 * Authors         : Steve Park & Dave Geyer
 * Language        : ANSI C
 * Latest Revision : 09-22-98
 * -------------------------------------------------------------------------
 */

#include <stdint.h>

#define MODULUS    2147483647 /* DON'T CHANGE THIS VALUE                  */
#define MULTIPLIER 48271      /* DON'T CHANGE THIS VALUE                  */
#define CHECK      399268537  /* DON'T CHANGE THIS VALUE                  */
#define STREAMS    256        /* # of streams, DON'T CHANGE THIS VALUE    */

static int32_t _randomState;  /* current state of each stream   */

/*
 * m = 2,147,483,647
 * returns a positive integer between 1 and m-1.
 */
int32_t random(void) {
  const int32_t Q = MODULUS / MULTIPLIER;
  const int32_t R = MODULUS % MULTIPLIER;
        int32_t t;

  t = MULTIPLIER * (_randomState % Q) - R * (_randomState / Q);
  if (t > 0)
    _randomState = t;
  else
    _randomState = t + MODULUS;
  return _randomState;
}

/*
 * State must be between 1 and m-1.
 */
void randomSetState(int32_t x)
/* ---------------------------------------------------------------
 * Use this function to set the state of the current random number
 * generator stream according to the following conventions:
 *    if x > 0 then x is the state (unless too large)
 *    if x < 0 then the state is obtained from the system clock
 *    if x = 0 then the state is to be supplied interactively
 * ---------------------------------------------------------------
 */
{
	 if (x < 0)  x = -x;
	 if (x == 1) x = 1 ;
	 _randomState = x % MODULUS;
}

/*
 * returns 1 if correct, 0 otherwise.
 */
#if 0
static
int randomTest(void)
{
  int32_t   i;
  //int32_t   x;

  randomSetState(1);
  for(i = 0; i < 10000; i++) random();
  return (_randomState == CHECK);   /* and check for correctness */
}
#endif



