#include "Helper.h"
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
long millis()
{
	struct timeval start;

    gettimeofday(&start, NULL);
    long mtime, seconds, useconds;    
    seconds  = start.tv_sec;
    useconds = start.tv_usec;

    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    return mtime;
}
long us()
{
  struct timeval start;

    gettimeofday(&start, NULL);
    long mtime, seconds, useconds;    
    seconds  = start.tv_sec;
    useconds = start.tv_usec;

    mtime = ((seconds) * 1000000 + useconds) + 0.5;
    return mtime;
}
float constrain(float val,float min, float max){
    if (val>max)
        return max;
    if (val<min)
        return min;
    return val;
}