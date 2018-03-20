#include "FC.h"
#include <stdio.h>
int main()
{
	printf("Starting\n");
	FC& fc=FC::instance();
    fc.Init();
	while (1)
	{
	 fc.Update();
	}
	return 0;
}
