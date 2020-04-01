#include <math.h>
#include "mathematics.h"	

float frac(float f)
{
	if(f>=0.0f)
	{
		return f-floor(f);
	}
	else
	{
		return f-ceil(f);
	}
}
