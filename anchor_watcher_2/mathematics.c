#include <math.h>
#include "mathematics.h"	

/** Calculate the great circle distance in metres between 2 points specified in degrees lat/long.
 *  Uses the Haversine formulae which assumes the earth is spherical. Error can be 0.5% maximum. */
float distance_between_points(float lat1, float long1, float lat2, float long2)
{
	float half_dlat;
	float half_dlong;
	float a;
	float c;

	lat1/=DEGREES_TO_RADS;
	long1/=DEGREES_TO_RADS;
	lat2/=DEGREES_TO_RADS;
	long2/=DEGREES_TO_RADS;
	half_dlat=(lat2-lat1)/2.0f;
	half_dlong=(long2-long1)/2.0f;
	a=sin(half_dlat)*sin(half_dlat)+sin(half_dlong)*sin(half_dlong)*cos(lat1)*cos(lat2); 
	c=2*atan2(sqrt(a), sqrt(1-a)); 
	return 6371000.0f*c; 
}

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
