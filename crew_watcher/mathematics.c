#include <math.h>
#include "mathematics.h"	

float convert_degrees_and_minutes_to_degrees_frac_degrees(float degrees, float minutes)
{
	unsigned char coord_negative;
	float result;

	coord_negative=degrees<0.0f;
	result=fabs(degrees)+minutes/60.0f;
	if(coord_negative)
	{
		result=-result;
	}	

	return result;	
}

/** Calculate the degrees bearing between 2 points defined by degrees lat/long. */
float bearing_between_points(float lat1, float long1, float lat2, float long2)
{
	float dlong;
	float x, y;
	float bearing;	
	
	lat1/=DEGREES_TO_RADS;
	long1/=DEGREES_TO_RADS;
	lat2/=DEGREES_TO_RADS;
	long2/=DEGREES_TO_RADS;
	
	dlong=long2-long1;
	y=sin(dlong)*cos(lat2);
	x=cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlong);
	bearing=atan2(y, x)*DEGREES_TO_RADS;	
		
	if(bearing<0.0)
	{
		bearing+=360.0;
	}
	
	return bearing;
}	

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
	half_dlat=(lat2-lat1)/2.0;
	half_dlong=(long2-long1)/2.0;
	a=sin(half_dlat)*sin(half_dlat)+sin(half_dlong)*sin(half_dlong)*cos(lat1)*cos(lat2); 
	c=2*atan2(sqrt(a), sqrt(1-a)); 
	return 6371000.0*c; 
}


