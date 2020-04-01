#ifndef _MATHEMATICS_H
#define _MATHEMATICS_H

#define	DEGREES_TO_RADS				57.296

float convert_degrees_and_minutes_to_degrees_frac_degrees(float degrees, float minutes);
float distance_between_points(float lat1, float lon1, float lat2, float lon2);
float bearing_between_points(float lat1, float long1, float lat2, float long2);

#endif