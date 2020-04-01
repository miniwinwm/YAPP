#include "mathematics.h"	

/** Convert an angle where integer part is degrees and fractional part is fraction of a minute to whole number degree value (integer and fractional parts). */
float convert_degrees_frac_minutes_2_degrees_frac_degrees(float dfd)
{
	float decimal_part=dfd-(int)dfd;
	float integer_part=dfd-decimal_part;
	decimal_part/=0.6;
	return integer_part+decimal_part;
}	

