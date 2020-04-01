#include <stdarg.h>
#include <ctype.h>
#include "sscanf.h"

/** Standard atof stuff, but returns where it stopped scanning as well */
float my_atof(char *s, char **end_ptr) 
{ 
	float power_of_10=1.0; 
	float result=0.0; 
	unsigned char sign=FALSE; 
	char c; 
	
	c=*s; 
	if(c=='-') 
	{ 
		sign=TRUE; 
		c=*(++s); 
	} 
	else if(c=='+') 
	{
		c=*(++s); 
	}
	
	while((c>='0' && c<='9')) 
	{ 
		result=10*result + c-'0'; 
		c=*(++s); 
	} 
	
	if(c=='.') 
	{ 
		c=*(++s); 
		while((c>='0' && c<='9')) 
		{ 
			power_of_10=power_of_10*10; 
			result+=(c-'0')/power_of_10; 
			c=*(++s); 
		} 
	} 
	
	if(sign) 
	{
		result=-result; 
	}
	*end_ptr = s; 

	return(result); 
} 

/** Standard atol stuff, but returns where it stopped scanning as well */
unsigned long my_atoul(char *s, char **end_ptr, int base) 
{ 
	unsigned long result=0; 
	char c; 
	
	*end_ptr=s; 
	
	if((!s) || (!*s )) 
	{
		return 0; 
	}
	
	c=*s; 
	if(c=='+') 
	{ 
		c=*(++s); 
	} 
	
	if(base==10) 
	{ 
		while(c>='0' && c<='9') 
		{ 
			result=10*result + (c-'0'); 
			c=*(++s); 
		} 
	} 
	else if(base==16)    
	{ 
		if(c=='0' && (*(s+1)=='x' || *(s+1)=='X')) 
		{ 
			s+=2; 
			c=*s; 
		} 
	
		c=toupper(c); 
		while(1) 
		{ 
			if(c>='0' && c<='9') 
			result=(result<<4) + (c-'0'); 
			else if(c>='A' && c<='F') 
			{
				result=(result<<4) + (c-'A' + 10); 
			}
			else 
			{
				break; 
			}
			c=toupper(*(++s)); 
		} 
	} 
	*end_ptr=s; 

	return result; 
} 

/** A cut down version of sscanf. Supports %c, %f, %d %ld, %x, %s, %u */
unsigned char sscanf(char *buf, const far rom char *format, ...)  
{ 
	va_list vargs_ptr; 
	unsigned char count=0; 
	void *p; 
	unsigned char is_long; 
	unsigned char sign; 
	char *end_ptr; 

	va_start(vargs_ptr, format); 

	while(1) 
	{ 
		while(*buf==*format) 
		{ 
			if((*buf==0) || (*format==0)) 
			{
				return (count); 
			}
			
			buf++; 
			format++; 
		} 

		if(*format!='%') 
		{
			break; 
		}
		format++; 

		if(toupper(*format)=='L')
		{
			is_long=TRUE;
			format++;
		}
		else
		{
			is_long=FALSE;
		}

		switch(*format) 
		{ 
			case 'f': 
			case 'F': 
			{
				*((float *)va_arg(vargs_ptr, float*))=my_atof(buf, &end_ptr); 
				
				if(buf==end_ptr) 
				{
					return(count);        
				}
				buf=end_ptr; 

				count++; 
				break; 
			}		

			case 'd': 
			case 'D': 
			{
				if(*buf=='-') 
				{ 
					buf++; 
					sign=TRUE; 
				} 
				else 
				{
					sign=FALSE; 
				}

				if(is_long) 
				{ 
	        		p=va_arg(vargs_ptr, long*); 
					*(signed long *)p=(signed long)my_atoul(buf, &end_ptr, 10); 
					if(sign) 
					{
						*(signed long *)p=-(*(signed long *)p); 
					}
				} 
				else 
				{ 
	        		p=va_arg(vargs_ptr, int*); 
					*(signed int *)p=(signed int)my_atoul(buf, &end_ptr, 10); 
					if(sign) 
					{
						*(signed int *)p=-(*(signed int *)p); 
					}
				} 

				if(buf==end_ptr ) 
				{
					return(count); 
				}
				buf=end_ptr; 
				count++; 
				break; 
			}	

/* removed to save space
			case 'u': 
			case 'U': 
			{
				if(is_long) 
				{
					*((unsigned long*)va_arg(vargs_ptr, unsigned long*))=(unsigned long)my_atoul(buf, &end_ptr, 10); 
				}
				else 
				{
					*((unsigned int*)va_arg(vargs_ptr, unsigned int*))=(unsigned int)my_atoul(buf, &end_ptr, 10); 
				}
				
				if(buf==end_ptr) 
				{
					return(count); 
				}
				buf=end_ptr; 
				
				count++; 
				break; 
			}	

			case 's': 
			{
				p=(char*)va_arg(vargs_ptr, char*); 
				
				while(TRUE) 
				{ 
					if((isspace(*buf)) || (!*buf)) 
					{ 
						*(char*)p=0; 
						break; 
					} 
					else 
					{ 
						*(char*)p=*buf; 
						p+=sizeof(char); 
						buf++; 
					} 
				} 
				
				count++; 
				break; 
			}	

			case 'c': 
			{
				*((char*)va_arg(vargs_ptr, char*))=*buf; 
				buf++; 
				count++; 
				break; 
			}

			case 'x': 
			case 'X': 
			{
				if(is_long) 
				{
					*((unsigned long*)va_arg(vargs_ptr, unsigned long*))=(unsigned long)my_atoul(buf, &end_ptr, 16); 
				}
				else 
				{
					*((unsigned int*)va_arg(vargs_ptr, unsigned int*))=(unsigned int)my_atoul(buf, &end_ptr, 16); 
				}
				
				if(buf==end_ptr) 
				{
					return(count); 
				}
				buf=end_ptr; 
				count++; 
				break; 
			}
*/
			default: 
			{
				return count; 
			}
    	} 
    	format++; 
  	} 

  return count; 
} 

/** Implementation of missing functionality in C18 library to convert a float to ASCII representation. */
void ftoa(float value, char *s, int places) 
{
	int digit;
	float tens=0.1;
	int tenscount=0;
	int i;
	float tempfloat=value;
	float d=0.5;
	
	// calculate rounding term d:   0.5/pow(10,places)
	if (value<0.0)
	{
		d*=-1.0;
	}	
	
	// divide by ten for each decimal place
	for (i = 0; i < places; i++)
	{
		d/=10.0;
	}	
	
	// this small addition, combined with truncation will round our values properly
	tempfloat+= d;
	
	// first get value tens to be the large power of ten less than value
	// tenscount isn't necessary but it would be useful if you wanted
	// to know after this how many chars the number will take
	
	if(value<0.0)
	{
		tempfloat*=-1.0;
	}	
	while((tens*10.0)<=tempfloat) 
	{
		tens*=10.0;
		tenscount++;
	}
	
	// write out the negative if needed
	if(value<0.0)
	{
		*s++='-';
	}	
	
	if(tenscount==0)
	{
		*s++='0';
	}	
	
	for(i=0; i<tenscount; i++) 
	{
		digit=(int)(tempfloat/tens);
		*s++=digit+'0';
		tempfloat=tempfloat-((float)digit * tens);
		tens/=10.0;
	}
	
	// if no places after decimal, stop now and return
	if(places<=0)
	{
		return;
	}	
	
	// otherwise, write the point and continue on
	*s++='.';
	
	// now write out each decimal place by shifting digits one by one
	// into the ones place and writing the truncated value
	for(i=0; i<places; i++) 
	{
		tempfloat*=10.0;
		digit=(int)tempfloat;
		*s++=digit+'0';
		
		// once written, subtract off that digit
		tempfloat=tempfloat-(float)digit;
	}
	*s='\0';
}
