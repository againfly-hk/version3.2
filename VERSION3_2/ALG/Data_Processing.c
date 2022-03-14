#include "main.h"
#include "Data_Processing.h"

float match(uint32_t In_Max, uint32_t In_Min, uint32_t Out_Max, uint32_t Out_Min,uint32_t value)
{
	if(In_Max>=In_Min)
	{
		if(value>In_Max)
			value=In_Max;
		if(value<In_Min)
			value=In_Min;
		return ((1.0f*value*(Out_Max-Out_Min)/(In_Max-In_Max)));
	}
	else
	{
		if(value<In_Max)
			value=In_Max;
		if(value>In_Min)
			value=In_Min;
		return Out_Max-((1.0f*(value-In_Max)*(Out_Max-Out_Min)/(In_Min-In_Max)));		
	}
}
