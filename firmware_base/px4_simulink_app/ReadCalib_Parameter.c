/* Updates CSC parameter value
 * Refer to:
 * https://github.com/PX4/Firmware/blob/cddfcb35d8132be466188c4bd695980d3a760d0b/src/systemcmds/param/param.c
 **/

#include "ReadCalib_Parameter.h"
//#include "nuttxinitialize.h"

void InitParamFunction(char *ParameterStr, void* ReturnVal) 
{
int32_t  ReturnTmpInt;
float ReturnTmpFlt;
param_t Parameter_H = param_find(ParameterStr); //parameter handle
   
switch (param_type(Parameter_H)) {
    
	case PARAM_TYPE_INT32:
		if (!param_get(Parameter_H, &ReturnTmpInt)) {
			*(int*)ReturnVal  = ReturnTmpInt;            
//             printf("%ld\n", *(int*)ReturnVal);
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(Parameter_H, &ReturnTmpFlt)) {
            *(float*)ReturnVal  = ReturnTmpFlt;   
// 			printf("%4.4f\n", *(float*)ReturnVal);
		}

		break;
	default:
		printf("<unknown type %d>\n");
	}

}


