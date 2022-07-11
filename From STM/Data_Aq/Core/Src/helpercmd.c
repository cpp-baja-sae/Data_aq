/*
 * helpercmd.c
 *
 *  Created on: May 28, 2022
 *      Author: forre
 */
#include "stdio.h"
#include "helpercmd.h"

void HalfKBWrite(char* BufferToFill,uint16_t qty)
{
	for (int count=0; count<qty; count++)
	{
		for(int HexFiller = 0; HexFiller<512;HexFiller++)
		{
			if(HexFiller >=256)
			{
				BufferToFill[count*512+HexFiller] = HexFiller - 256;
			}else
			{
				BufferToFill[count*512+HexFiller] = HexFiller;
			}

		}

	}
}
