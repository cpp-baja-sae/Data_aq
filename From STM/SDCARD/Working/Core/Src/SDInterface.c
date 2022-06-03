/*
 * SDInterface.c
 *
 *  Created on: May 29, 2022
 *      Author: forre
 */
#include "fatfs.h"
#include "SDInterface.h"
#include "stdio.h"
#include "ADS8588H.h"
#include "string.h"

void MountSD()
{
	// Temporary memory allocation to setup file system
	// http://elm-chan.org/fsw/ff/doc/mkfs.html
	uint8_t rtext[_MAX_SS*64];
	FRESULT res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
    if( res != FR_OK)
    {
        Error_Handler();
    }
    else
    {
    	FRESULT res = f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext));
        if( res != FR_OK)
        {
            Error_Handler();
        }
    }
};

void UnMountSD()
{
	f_mount(0, (TCHAR const*)NULL, 0);
}

void OpenSD(const char **file_name,enum SD_FATFS_POSIX file_acces_type)
{
	FRESULT res = f_open(&SDFile, *file_name, file_acces_type);
	if(res != FR_OK)
	{
		Error_Handler();
	}
};

void WriteSD(uint16_t *wtext, uint32_t wtextSize, uint32_t *BytesWritten)
{
	FRESULT res = f_write(&SDFile, (uint16_t *)wtext, wtextSize, (void *)&BytesWritten);
	if((BytesWritten == 0) || (res != FR_OK))
	{
	  Error_Handler();
	}
};

void CloseSD()
{
	f_close(&SDFile);
};

