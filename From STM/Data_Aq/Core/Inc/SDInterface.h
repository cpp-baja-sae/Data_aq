/*
 * SDInterface.h
 *
 *  Created on: May 29, 2022
 *      Author: forre
 */

#ifndef INC_SDINTERFACE_H_
#define INC_SDINTERFACE_H_

#include "fatfs.h"

// http://elm-chan.org/fsw/ff/doc/open.html
enum SD_FATFS_POSIX
{
	r		 = FA_READ,
	rPLUS	 = FA_READ | FA_WRITE,
	w		 = FA_CREATE_ALWAYS | FA_WRITE,
	wPLUS	 = FA_CREATE_ALWAYS | FA_WRITE | FA_READ,
	a		 = FA_OPEN_APPEND | FA_WRITE,
	aPLUS	 = FA_OPEN_APPEND | FA_WRITE | FA_READ,
	wx		 = FA_CREATE_NEW | FA_WRITE,
	wxPLUS	 = FA_CREATE_NEW | FA_WRITE | FA_READ
}SD_FATFS_POSIX;

void MountSD();
void OpenSD(const char **file_name,enum SD_FATFS_POSIX file_acces_type);
void WriteSD(char *wtext, uint32_t wtextSize, uint32_t *BytesWritten);
void CloseSD();
#endif /* INC_SDINTERFACE_H_ */
