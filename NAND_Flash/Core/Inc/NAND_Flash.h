/*
 * NAND_Flash.h
 *
 *  Created on: Mar 22, 2022
 *      Author: forre
 */

#ifndef INC_NAND_FLASH_H_
#define INC_NAND_FLASH_H_

#define RESET 0xFF
#define READ_ID 0x9F
#define SET_FEATURES 0x1F
#define GET_FEATURE	0x0F
#define OIP_BIT 0xC0
#define SET_FEATURE_ADDR 0xB0
#define ID_LOC 0x50
#define PAGE_READ 0x13
#define UID1 0x01
#define UID2 0x80
#define READ_BUF 0x03

#endif /* INC_NAND_FLASH_H_ */
