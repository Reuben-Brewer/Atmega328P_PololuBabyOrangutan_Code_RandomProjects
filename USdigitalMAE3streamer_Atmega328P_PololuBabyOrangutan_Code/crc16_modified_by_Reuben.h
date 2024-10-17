#ifndef _CRC16_H_
#define _CRC16_H_
#include <stdint.h>

/*
 * Get the CRC16 calculated value
 */
uint16_t crc16_read(uint16_t *caller_crc16_reg_ptr);

/*
 * Update the CRC16 for new byte value
 */
void crc16_update(uint16_t *caller_crc16_reg_ptr, char byte);

/*
 * Reset the CRC16 value
 */
void crc16_reset(uint16_t *caller_crc16_reg_ptr);

/*
 * Calculate CRC16 for buffer
 */
uint16_t crc16_calc(uint16_t *caller_crc16_reg_ptr, const char *input, int length);

#endif
