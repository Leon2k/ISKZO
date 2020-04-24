/**
 * @file one_wire.h
 * ���������� ��� ������ � ����� 1-wire.
 */

#ifndef ONE_WIRE_H
#define	ONE_WIRE_H

#include "stm32f1xx.h"
#include "../errors/errors.h"
#include "../defs/defs.h"
#include <stdint.h>
#include <stddef.h>

#include "stm32f1xx_hal_gpio.h"

//���� ������.
#define E_ONE_WIRE                      (E_USER + 10)
#define E_ONE_WIRE_INVALID_CRC          (E_ONE_WIRE + 1)
#define E_ONE_WIRE_SEARCH_LOGIC_ERROR   (E_ONE_WIRE + 2)
#define E_ONE_WIRE_DEVICES_NOT_FOUND    (E_ONE_WIRE + 3)


//���� ������
//����� ���������.
#define ONE_WIRE_CMD_SEARCH_ROM         0xf0
//������ �������������� ROM.
#define ONE_WIRE_CMD_READ_ROM           0x33
//����� ����������.
#define ONE_WIRE_CMD_MATCH_ROM          0x55
//������������� ���������������.
#define ONE_WIRE_CMD_SKIP_ROM           0xcc


/**
 * ��������� ���� 1-wire.
 */
typedef struct _One_Wire{
    //���� ����.
    GPIO_TypeDef* GPIO;
    //��� ����� ����.
    uint16_t GPIO_Pin;
}one_wire_t;

/**
 * ��������� �������������� ROM ���������� �� ���� 1-wire.
 */
#define ONE_WIRE_SERIAL_LEN 6
#pragma pack(push, 1)
typedef struct _Rom_Id{
    //��� ���� ����������.
    uint8_t family_code;
    //48-������ �������� �����.
    uint8_t serial[ONE_WIRE_SERIAL_LEN];
    //����������� �����.
    uint8_t crc;
}one_wire_rom_id_t;
#pragma pack(pop)

/**
 * �������������� ��������� ���� 1-wire.
 * @param ow ��������� ���� 1-wire.
 * @param port_n ����� �����.
 * @param pin_n ����� ���� �����.
 * @return ��� ������.
 */
EXTERN err_t one_wire_init(one_wire_t* ow, GPIO_TypeDef* GPIO, uint16_t GPIO_Pin);

/**
 * ���������� ���������� �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @return 1 � ������ ������� ��������� �� ����, ����� 0.
 */
EXTERN uint8_t one_wire_reset(one_wire_t* ow);

/**
 * ���������� ��� � ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param bit ���, ����� �������� ������ ������� ���.
 */
EXTERN void one_wire_write_bit(one_wire_t* ow, uint8_t bit);

/**
 * ��������� ��� �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @return ���, � ������� ����.
 */
EXTERN uint8_t one_wire_read_bit(one_wire_t* ow);

/**
 * ���������� ���� � ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param byte ����.
 */
EXTERN void one_wire_write_byte(one_wire_t* ow, uint8_t byte);

/**
 * ��������� ���� �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @return ����.
 */
EXTERN uint8_t one_wire_read_byte(one_wire_t* ow);

/**
 * ���������� ������ � ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param data ������.
 * @param size ������ ������.
 */
EXTERN void one_wire_write(one_wire_t* ow, const void* data, size_t size);

/**
 * ��������� ������ �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param data ������.
 * @param size ������ ������.
 */
EXTERN void one_wire_read(one_wire_t* ow, void* data, size_t size);

/**
 * ��������� ����������� ����� ������.
 * @param data ������.
 * @param size ������ ������.
 * @return ����������� �����.
 */
EXTERN uint8_t one_wire_calc_crc(const void* data, size_t size);

/**
 * ����� ��� one_wire_write_byte().
 */
#define one_wire_send_cmd(ow, cmd) one_wire_write_byte(ow, cmd)

/**
 * ������ ������������� ���������� �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param rom ������������� ����������.
 * @return ��� ������.
 */
EXTERN err_t one_wire_read_rom(one_wire_t* ow, one_wire_rom_id_t* rom);

/**
 * �������� ���������� � �������� ��������������� �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param rom ������������� ����������.
 * @return ��� ������.
 */
EXTERN err_t one_wire_match_rom(one_wire_t* ow, one_wire_rom_id_t* rom);

/**
 * ���������� ���������� �� ���� 1-wire ������������ ���������.
 * @param ow ���� 1-wire.
 * @return ��� ������.
 */
EXTERN err_t one_wire_skip_rom(one_wire_t* ow);

/**
 * ���� ���������� �� ���� 1-wire.
 * @param ow ���� 1-wire.
 * @param roms �������������� ���������.
 * @param roms_count ����� ���������������.
 * @param roms_found ����� ��������� ���������.
 * @param max_attempts ����� �������� ��� ������ ��� ������� ����������.
 * @return ��� ������.
 */
EXTERN err_t one_wire_search_roms(one_wire_t* ow, one_wire_rom_id_t* roms,
                                  uint8_t roms_count, uint8_t* roms_found,
                                  uint8_t max_attempts);

#endif	/* ONE_WIRE_H */
