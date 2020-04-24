/**
 * @file utf.h ���������� ������ � utf8.
 */

#ifndef UTF8_H
#define	UTF8_H

#include <stddef.h>
#include <stdbool.h>
#include "../defs/defs.h"


/**
 * �������� ������ ������� ������ utf8.
 * @param str_c ������ utf8.
 * @return ������ ������� ������ utf8.
 */
EXTERN size_t utf8_str_char_size(const char* str_c);

/**
 * �������� ������ ������� utf8.
 * @param c ��� ������� utf8.
 * @return ������ ������� utf8.
 */
EXTERN size_t utf8_char_size(wchar_t c);

/**
 * ���������� ������ ������ utf8.
 * @param str_c ������ ������ utf8.
 * @return ��� ������� utf8.
 */
EXTERN wchar_t utf8_char_decode(const char* str_c);

/**
 * �������� ������ � ������ utf8.
 * @param str_c ������ utf8.
 * @param c ��� ������� utf8.
 * @return ������ ������� utf8.
 */
EXTERN size_t utf8_char_encode(char* str_c, wchar_t c);

/**
 * ������������ ������� ������ utf8.
 * @param str_c ������ utf8.
 * @return true ���� ������ �������, ����� false.
 */
EXTERN bool utf8_char_validate(const char* str_c);

#endif	/* UTF8_H */
