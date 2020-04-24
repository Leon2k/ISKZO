/**
 * @file errors.h
 * ��� � ����� ���� ������.
 */

#ifndef ERRORS_H
#define	ERRORS_H

#include <stdint.h>

//! ��� ������.
typedef uint8_t err_t;

//! ���������� ������.
#define E_NO_ERROR 0
//! ������� ���������.
#define E_NULL_POINTER 1
//! ������������ ��������.
#define E_INVALID_VALUE 2
//! ����� �� ���������� �������.
#define E_OUT_OF_RANGE 3
//! ������.
#define E_BUSY 4
//! ������ �����-������.
#define E_IO_ERROR 5
//! �������� ������.
#define E_OUT_OF_MEMORY 6
//! ��������������� ����������.
#define E_NOT_IMPLEMENTED 7
//! ������ ����������� �����.
#define E_CRC 8
//! ������ ���������.
#define E_STATE 9
//! ������ ����-����.
#define E_TIME_OUT 10

//! ��������� ��� ��� ���������������� ������.
#define E_USER 100

#endif	/* ERRORS_H */

