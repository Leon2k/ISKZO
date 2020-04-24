/**
 * @file net.h ������� �������������� ������� ����.
 */

#ifndef NET_H
#define NET_H

#include <stdint.h>
#include "defs/defs.h"

/**
 * ����������� ������� ������� ������������ ������
 * �������������� ��������� ������ �
 * ������� ������� ������������ ������.
 * @param hostshort ����� ��� ��������������.
 * @return ����� � ������� ������� ����.
 */
ALWAYS_INLINE static uint16_t htons(uint16_t hostshort)
{
    uint16_t res;
    
    __asm__ __volatile__ ("rev16 %0, %1" : "=r"(res) : "r"(hostshort));
    
    return res;
}

/**
 * ����������� ������� ������� ������������ ������
 * �������������� ��������� ������ �
 * ������� ������� ������������ ������.
 * @param netshort ����� ��� ��������������.
 * @return ����� � ������� ������� ����.
 */
ALWAYS_INLINE static uint16_t ntohs(uint16_t netshort)
{
    uint16_t res;
    
    __asm__ __volatile__ ("rev16 %0, %1" : "=r"(res) : "r"(netshort));
    
    return res;
}

/**
 * ����������� ������� ������� ������������ ������
 * �������������� ������ �
 * ������� ������� ������������ ������.
 * @param hostshort ����� ��� ��������������.
 * @return ����� � ������� ������� ����.
 */
ALWAYS_INLINE static uint32_t htonl(uint32_t hostlong)
{
    uint32_t res;
    
    __asm__ __volatile__ ("rev %0, %1" : "=r"(res) : "r"(hostlong));
    
    return res;
}

/**
 * ����������� ������� ������� ������������ ������
 * �������������� ������ �
 * ������� ������� ������������ ������.
 * @param netshort ����� ��� ��������������.
 * @return ����� � ������� ������� ����.
 */
ALWAYS_INLINE static uint32_t ntohl(uint32_t netlong)
{
    uint32_t res;
    
    __asm__ __volatile__ ("rev %0, %1" : "=r"(res) : "r"(netlong));
    
    return res;
}

#endif  //NET_H
