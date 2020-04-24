/**
 * @file bits.h
 * ������� ��� ������ � ������.
 */

#ifndef BITS_H
#define	BITS_H

#include "../defs/defs.h"

#ifndef BIT_SET_TEST_0BIT
#define BIT_SET_TEST_0BIT 1
#endif

/**
 * ��������� ����� � ������������� �����
 * � �������� ������� B.
 */
#define BIT(B) (1 << (B))
/**
 * ��������� ����� � �����, ������ V
 * � �������� ������� B.
 */
#if BIT_SET_TEST_0BIT == 1
#define BIT_BYVAL(B, V) ((V & 0x1) << B)
#else
#define BIT_BYVAL(B, V) ((V      ) << B)
#endif
/**
 * ������������� ��� ����� B � ���������� P.
 */
#define BIT_ON(P, B) do{(P) |= BIT(B);}while(0)
/**
 * ���������� ��� ����� B � ���������� P.
 */
#define BIT_OFF(P, B) do{(P) &= ~BIT(B);}while(0)
/**
 * �������� ��� ����� B � ���������� P.
 */
#define BIT_TOGGLE(P, B) do{(P) ^= BIT(B);}while(0)
/**
 * ������������� ��� ����� B � ���������� P � �������� V.
 */
#if BIT_SET_TEST_0BIT == 1
    #define BIT_SET(P, B, V) do{ if( (V) & 0x1 ){ BIT_ON(P, B); }else{ BIT_OFF(P, B); } }while(0)
#else
    #define BIT_SET(P, B, V) do{ if( (V)       ){ BIT_ON(P, B); }else{ BIT_OFF(P, B); } }while(0)
#endif
/**
 * ����������� ������� ��� � ����� P.
 */
#define BIT0_NOT(P) ((P) ^ 1)
/**
 * ��������� ��� ����� B � ���������� P.
 */
#define BIT_TEST(P, B) ((P) & BIT(B))
/**
 * ���������� ����� �������� ���� B � ���������� P.
 */
#define BIT_RAW_VALUE(P, B) ((P) & BIT(B))
/**
 * ��� ��������� ���� B � ���������� P.
 */
#define BIT_WAIT_ON(P, B) do{}while(!BIT_TEST(P, B))
/**
 * ��� ������ ���� B � ���������� P.
 */
#define BIT_WAIT_OFF(P, B) do{}while(BIT_TEST(P, B))
/**
 * ���������� �������� ���� B � ���������� P.
 */
#define BIT_VALUE(P, B) ((BIT_TEST(P, B)) ? 0x1 : 0x0)
/**
 * ������������� ����, �������������� �����.
 */
#define BIT_ON_MASK(P, M) do{(P) |= (M);}while(0)
/**
 * ���������� ����, ��������������� �����.
 */
#define BIT_OFF_MASK(P, M) do{(P) &= ~(M);}while(0)
/**
 * �������� ����, ��������������� �����.
 */
#define BIT_TOGGLE_MASK(P, M) do{(P) ^= (M);}while(0)
/**
 * ������������� ����, �������������� ����� M, � ���������� P � �������� V.
 */
#if BIT_SET_TEST_0BIT == 1
    #define BIT_SET_MASK(P, M, V) do{ if( (V) & 0x1 ){ BIT_ON_MASK(P, M); }else{ BIT_OFF_MASK(P, M); } }while(0)
#else
    #define BIT_SET_MASK(P, M, V) do{ if( (V)       ){ BIT_ON_MASK(P, M); }else{ BIT_OFF_MASK(P, M); } }while(0)
#endif
/**
 * ��������� ����, ��������������� �����.
 */
#define BIT_TEST_MASK(P, M) ((P) & (M))
/**
 * ���������� ������� ������ ������ �������������� ���� �� ����� M � ���������� P.
 */
#define BIT_TEST_VALUE_MASK(P, M) ((BIT_TEST_MASK(P, M)) ? 0x1 : 0x0)
/**
 * ��������� ����� ������ N ��� � ��������� S ���.
 */
#define BIT_MAKE_MASK(N, S) ((BIT(N) - 1) << (S))
/**
 * �������� �������� ��� ��������� O � ������ S � ���������� P.
 */
#define BIT_VALUE_LEN(P, O, S) (((P) >> (O)) & (BIT(S) - 1))
/**
 * �������� �������� ��� ��������� O � ������ M � ���������� P.
 */
#define BIT_VALUE_MASK(P, O, M) (((P) >> (O)) & (M))

/**
 * ������������� ��� � �������.
 * @param bits ������.
 * @param bit_n ����� ����.
 */
ALWAYS_INLINE static void bits_on(uint8_t* bits, uint8_t bit_n)
{
    BIT_ON(bits[bit_n >> 3], bit_n & 0x7);
}

/**
 * ���������� ��� � �������.
 * @param bits ������.
 * @param bit_n ����� ����.
 */
ALWAYS_INLINE static void bits_off(uint8_t* bits, uint8_t bit_n)
{
    BIT_OFF(bits[bit_n >> 3], bit_n & 0x7);
}

/**
 * ���������� ��� � �������.
 * @param bits ������.
 * @param bit_n ����� ����.
 * @return ���.
 */
ALWAYS_INLINE static uint8_t bits_value(uint8_t* bits, uint8_t bit_n)
{
    return BIT_VALUE(bits[bit_n >> 3], bit_n & 0x7);
}

/**
 * ������������� �������� ���� � �������.
 * @param bits ������.
 * @param bit_n ����� ����.
 * @param bit �������� ����.
 */
static ALWAYS_INLINE void bits_set_value(uint8_t* bits, uint8_t bit_n, uint8_t bit)
{
    uint8_t byte_n = bit_n >> 3;
    uint8_t byte_bit_n = bit_n & 0x7;
    
    BIT_SET(bits[byte_n], byte_bit_n, bit);
}

#endif	/* BITS_H */

