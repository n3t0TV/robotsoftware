#pragma once
/*
 * Classes, structures and functions for data management
 */

typedef union
{
    unsigned int as_uint;
    int as_int;
    float as_float;
    unsigned short as_ushort[2];
    short as_short[2];
    //~ short _Accum as_accum[2];
    unsigned char as_uchar[4];
    char as_char[4];
} ReinterpretedData;
