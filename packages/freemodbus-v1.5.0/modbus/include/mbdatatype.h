#ifndef __DATATYPE__H
#define __DATATYPE__H

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef unsigned char BOOL;
#define TRUE (1)
#define FALSE (0)

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#define _CONST  const
#define _REENTRANT reentrant



#endif
