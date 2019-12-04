/* file type_def.h */

#ifndef TYPE_DEF_H_
#define TYPE_DEF_H_

/* bool and bool like types */
#ifndef bool
#define bool unsigned char
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef LOW
#define LOW 0
#endif

#ifndef HIGH
#define HIGH 1
#endif

#ifndef SUCCESS
#define SUCCESS 0
#endif
/* don't define FAIL or FAILURE, because any return value
        other than 0 is regarded as FAIL*/

/* data types */
/* fixed-width integer types*/
#ifndef uint8_t
typedef signed long int32_t;
typedef signed short int16_t;
typedef signed char int8_t;
typedef unsigned long uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
/* define min/max values */
#define INT8_MAX 0x7f
#define INT8_MIN (-INT8_MAX - 1)
#define UINT8_MAX 0xFF
#define UINT8_MIN 0
#define INT16_MAX 0x7fff
#define INT16_MIN (-INT16_MAX - 1)
#define UINT16_MAX 0xFFFF
#define UINT16_MIN 0
#define INT32_MAX 0x7fffffffL
#define INT32_MIN (-INT32_MAX - 1L)
#define UINT32_MAX 0xFFFFFFFF
#define UINT32_MIN 0
#endif

/* union for bit- or bytewise r/w-access to 8-bit data (byte_t) */
typedef union { /* for byte access */
  uint8_t byte;
  struct {          /* for bitwise access */
    uint8_t b0 : 1; /* bit 0 in byte */
    uint8_t b1 : 1; /* bit 1 in byte */
    uint8_t b2 : 1; /* bit 2 in byte */
    uint8_t b3 : 1; /* bit 3 in byte */
    uint8_t b4 : 1; /* bit 4 in byte */
    uint8_t b5 : 1; /* bit 5 in byte */
    uint8_t b6 : 1; /* bit 6 in byte */
    uint8_t b7 : 1; /* bit 7 in byte */
  } bit;
} byte_t;

/* struct for bytewise r/w access to 16bit data (word_t) */
typedef struct {
  uint8_t byteH; /* high byte in 16b word */
  uint8_t byteL; /* low byte in 16b word */
} word_t;

#endif /* TYPE_DEF_H_ */
