/**
 * @file user_math.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER__MATH_H
#define __USER__MATH_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
/* Private includes ----------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
/* Exported constants --------------------------------------------------------*/
/***********************角度***************************************************/
#define ushort uint16_t //强制转换无符号整型
#define _short int16_t  //强制转换有符号整型
#define PI acos(-1)

/* Exported types ------------------------------------------------------------*/

/***********位置式PID***************************************/
//#define PID_INTEGRAL_ON    //位置式PID是否包含积分项。如果仅用PD控制，注释本行
typedef struct
{ 
    float P;               
    float I;
    float D;	
#ifdef 	PID_INTEGRAL_ON
    float Integral;        //位置式PID积分项
    float IntegralMax;     //位置式PID积分项最大值，用于限幅
#endif	
    float Last_Error;      //上一次误差	
    float OutputMax;       //位置式PID输出最大值，用于限幅
}position_PID;
/***********增量式PID***************************************/
typedef struct
{
    float P;              //Kp系数
    float I;              //Ki系数
    float D;              //Kd系数
    float OutputMax;      //输出最大值，用于限幅	
	
    int32_t LastError;     //前一次误差    
    int32_t PrevError;     //前两次误差
}increment_PID;
/***********一阶滞后滤波**************************************/
typedef struct
{
  float K;//系数【越小越平滑，响应时间越慢】
  float Ai;//这次值
  float Bi;//上一次值
  float result;//输出
}FOL_TypeDef;
/* Exported macro ------------------------------------------------------------*/
//https://graphics.stanford.edu/~seander/bithacks.html#SwappingValuesSubAdd
#define SWAP(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))//交换两个值
/*https://blog.csdn.net/pl0020/article/details/104813884*/
#define	USER_CLEAR_BIT(x, bit)	(x  &= ~(1 << bit))	        /* 清零第bit位 */
#define	USER_GET_BIT(x, bit)	  ((x &   (1 << bit)) >> bit)	/* 获取第bit位 */
#define	USER_SET_BIT(x, bit)	  (x  |=  (1 << bit))	        /* 置位第bit位 */
/*https://www.cnblogs.com/lvdongjie/p/4488011.html*/

/*https://www.cnblogs.com/life2refuel/p/8283435.html*/
extern bool compare_and_swap (int* reg, int oldval,int newval);
#define ONCE_RUN(code) {                                    \
    static int _done;                                       \
    if (!_done) {                                           \
        if (compare_and_swap(&_done, 0, 1)) {   \
            code                                            \
        }                                                   \
    }                                                       \
}

//
// EXTERN_RUN - 简单的声明, 并立即使用的宏
// ftest    : 需要执行的函数名称
// ...      : 可变参数, 保留
//
#define EXTERN_RUN(ftest, ...)  \
do {                            \
    extern void ftest();        \
    ftest (__VA_ARGS__);        \
} while(0)

/*四舍五入宏定义
该宏定义来自linux kernel 3.10。

https://blog.csdn.net/u012028275/article/details/118864192

x表示被除数，被除数可以是正数或者负数。

divisor表示除数，除数只能是正数。

注意：如果被除数是负数，那么除数divisor的变量类型不能是无符号的，否者结果是未定义的。
*/
#define DIV_ROUND_CLOSEST(x, divisor)(			\
{							\
	typeof(x) __x = x;				\
	typeof(divisor) __d = divisor;			\
	(((typeof(x))-1) > 0 ||				\
	 ((typeof(divisor))-1) > 0 || (__x) > 0) ?	\
		(((__x) + ((__d) / 2)) / (__d)) :	\
		(((__x) - ((__d) / 2)) / (__d));	\
}							\
)

#define GPIO_TURN(x) (((x) == (GPIO_PIN_RESET))?(GPIO_PIN_SET):(GPIO_PIN_RESET))//电平翻转
/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
extern float PID_Cal(position_PID *pid, float NowValue, float AimValue);
extern float IncPIDCal(increment_PID *pid, float NowValue, float AimValue);

/*滤波函数*/
extern float First_Order_Lag(FOL_TypeDef *p,float intput);

/*其他函数*/
//https://graphics.stanford.edu/~seander/bithacks.html#SwappingValuesSubAdd
extern bool Detect_Opposite_INT(int x, int y);
extern int Compute_Sign_INT(int v,uint8_t CHAR_BIT);

extern long long qpow(int16_t a, int16_t n);
//https://blog.csdn.net/qq_43537721/article/details/107757766
extern float Hex_To_Decimal(unsigned char *Byte);
extern long FloatTohex(float HEX);
extern void FloatToByte(float floatNum,unsigned char* byteArry);
//https://blog.csdn.net/qq_43630810/article/details/108819378
extern unsigned char Reverse_byte(unsigned char wdata);
//https://blog.csdn.net/mayh554024289/article/details/47914237
extern void LeftShift(int* arr,int N, int K);
extern void RightShift(int* arr,int N, int K);

extern float Angle_Conversion(float absolute_angle);

extern uint16_t Binary_Search(uint32_t input,uint32_t *Array,uint16_t size);
//https://blog.csdn.net/qq_40757240/article/details/85727578
extern int Bit_Int_2(long n);

#ifdef __cplusplus
}
#endif

#endif /* __USER__MATH_H */