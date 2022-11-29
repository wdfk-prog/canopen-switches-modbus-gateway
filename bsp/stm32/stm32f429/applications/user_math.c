/**
  ******************************************************************************
  * @file    user_math.c
  * @brief   用户自定义函数
  ******************************************************************************
  * @attention  
  * https://blog.csdn.net/qq_39016531/article/details/107411030#comments_19409528
  * https://blog.csdn.net/qlexcel/article/details/103651072
  * @author 进击的蜗牛_QJ、丘木木
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "user_math.h"
/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
bool compare_and_swap (int* reg, int oldval,int newval)
{
  int old_reg_val = *reg;
  if(old_reg_val == oldval)
  {
    *reg = newval;
    return true;
  }
  return false;
}
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**********************************************************************************************************
*	函 数 名：PID_Cal
*	功能说明：位置式PID控制
*   输    入：
    NowValue:当前值
    AimValue:目标值
*   输    出：PID控制值，直接赋值给执行函数
https://blog.csdn.net/qlexcel/article/details/103651072
**********************************************************************************************************/

float PID_Cal(position_PID *pid, float NowValue, float AimValue)
{
    float  iError,     //当前误差
            Output;    //控制输出	
    iError = AimValue - NowValue;                   //计算当前误差
	
#ifdef 	PID_INTEGRAL_ON	
    pid->Integral += pid->I * iError;	            //位置式PID积分项累加
    pid->Integral = pid->Integral > pid->IntegralMax?pid->IntegralMax:pid->Integral;  //积分项上限幅
    pid->Integral = pid->Integral <-pid->IntegralMax?-pid->IntegralMax:pid->Integral; //积分项下限幅
#endif		
	
    Output = pid->P * iError                        //比例P            
           + pid->D * (iError - pid->Last_Error);   //微分D
	
#ifdef 	PID_INTEGRAL_ON		
    Output += pid->Integral;                        //积分I
#endif	
 
    Output = Output > pid->OutputMax?pid->OutputMax:Output;  //控制输出上限幅
    Output = Output <-pid->OutputMax?-pid->OutputMax:Output; //控制输出下限幅
	
	pid->Last_Error = iError;		  	                     //更新上次误差，用于下次计算 
	return Output;	//返回控制输出值
}
/**********************************************************************************************************
*	函 数 名：IncPIDCal
*	功能说明：增量式PID计算
*	形    参：
*	返 回 值：
**********************************************************************************************************/ 
float IncPIDCal(increment_PID *pid, float NowValue, float AimValue)     
{
	float iError;                          //当前误差值
	float Output;                           //控制输出增量值
	
	iError = AimValue - NowValue;            //目标值与当前值之差
 
	Output = (pid->P * iError)               //E[k]项
			-(pid->I * pid->LastError)       //E[k-1]项
			+(pid->D * pid->PrevError);      //E[k-2]项
 
	pid->PrevError = pid->LastError;         //存储误差，用于下次计算
	pid->LastError = iError;
	
    Output = Output > pid->OutputMax?pid->OutputMax:Output;  //控制输出上限幅
    Output = Output <-pid->OutputMax?-pid->OutputMax:Output; //控制输出下限幅
	
	return(Output);                         //返回增量值
}
/**
  * @brief  一阶滞后滤波函数
  * @param  FOL_TypeDef 滞后滤波参数
  * @param  intput 输入值
  * @retval None.
  * @note   K系数需要调整，输入输出需要给予
*/
float First_Order_Lag(FOL_TypeDef *p,float intput)
{
  p->Ai = intput;
  p->result = p->Ai * p->K + p->Bi * (1 - p->K);
  p->Bi = p->result;
  return p->result;
}
/**
  * @brief  检测两个整数的符号是否相反
  * @param  None.
  * @retval 当两个数相反时返回1,否则返回0 。其中有一个数为0则返回1
  * @note   Detect if two integers have opposite signs.
Manfred Weis suggested I add this entry on November 26, 2009.
https://graphics.stanford.edu/~seander/bithacks.html#SwappingValuesSubAdd
*/
bool Detect_Opposite_INT(int x, int y)// input values to compare signs
{
  return ((x ^ y) < 0); // true iff x and y have opposite signs
}
/**
  * @brief  计算整数的符号
  * @param  需要计算符号的数，该数的位数
  * @retval 结果是 -1、0 或 +1，则使用
  * @note   None.
*/
int Compute_Sign_INT(int v,uint8_t CHAR_BIT)// we want to find the sign of v
{
  int sign;   // the result goes here 
  sign = (v != 0) | -(int)((unsigned int)((int)v) >> (sizeof(int) * CHAR_BIT - 1));
  // Or, for more speed but less portability:
  sign = (v != 0) | (v >> (sizeof(int) * CHAR_BIT - 1));  // -1, 0, or +1
  // Or, for portability, brevity, and (perhaps) speed:
  sign = (v > 0) - (v < 0); // -1, 0, or +1
  return sign;
}
//非递归快速幂
long long qpow(int16_t a, int16_t n)
{
    int ans = 1;
    while(n)
    {
      if(n&1)        //如果n的当前末位为1
        ans *= a;    //ans乘上当前的a
      
      a *= a;        //a自乘
      n >>= 1;       //n往右移一位
    }
    return ans;
}
/**
  * @brief  角度换算
  * @param  None.
  * @retval None.
  * @note   角度换算 范围 -180~180
*/
float Angle_Conversion(float absolute_angle)
{
  absolute_angle = fmod(absolute_angle,360);
  if(absolute_angle == 180)
    absolute_angle = -180;
  else if(absolute_angle > 180)
    absolute_angle =  absolute_angle- 360;
  else if(absolute_angle < -180)
    absolute_angle = absolute_angle + 360;
  return absolute_angle;
}
/**
  * @brief  二分法查表
  * @param  input:需要查找数值
  * @param  Array:需要查找数组
  * @param  size:数组大小
  * @retval None.
  * @note   http://www.wawooo.com/191.html
*/
uint16_t Binary_Search(uint32_t input,uint32_t *Array,uint16_t size)//数组需要从小到大排序
{
	
  int start=0; //数组头（标号）
  int end = size; //数组尾（标号）
  int mid = 0;  //数组中间（标号）
  while(start<=end)
  {
    mid=(start+end)/2;//折半 取中间（标号）
    if(input==Array[mid]) break;  //当前AD值与数组中间值相等 说明查到
    if((input<Array[mid])&&(input>Array[mid+1]))  break; //在两者之间
    if(input>Array[mid])  //当前AD值大于数组中间数值 说明要查找的数在后一半
      start = mid+1;   //前一半的数组尾不变 数组头重新分配
    else if(input<Array[mid])  //当前AD值小于数组中间数值 说明要查找的数在前一半
      end = mid-1;  //前一半的数组头不变 数组尾重新分配
  }
  return mid;//返回查到的数组标号
}
/**
  * @brief  判断整数的位数
  * @param  long n:数字
  * @retval 位数.
  * @note   https://blog.csdn.net/qq_40757240/article/details/85727578
*/
int Bit_Int_2(long n)
{
	if(n==0)
		return 1;
	else
		return log10(abs(n))+1;//求整数的位数
	//例如  1234  lgx+1;                   
}
//https://blog.csdn.net/qq_43537721/article/details/107757766
/**
  * @brief  十六进制到浮点数
  * @param  None.
  * @retval None.
  * @note   None.
*/
float Hex_To_Decimal(unsigned char *Byte)//,int num
{
//  	char cByte[4];//方法一
//     for (int i=0;i<num;i++)
//     {
// 	cByte[i] = Byte[i];
//     }
//      
// 	float pfValue=*(float*)&cByte;
//return  pfValue;
		return *((float*)Byte);//方法二
}
/**
  * @brief  //浮点数到十六进制转换
  * @param  None.
  * @retval None.
  * @note   None.
*/
long FloatTohex(float HEX)
{
	return *( long *)&HEX;
}
/**
  * @brief  浮点数到十六进制转换2进制
  * @param  None.
  * @retval None.
  * @note   None.
*/
void FloatToByte(float floatNum,unsigned char* byteArry)
{
    int i = 0;
    char* pchar=(char*)&floatNum;
    for( i=0;i < sizeof(float);i++)
    {
      *byteArry=*pchar;
      pchar++;
      byteArry++;
    }
}
/**
  * @brief  高低位交换.
  * @param  None.
  * @retval None.
  * @note   蝶式交换法.
https://blog.csdn.net/qq_43630810/article/details/108819378
*/
unsigned char Reverse_byte(unsigned char wdata)
{
	wdata=(wdata<<4)|(wdata>>4);
	wdata=((wdata<<2)&0xcc)|((wdata>>2)&0x33);
	wdata=((wdata<<1)&0xaa)|((wdata>>1)&0x55);
	return wdata;
}
/**
  * @brief  整型转字符串，支持任意进制.
  * @param  None.
  * @retval None.
  * @note   原实现仅支持十进制，并且在输入末端为0，会截断输出。
例如10 - > 1
https://www.edaboard.com/threads/itoa-for-8051-based-micros.133344/.
*/
void itoa(int n, char *s,int radix) 
{ 
  char i;
  int n1;
  if (n<0) 
  {
    n=-n;
    *s++='-';
  }
  do
  {
    n1=n;
    i=0;
    while (1) 
    {
      if (n1<=(radix-1))   
      {
        *s++=n1+'0';
        break;
      }
      n1=n1/radix;
      i++;
    }
    while (i) 
    {
      i--;
      n1=n1*radix;
    }
    n-=n1;
  }while (n);
  if(!(n1 % radix))
     *s++='0';
  *s++=0;
}
/**
  * @brief  逆置
  * @param  None.
  * @retval None.
  * @note   None.
*/
void Reverse(int *arr,int start,int end)
{      //逆置
  for(; start < end;start++,end--)
  {
     int s = arr[end];
     arr[end] = arr[start];
     arr[start] = s;
   }
}
/**
  * @brief  数组循环向左移动k位的算法.
  * @param  None.
  * @retval None.
  * @note  
 1 2 3 4 5 6 7 需要左移4次，那么我们想要的结果是： 5 6 7 1 2 3 4。
1.将1234逆置 变成 4321
2.将567逆置 变成 765
3.将两个逆置数组拼接： 4321765
4.将这个已拼接的数组逆置： 5671234 就成了我们想要的结果了。
*/
void LeftShift(int* arr,int N, int K)
{
      K = K%N;                      //对应上文步骤
      Reverse(arr,0,K-1);           //1 
      Reverse(arr,K,N-1);           //2
      Reverse(arr,0,N-1);           //4
}
/**
  * @brief  数组循环向右移动k位的算法.
  * @param  arr:数组.
  * @param  N:数组长度
  * @param  K:移位长度
* @retval None.
  * @note   None.
 1 2 3 4 5 6 7 需要右移4次，那么我们想要的结果是：4 5 6 7 1 2 3。
1.将1234逆置 变成 4321
2.将567逆置 变成 765
3.将两个逆置数组拼接： 4321765
4.将这个已拼接的数组逆置： 5671234 就成了我们想要的结果了。
*/
void RightShift(int* arr,int N, int K)
{
    K = K%N;                      //对应上文步骤
    Reverse(arr,  0 , N - K - 1 );
    Reverse(arr, N - K, N - 1 );
    Reverse(arr,  0 , N - 1 );
}