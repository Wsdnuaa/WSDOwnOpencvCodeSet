#ifndef SGM_H
#define SGM_H
#include<iostream>
typedef int8_t			sint8;		// 有符号8位整数
typedef uint8_t			uint8;		// 无符号8位整数
typedef int16_t			sint16;		// 有符号16位整数
typedef uint16_t		uint16;		// 无符号16位整数
typedef int32_t			sint32;		// 有符号32位整数
typedef uint32_t		uint32;		// 无符号32位整数
typedef int64_t			sint64;		// 有符号64位整数
typedef uint64_t		uint64;		// 无符号64位整数
typedef float			float32;	// 单精度浮点
typedef double			float64;	// 双精度浮点
class SGM
{
private:
    /* data */
public:
    SGM(/* args */);
    ~SGM();

struct SGMOption {
	uint8	num_paths;		// 聚合路径数
	sint32  min_disparity;	// 最小视差
	sint32	max_disparity;	// 最大视差

	// P1,P2 
	// P2 = P2_int / (Ip-Iq)
	sint32  p1;				// 惩罚项参数P1
	sint32  p2_int;			// 惩罚项参数P2

	SGMOption(): num_paths(8), min_disparity(0), max_disparity(640), p1(10), p2_int(150) {
	}

};

};

SGM::SGM(/* args */)
{
}

SGM::~SGM()
{
}
#endif
