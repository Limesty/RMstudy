#ifndef _SERIAL_H_
#define _SERIAL_H_

typedef unsigned int uint32_t;
typedef float fp32;
typedef struct toSTM32_t {
    uint32_t start;	// 开始标志符号，'S'
    fp32 pitch;	// 目标delta_pitch角度
    fp32 yaw;		// 目标delta_yaw角度
    fp32 x;		// 云台坐标系下的x速度
    fp32 y;		// 云台坐标系下的y速度
    fp32 w;		// 云台坐标系下的角速度w
    uint32_t shoot;	// 确认发射
    uint32_t found_armor;	// 找到了装甲板
    uint32_t reserved[7];	// 保留字段
    uint32_t end;	// 结尾标志符号，'E'
} toSTM32_t;

#define Linux

#ifdef Windows

#include <Windows.h>

class Serial
{
public:
	Serial(UINT  baud = CBR_115200, char  parity = 'N', UINT  databits = 8, UINT  stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR);
	~Serial();

	bool InitPort(UINT  portNo = 1, UINT  baud = CBR_9600, char  parity = 'N', UINT  databits = 8, UINT  stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR);
	UINT GetBytesInCOM() const ;
	bool WriteData(const unsigned char* pData, unsigned int length);
	bool ReadData(unsigned char* buffer, unsigned int length);
private:
	bool openPort(UINT  portNo);
	void ClosePort();
	void ErrorHandler();
private:
	HANDLE hComm;
	UINT portNo;
	UINT baud;
	char parity;
	UINT databits;
	UINT stopsbits;
	DWORD dwCommEvents;
};

#elif defined(Linux) || defined(Darwin)

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class Serial {
private:
    int fd;
    int nSpeed;
    char nEvent;
    int nBits;
    int nStop;

    int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);

public:
    bool OpenSerial(int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1);
    ~Serial();

    bool InitPort(int nSpeed = 115200, char  nEvent = 'N', int nBits = 8, int nStop = 1);
//    int GetBytesInCOM() const ;
    bool WriteData(const unsigned char* pData, unsigned int length);
    bool ReadData(unsigned char* buffer, unsigned int length);

	bool send_control(
		fp32 pitch,	// pitch轴目标偏移角度，弧度制，向左偏为正
		fp32 yaw,	// yaw轴目标偏移角度，弧度制，向上偏为正
		fp32 x,		// x方向速度（前进），单位m/s
		fp32 y,		// y方向速度（左移），单位m/s
		fp32 w,		// 小陀螺自转角速度 ，单位rad/s
		uint32_t shoot,	// 为1时表示立刻发射，否则设置为0
		uint32_t found_armor	// 为1时表示在当前画面中找到了需要瞄准的装甲板，否则设置为0
	);
};



#endif

#endif /* _SERIAL_H_ */
