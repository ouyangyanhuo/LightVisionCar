#ifndef _TCPServer_H_
#define _TCPServer_H_



#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)
typedef struct {
	unsigned int IntCnt;
	unsigned int ConCnt;

} timeoutcnt;
extern timeoutcnt TimeOutCnt;

void WIFIConnect(void);
void TrySANDRData(void);
void DataHandleInt(void);
void DataHandle(void);
#endif