/* Copyright 2005-2007 The MathWorks, Inc. */

/* $Revision: 1.1.6.1 $ */

//http://www.dreamincode.net/forums/topic/177466-serial-port-programming-why-fcntlfd-f-setfl-0%3B/
// http://en.wikibooks.org/wiki/Serial_Programming/termios

int initSerial(const unsigned char* DeviceName, int BaudSetting, int BlockMode);
void termSerial(int fh);
int flushSerial(int fh_index);
int MW_openSerialPort(unsigned char * DeviceName, unsigned int BaudRate, int BlockingMode, int * ArrayElementReturn);
int MW_closeSerialPort(unsigned char * DeviceName, int DeviceElement);
void MW_writeSerial(int fh_index,unsigned char * data, int ByteCount);
void MW_writeSerial_Header(int fh_index, unsigned char * data, int ByteCount, const unsigned char * dataHeader, int ByteCountHeader);
void MW_readSerial(int fh_index, unsigned char * data, int ByteCount,int *Status);
void MW_readSerial_TermCheck(int fh_index, unsigned char * data, unsigned char chTerm, int MaxNumOfBytes, int *NumOfBytes);
int MW_readSerial_Protocol(int fh_index, unsigned char * data, int MaxNumOfBytes, unsigned int *NumOfBytes, unsigned char *rx_cmd, int bResponse);
int MW_writeSerial_Protocol(int fh_index, unsigned char * data, int ByteCount, unsigned char tx_cmd, unsigned char flags);


