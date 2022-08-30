#ifndef cliStream_h
#define cliStream_h

#include <Stream.h>

class cliStream: public Stream 
{
	private:
		AsyncWebSocketClient *_wsclient;
	public:
		void setClient(AsyncWebSocketClient * &wsclient);
		virtual int available(void);
		virtual int read(void);
};
#endif
