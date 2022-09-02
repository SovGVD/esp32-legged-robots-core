#ifndef cliStream_h
#define cliStream_h

#define CLI_STREAM_BUFFER_SIZE 254	// uint8_t len - 1

#include <Print.h>

class cliStream: public Print
{
	private:
		AsyncWebSocketClient *_wsClient;
		uint8_t _wsClientBuffer[CLI_STREAM_BUFFER_SIZE+1];
		uint8_t _buffer[CLI_STREAM_BUFFER_SIZE];
		uint8_t _bufferLen = 0;
		uint8_t _bufferPosition = 0;
		char _streamPrefix;
		bool _clientReady = false;
	public:
		void setClient(AsyncWebSocketClient * &wsClient);
		void setStreamPrefix(char streamPrefix);
		void setClientReady(AsyncWebSocketClient * &wsClient);
		void setClientDisconnected();
		size_t output(const uint8_t *buffer, size_t size);
		uint8_t available(void);
		uint8_t read(void);
		void cleanBuffer();
		bool input(uint8_t c);

		size_t write(uint8_t c) {
			return output((const uint8_t *)c, 1);
		};
		size_t write(const char *str) {
			if (str == NULL) return 0;
			return output((const uint8_t *)str, strlen(str));
		}
		size_t write(const uint8_t *buffer, size_t size) {
			return output(buffer, size);
		};
		size_t write(const char *buffer, size_t size) {
			return output((const uint8_t *)buffer, size);
		}
};
#endif
