/**
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "cliStream.h"

void cliStream::setClient(AsyncWebSocketClient * &wsClient) {
	_wsClient = wsClient;
};

void cliStream::setStreamPrefix(char streamPrefix) {
	_streamPrefix = streamPrefix;
}

void cliStream::setClientReady(AsyncWebSocketClient * &wsClient) {
	_clientReady = true;
	_wsClient = wsClient;
}

void cliStream::setClientDisconnected() {
	_clientReady = false;
}

size_t cliStream::output(const uint8_t *buffer, size_t size)
{
	if (!_clientReady) {
		Serial.println("Client not ready");
		return 0;
	}

	if (size >= CLI_STREAM_BUFFER_SIZE) {
		Serial.println("Message too big!");
		return 0;
	}

	_wsClientBuffer[0] = _streamPrefix;
	for (size_t idx = 0; idx < size; idx++) {
		_wsClientBuffer[idx+1] = buffer[idx];
	}

	_wsClient->binary(_wsClientBuffer, size + 1);

	return size;
}

uint8_t cliStream::available()
{
	return _bufferLen - _bufferPosition;
}

uint8_t cliStream::read()
{
	cleanBuffer();
	return _buffer[_bufferPosition++];
}

void cliStream::cleanBuffer()
{
	if (_bufferLen > 0 && _bufferPosition == _bufferLen) {
		_bufferLen = 0;
		_bufferPosition = 0;
	}
}

bool cliStream::input(uint8_t c) {
	cleanBuffer();
	if (_bufferLen == CLI_STREAM_BUFFER_SIZE) {
		return false;
	}
	_buffer[_bufferLen++] = c;

	return true;
}
