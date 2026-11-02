/**
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "cli_stream.h"

void cliStream::setSocket(AsyncWebSocket *ws) {
	_ws = ws;
}

void cliStream::setStreamPrefix(char streamPrefix) {
	_streamPrefix = streamPrefix;
}

void cliStream::setClientReady(uint32_t clientId) {
	_clientId    = clientId;
	_clientReady = true;
}

void cliStream::setClientDisconnected() {
	_clientReady = false;
}

size_t cliStream::output(const uint8_t *buffer, size_t size)
{
	if (!_clientReady || _ws == nullptr) {
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

	// by id: the library looks the client up under its own lock, so a client
	// that disconnected in the meantime is simply skipped
	_ws->binary(_clientId, _wsClientBuffer, size + 1);

	return size;
}

uint8_t cliStream::available()
{
	portENTER_CRITICAL(&_lock);
	uint8_t n = _bufferLen - _bufferPosition;
	portEXIT_CRITICAL(&_lock);
	return n;
}

uint8_t cliStream::read()
{
	portENTER_CRITICAL(&_lock);
	cleanBuffer();
	uint8_t c = _buffer[_bufferPosition++];
	portEXIT_CRITICAL(&_lock);
	return c;
}

void cliStream::cleanBuffer()
{
	if (_bufferLen > 0 && _bufferPosition == _bufferLen) {
		_bufferLen = 0;
		_bufferPosition = 0;
	}
}

bool cliStream::input(uint8_t c) {
	portENTER_CRITICAL(&_lock);
	cleanBuffer();
	bool stored = _bufferLen < CLI_STREAM_BUFFER_SIZE;
	if (stored) {
		_buffer[_bufferLen++] = c;
	}
	portEXIT_CRITICAL(&_lock);

	return stored;
}
