/**
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "cliStream.h"

void cliStream::setClient(AsyncWebSocketClient * &wsclient) {
	_wsclient = wsclient;
};

int cliStream::available(void) {
	// @TODO
	return 0;
}

int cliStream::read(void) {
	// @TODO
	return -1;
}
