/**
 * @file ledblinktask.cpp
 *
 */
/* Copyright (C) 2018-2020 by Arjan van Vught mailto:info@orangepi-dmx.nl
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <cstdint>

#include "ledblink.h"

#include "hardware.h"

void LedBlink::SetFrequency(uint32_t nFreqHz) {
	m_nFreqHz = nFreqHz;

	if (nFreqHz == 0) {
		Hardware::Get()->SetLed(hardware::LedStatus::OFF);
	} else if (nFreqHz > 20) {
		Hardware::Get()->SetLed(hardware::LedStatus::ON);
	} else {
		if (nFreqHz > 1) {
			Hardware::Get()->SetLed(hardware::LedStatus::HEARTBEAT);
		} else {
			Hardware::Get()->SetLed(hardware::LedStatus::FLASH);
		}
	}
}

