/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace qti::audio::core {

/*
 * Xiaomi's sensors.ultrasoundproximity.so asks the audio HAL over the audio_us_socket_0 init
 * socket to run the ultrasound stream, and gets the near/far events back through
 * libultrasound_notify.
 */
void ultrasoundProximityStart();
void ultrasoundProximityEnable(bool enable);

} // namespace qti::audio::core
