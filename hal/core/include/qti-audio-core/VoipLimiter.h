/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#if __has_include(<aidl/android/media/audio/common/PcmType.h>)
#include <aidl/android/media/audio/common/PcmType.h>
#define HAS_AIDL_PCM_TYPE 1
#endif

namespace qti::audio::core {

class VoipLimiter {
  public:
    static constexpr float kDefaultThreshold = 0.725f;
    static constexpr float kDefaultReleaseMs = 50.0f;

    explicit VoipLimiter(float threshold = kDefaultThreshold,
                         float releaseMs = kDefaultReleaseMs)
        : mThreshold(threshold),
          mReleaseMs(releaseMs),
          mSampleRate(48000),
          mGain(1.0f) {
        updateCoefficients();
    }

    void reset() {
        mGain = 1.0f;
    }

    void setThreshold(float threshold) {
        mThreshold = std::clamp(threshold, 0.01f, 1.0f);
    }

    float getThreshold() const {
        return mThreshold;
    }

    void setReleaseMs(float releaseMs) {
        mReleaseMs = std::max(releaseMs, 1.0f);
        updateCoefficients();
    }

    float getReleaseMs() const {
        return mReleaseMs;
    }

    void setSampleRate(uint32_t sampleRate) {
        if (sampleRate > 0 && sampleRate != mSampleRate) {
            mSampleRate = sampleRate;
            updateCoefficients();
        }
    }

    float getGain() const {
        return mGain;
    }

    void processInt16(int16_t* buffer, size_t frameCount, size_t channelCount,
                      uint32_t sampleRate) {
        if (!buffer || frameCount == 0 || channelCount == 0) return;
        setSampleRate(sampleRate);

        const int32_t maxSample = static_cast<int32_t>(
                std::clamp(std::round(mThreshold * 32767.0f), 1.0f, 32767.0f));

        for (size_t f = 0; f < frameCount; ++f) {
            int16_t* frame = &buffer[f * channelCount];
            int32_t maxAbs = 0;
            for (size_t ch = 0; ch < channelCount; ++ch) {
                int32_t val = std::abs(static_cast<int32_t>(frame[ch]));
                if (val > maxAbs) maxAbs = val;
            }

            float normalizedPeak = static_cast<float>(maxAbs) / 32768.0f;
            updateGain(normalizedPeak);

            for (size_t ch = 0; ch < channelCount; ++ch) {
                float scaled = static_cast<float>(frame[ch]) * mGain;
                int32_t rounded = static_cast<int32_t>(std::round(scaled));
                frame[ch] = static_cast<int16_t>(std::clamp(rounded, -maxSample, maxSample));
            }
        }
    }

    void processFloat(float* buffer, size_t frameCount, size_t channelCount,
                      uint32_t sampleRate) {
        if (!buffer || frameCount == 0 || channelCount == 0) return;
        setSampleRate(sampleRate);

        for (size_t f = 0; f < frameCount; ++f) {
            float* frame = &buffer[f * channelCount];
            float maxAbs = 0.0f;
            for (size_t ch = 0; ch < channelCount; ++ch) {
                float val = std::abs(frame[ch]);
                if (val > maxAbs) maxAbs = val;
            }

            updateGain(maxAbs);

            for (size_t ch = 0; ch < channelCount; ++ch) {
                frame[ch] = std::clamp(frame[ch] * mGain, -mThreshold, mThreshold);
            }
        }
    }

#ifdef HAS_AIDL_PCM_TYPE
    void process(void* buffer, size_t frameCount, size_t channelCount, uint32_t sampleRate,
                 ::aidl::android::media::audio::common::PcmType pcmType) {
        using ::aidl::android::media::audio::common::PcmType;
        if (pcmType == PcmType::INT_16_BIT) {
            processInt16(static_cast<int16_t*>(buffer), frameCount, channelCount, sampleRate);
        } else if (pcmType == PcmType::FLOAT_32_BIT) {
            processFloat(static_cast<float*>(buffer), frameCount, channelCount, sampleRate);
        }
    }
#endif

  private:
    void updateGain(float normalizedPeak) {
        float targetGain = 1.0f;
        if (normalizedPeak > mThreshold && normalizedPeak > 1e-6f) {
            targetGain = mThreshold / normalizedPeak;
        }

        if (targetGain < mGain) {
            // Instant attack: catch loud peaks immediately with zero overshoot
            mGain = targetGain;
        } else {
            // Exponential release towards targetGain (usually 1.0 when quiet)
            mGain += (targetGain - mGain) * (1.0f - mAlphaRel);
        }
    }

    void updateCoefficients() {
        if (mSampleRate > 0 && mReleaseMs > 0.0f) {
            mAlphaRel = std::exp(-1.0f / (static_cast<float>(mSampleRate) * (mReleaseMs / 1000.0f)));
        } else {
            mAlphaRel = 0.999f;
        }
    }

    float mThreshold;
    float mReleaseMs;
    uint32_t mSampleRate;
    float mGain;
    float mAlphaRel;
};

} // namespace qti::audio::core
