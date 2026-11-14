/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Look-ahead peak limiter for VoIP playback on the onyx earpiece.
 *
 * Loud or high-pitched far-end speech distorts on the earpiece path once the level that reaches
 * the DSP goes above about 0.58 of full scale (the call volume step at -4.7 dB is clean, the one
 * at -2.35 dB is not). Scaling the whole stream down by that much makes every VoIP app quieter,
 * so only the peaks above the threshold are brought down here, and the rest passes unchanged.
 *
 * The hardware volume is applied in the DSP after this, so the threshold is kSafeLevel divided by
 * that volume, capped at 1.0: 0.58 at full volume, 0.76 one step down, and no limiting at all from
 * the 0.58 step downwards.
 *
 * The output is delayed by a short look-ahead, so the gain is already down when a peak leaves
 * the delay line. That avoids flattening the waveform tops, which an instant-attack limiter
 * without look-ahead does and which sounds like the crackle this is meant to remove.
 *
 * process() runs on the stream worker thread. reset() may be called from binder threads, so it
 * only raises a flag that process() acts on.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#if __has_include(<aidl/android/media/audio/common/PcmType.h>)
#include <aidl/android/media/audio/common/PcmType.h>
#define VOIP_LIMITER_HAS_AIDL_PCM_TYPE 1
#endif

namespace qti::audio::core {

class VoipLimiter {
  public:
    static constexpr float kSafeLevel = 0.58f;   // -4.7 dBFS at the DSP, the clean call volume step
    static constexpr float kLookaheadMs = 3.0f;
    static constexpr float kReleaseMs = 80.0f;
    static constexpr size_t kMaxChannels = 8;

    void reset() { mResetPending.store(true, std::memory_order_release); }

    // Hardware volume the DSP applies after this limiter (largest channel). May be called from
    // binder threads.
    void setHwVolume(float volume) {
        const float threshold = volume > 0.0f ? std::min(kSafeLevel / volume, 1.0f) : 1.0f;
        mThresholdTarget.store(threshold, std::memory_order_relaxed);
    }

    // Returns false for formats it does not handle; the buffer is then left untouched.
    bool processInt16(int16_t* buffer, size_t frameCount, size_t channelCount,
                      uint32_t sampleRate) {
        if (!prepare(frameCount, channelCount, sampleRate) || !buffer) return false;
        for (size_t f = 0; f < frameCount; ++f) {
            float in[kMaxChannels];
            for (size_t ch = 0; ch < channelCount; ++ch) {
                in[ch] = static_cast<float>(buffer[f * channelCount + ch]) / 32768.0f;
            }
            float out[kMaxChannels];
            processFrame(in, out);
            for (size_t ch = 0; ch < channelCount; ++ch) {
                const float s = std::round(out[ch] * 32768.0f);
                buffer[f * channelCount + ch] =
                        static_cast<int16_t>(std::clamp(s, -32768.0f, 32767.0f));
            }
        }
        return true;
    }

    bool processFloat(float* buffer, size_t frameCount, size_t channelCount,
                      uint32_t sampleRate) {
        if (!prepare(frameCount, channelCount, sampleRate) || !buffer) return false;
        for (size_t f = 0; f < frameCount; ++f) {
            float out[kMaxChannels];
            processFrame(&buffer[f * channelCount], out);
            std::copy(out, out + channelCount, &buffer[f * channelCount]);
        }
        return true;
    }

#ifdef VOIP_LIMITER_HAS_AIDL_PCM_TYPE
    bool process(void* buffer, size_t frameCount, size_t channelCount, uint32_t sampleRate,
                 ::aidl::android::media::audio::common::PcmType pcmType) {
        using ::aidl::android::media::audio::common::PcmType;
        if (pcmType == PcmType::INT_16_BIT) {
            return processInt16(static_cast<int16_t*>(buffer), frameCount, channelCount,
                                sampleRate);
        }
        if (pcmType == PcmType::FLOAT_32_BIT) {
            return processFloat(static_cast<float*>(buffer), frameCount, channelCount, sampleRate);
        }
        return false;
    }
#endif

  private:
    bool prepare(size_t frameCount, size_t channelCount, uint32_t sampleRate) {
        if (frameCount == 0 || channelCount == 0 || channelCount > kMaxChannels ||
            sampleRate == 0) {
            return false;
        }
        const bool reconfigure = channelCount != mChannels || sampleRate != mSampleRate;
        if (reconfigure) {
            mChannels = channelCount;
            mSampleRate = sampleRate;
            mLookahead = std::max<size_t>(1, static_cast<size_t>(sampleRate * kLookaheadMs / 1000.0f));
            mDelay.assign(mLookahead * mChannels, 0.0f);
            mPeakWindow.assign(mLookahead, 0.0f);
            // Attack: reach the target within the look-ahead. Release: back to unity smoothly.
            mAttackCoef = 1.0f - std::exp(-4.0f / static_cast<float>(mLookahead));
            mReleaseCoef = 1.0f - std::exp(-1000.0f / (kReleaseMs * static_cast<float>(sampleRate)));
        }
        // Always consume the flag, so a reset requested before the first configuration does not
        // clear the delay line again one buffer later.
        const bool resetRequested = mResetPending.exchange(false, std::memory_order_acq_rel);
        if (reconfigure || resetRequested) {
            std::fill(mDelay.begin(), mDelay.end(), 0.0f);
            std::fill(mPeakWindow.begin(), mPeakWindow.end(), 0.0f);
            mPos = 0;
            mGain = 1.0f;
        }
        mThreshold = mThresholdTarget.load(std::memory_order_relaxed);
        return true;
    }

    void processFrame(const float* in, float* out) {
        float peak = 0.0f;
        for (size_t ch = 0; ch < mChannels; ++ch) peak = std::max(peak, std::fabs(in[ch]));
        mPeakWindow[mPos] = peak;

        // Loudest frame still inside the look-ahead window, against the current threshold, so a
        // volume change applies to everything that has not been output yet.
        float windowPeak = 0.0f;
        for (float p : mPeakWindow) windowPeak = std::max(windowPeak, p);
        const float target = windowPeak > mThreshold ? mThreshold / windowPeak : 1.0f;

        mGain += (target - mGain) * (target < mGain ? mAttackCoef : mReleaseCoef);

        // Output the frame that entered the delay line one look-ahead ago, then store this one.
        for (size_t ch = 0; ch < mChannels; ++ch) {
            const float delayed = mDelay[mPos * mChannels + ch];
            float sample = delayed * mGain;
            // The smoothed attack can leave a sliver above the threshold; hold it there.
            if (mThreshold < 1.0f) sample = std::clamp(sample, -mThreshold, mThreshold);
            out[ch] = sample;
            mDelay[mPos * mChannels + ch] = in[ch];
        }
        mPos = (mPos + 1) % mLookahead;
    }

    std::atomic<bool> mResetPending{true};
    std::atomic<float> mThresholdTarget{kSafeLevel};
    float mThreshold = kSafeLevel;
    size_t mChannels = 0;
    uint32_t mSampleRate = 0;
    size_t mLookahead = 1;
    size_t mPos = 0;
    float mGain = 1.0f;
    float mAttackCoef = 1.0f;
    float mReleaseCoef = 0.001f;
    std::vector<float> mDelay;
    std::vector<float> mPeakWindow;
};

} // namespace qti::audio::core
