#pragma once

#include <JuceHeader.h>

class EnvelopeFollower
{
public:
    EnvelopeFollower() = default;

    void reset()
    {
        envelopeValue = 0.0f;
    }

    void setNoiseGateThreshold(float threshold)
    {
        noiseGateThreshold = threshold;
    }

    void setAmplitudeJumpThreshold(float threshold)
    {
        amplitudeJumpThreshold = threshold;
    }

    float processSample(float inputSample, float previousPeakAmplitude, bool& envelopeReady)
    {
        float peakAmplitude = std::abs(inputSample);

        // Check for reset conditions
        if (peakAmplitude < noiseGateThreshold)
        {
            envelopeReady = true;
        }
        else if (envelopeReady && peakAmplitude > previousPeakAmplitude + amplitudeJumpThreshold)
        {
            reset();
            envelopeReady = false;
        }

        // Apply envelope following logic
        envelopeValue = envelopeValue + (1.0f - envelopeValue) * envelopeAttack; // Chase 1.0

        // Apply an extreme curve to the envelope (as in your original code)
        float extremeEnvelope = std::pow(envelopeValue, 4.0f);

        // Return the modified sample
        return inputSample * extremeEnvelope;
    }

private:
    float envelopeValue = 0.0f;
    float noiseGateThreshold = 0.01f; // Default noise gate threshold
    float amplitudeJumpThreshold = 0.05f; // Default amplitude jump threshold
    float envelopeAttack = 0.01f; // Attack time factor, tweak as needed
};
