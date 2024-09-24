#include "EnvelopeFollower.h"
#include <cmath>
#include <algorithm>

EnvelopeFollower::EnvelopeFollower()
    : sampleRate(44100.0), numChannels(0)
{
}

void EnvelopeFollower::prepareToPlay(double sampleRate, int numChannels)
{
    this->sampleRate = sampleRate;
    this->numChannels = numChannels;

    previousAbsSamples.resize(numChannels, 0.0f);
    envelopeReadies.resize(numChannels, true);
    envelopeValues.resize(numChannels, 1.0f);
    envelopeActives.resize(numChannels, false);
    envelopeIncrements.resize(numChannels, 0.0f);

    calculateEnvelopeIncrement();
    reset();
}

void EnvelopeFollower::calculateEnvelopeIncrement()
{
    float increment = 1.0f / (attackTime * sampleRate);
    std::fill(envelopeIncrements.begin(), envelopeIncrements.end(), increment);
}

void EnvelopeFollower::reset()
{
    std::fill(envelopeValues.begin(), envelopeValues.end(), 1.0f);
    std::fill(envelopeActives.begin(), envelopeActives.end(), false);
}

void EnvelopeFollower::setAttackTime(float time)
{
    attackTime = time;
    calculateEnvelopeIncrement();
}

void EnvelopeFollower::setAmplitudeJumpThreshold(float threshold)
{
    amplitudeJumpThreshold = threshold;
}

void EnvelopeFollower::setNoiseGateThreshold(float threshold)
{
    noiseGateThreshold = threshold;
}

float EnvelopeFollower::processSample(int channel, float inputSample)
{
    if (channel < 0 || channel >= numChannels)
        return inputSample;

    float absSample = std::abs(inputSample);
    float& prevAbsSample = previousAbsSamples[channel];

    // Check for reset conditions
    if (absSample < noiseGateThreshold)
    {
        envelopeReadies[channel] = true;
    }
    else if (envelopeReadies[channel] && (absSample - prevAbsSample) > amplitudeJumpThreshold)
    {
        envelopeValues[channel] = 0.0f;
        envelopeActives[channel] = true;
        envelopeReadies[channel] = false;
        // DBG("Envelope reset triggered on channel " + juce::String(channel));
    }

    // Update the envelope value if active
    if (envelopeActives[channel])
    {
        envelopeValues[channel] += envelopeIncrements[channel];
        if (envelopeValues[channel] >= 1.0f)
        {
            envelopeValues[channel] = 1.0f;
            envelopeActives[channel] = false;
        }
    }
    else
    {
        // Keep envelope value at 1.0f when not active
        envelopeValues[channel] = 1.0f;
    }

    // Apply envelope to the input sample
    float envelopedSample = inputSample * envelopeValues[channel];

    // Update previous absolute sample
    prevAbsSample = absSample;

    return envelopedSample;
}
