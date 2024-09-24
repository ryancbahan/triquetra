#include "EnvelopeFollower.h"

EnvelopeFollower::EnvelopeFollower()
    : sampleRate(44100.0), numChannels(0)
{
}

void EnvelopeFollower::prepareToPlay(double sampleRate, int samplesPerBlock, int numChannels)
{
    this->sampleRate = sampleRate;
    this->numChannels = numChannels;

    previousPeakAmplitudes.resize(numChannels, 0.0f);
    envelopeReadies.resize(numChannels, true);
    envelopeValues.resize(numChannels, 0.0f);
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
    std::fill(envelopeValues.begin(), envelopeValues.end(), 0.0f);
    std::fill(envelopeActives.begin(), envelopeActives.end(), false);
}

void EnvelopeFollower::processBlock(juce::AudioBuffer<float>& buffer)
{
    int numSamples = buffer.getNumSamples();

    for (int channel = 0; channel < numChannels; ++channel)
    {
        auto* channelData = buffer.getWritePointer(channel);

        // Find peak amplitude in the current buffer
        float peakAmplitude = 0.0f;
        for (int sample = 0; sample < numSamples; ++sample)
        {
            peakAmplitude = std::max(peakAmplitude, std::abs(channelData[sample]));
        }

        // Check for reset conditions
        if (peakAmplitude < noiseGateThreshold)
        {
            envelopeReadies[channel] = true;
        }
        else if (envelopeReadies[channel] && peakAmplitude > previousPeakAmplitudes[channel] + amplitudeJumpThreshold)
        {
            envelopeValues[channel] = 0.0f;
            envelopeActives[channel] = true;
            envelopeReadies[channel] = false;
            DBG("Envelope reset triggered on channel " + juce::String(channel));
        }

        // Process each sample
        for (int sample = 0; sample < numSamples; ++sample)
        {
            float inputSample = channelData[sample];

            // Update the envelope value if active
            if (envelopeActives[channel])
            {
                envelopeValues[channel] += envelopeIncrements[channel];
                if (envelopeValues[channel] >= 1.0f)
                {
                    envelopeValues[channel] = 1.0f;
                    envelopeActives[channel] = false; // Envelope reached max value
                }
            }

            // Apply envelope to the input sample
            float envelopedSample = inputSample * envelopeValues[channel];

            // Mix between dry and wet signal
            channelData[sample] = envelopedSample * mix + inputSample * (1.0f - mix);
        }

        previousPeakAmplitudes[channel] = peakAmplitude;
    }
}
