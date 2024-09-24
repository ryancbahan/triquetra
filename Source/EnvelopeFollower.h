#ifndef ENVELOPEFOLLOWER_H_INCLUDED
#define ENVELOPEFOLLOWER_H_INCLUDED

#include <JuceHeader.h>
#include <vector>

class EnvelopeFollower
{
public:
    EnvelopeFollower();
    void prepareToPlay(double sampleRate, int samplesPerBlock, int numChannels);
    void processBlock(juce::AudioBuffer<float>& buffer);
    void reset();

private:
    // Hardcoded parameters
    static constexpr float noiseGateThreshold = 0.01f;
    static constexpr float amplitudeJumpThreshold = 0.05f;
    static constexpr float attackTime = 2.0f;   // Attack time in seconds
    static constexpr float mix = 0.8f;          // 80% wet, 20% dry

    double sampleRate;
    int numChannels;

    std::vector<float> previousPeakAmplitudes;
    std::vector<bool> envelopeReadies;
    std::vector<float> envelopeValues;
    std::vector<bool> envelopeActives;
    std::vector<float> envelopeIncrements;

    void calculateEnvelopeIncrement();
};

#endif // ENVELOPEFOLLOWER_H_INCLUDED
