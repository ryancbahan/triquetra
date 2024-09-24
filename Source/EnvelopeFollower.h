#ifndef ENVELOPEFOLLOWER_H_INCLUDED
#define ENVELOPEFOLLOWER_H_INCLUDED

#include <vector>

class EnvelopeFollower
{
public:
    EnvelopeFollower();
    void prepareToPlay(double sampleRate, int numChannels);
    void reset();

    // Processes a single sample and returns the fully wet signal
    float processSample(int channel, float inputSample);

    // Setters for adjustable parameters
    void setAttackTime(float time);
    void setAmplitudeJumpThreshold(float threshold);
    void setNoiseGateThreshold(float threshold);

private:
    // Parameters
    float noiseGateThreshold = 0.01f;
    float amplitudeJumpThreshold = 0.05f;
    float attackTime = 2.0f;   // Attack time in seconds
    double sampleRate;
    int numChannels;

    // State variables per channel
    std::vector<float> previousAbsSamples;
    std::vector<bool> envelopeReadies;
    std::vector<float> envelopeValues;
    std::vector<bool> envelopeActives;
    std::vector<float> envelopeIncrements;

    void calculateEnvelopeIncrement();
};

#endif // ENVELOPEFOLLOWER_H_INCLUDED
