#ifndef LONGDELAYPROCESSOR_H_INCLUDED
#define LONGDELAYPROCESSOR_H_INCLUDED

#include <array>
#include <vector>
#include <JuceHeader.h>
#include "EnvelopeFollower.h" // Include the EnvelopeFollower header

class LongDelayProcessor
{
public:
    LongDelayProcessor();
    void reset();
    void prepare(double newSampleRate, int numChannels, float newFeedback, float newBloomFeedbackGain,
                 float newModulationFeedbackAmount, float newAttenuationFactor, float newLongSubdivisionsFactor,
                 float newDecayRate);
    void process(const std::array<float, 4>& longDelayTimes,
                 std::array<float, 8>& longFeedbackLeft,
                 std::array<float, 8>& longFeedbackRight,
                 float modulationValue, float stereoOffset,
                 std::array<float, 8>& longHadamardLeft,
                 std::array<float, 8>& longHadamardRight,
                 float inputSampleLeft, float inputSampleRight,
                 float currentFeedback, float smearValue);

private:
    // Sample rate
    double sampleRate = 44100.0;
    float currentSmearValue = 0.0f; 

    // Delay buffers
    int delayBufferSize = 0;
    std::array<std::vector<float>, 8> delayBufferLeft;
    std::array<std::vector<float>, 8> delayBufferRight;
    int writePosition = 0;

    // Feedback variables
    float feedback = 0.0f;
    float bloomFeedbackGain = 0.0f;
    float modulationFeedbackAmount = 0.0f;
    float attenuationFactor = 0.0f;
    float longSubdivisionsFactor = 0.0f;
    float decayRate = 0.0f;

    // Modulation variables
    float modulationFrequency = 0.1f; // Adjust as needed
    std::array<float, 4> modulationPhaseOffsets; // Offsets for staggered modulation
    std::array<float, 4> modulationPhase;        // Modulation phases

    // Irregular delay factors
    std::array<float, 4> irregularDelayFactors;

    // All-pass filters for diffusion
    std::array<juce::dsp::IIR::Filter<float>, 4> allPassFiltersLong;

    // Envelope followers for input samples
    EnvelopeFollower envelopeFollowerLeft;
    EnvelopeFollower envelopeFollowerRight;

    // Helper functions
    float getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples);
};

#endif // LONGDELAYPROCESSOR_H_INCLUDED
