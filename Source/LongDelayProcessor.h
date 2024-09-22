#pragma once

#include <JuceHeader.h>
#include <array>

class LongDelayProcessor
{
public:
    LongDelayProcessor();
    void prepare(double newSampleRate, int numChannels, float newFeedback, float newBloomFeedbackGain, float newModulationFeedbackAmount, float newAttenuationFactor, float newLongSubdivisionsFactor, float newDecayRate);
    void reset();
    
    void process(const std::array<float, 4>& longDelayTimes,
                 std::array<float, 8>& longFeedbackLeft,
                 std::array<float, 8>& longFeedbackRight,
                 float modulationValue, float stereoOffset,
                 std::array<float, 8>& longHadamardLeft,
                 std::array<float, 8>& longHadamardRight,
                 float inputSampleLeft, float inputSampleRight);


private:
    double sampleRate = 44100.0;
    int delayBufferSize = 0;
    int writePosition = 0;
    float decayRate;
    float feedback = 0.5f;
    float bloomFeedbackGain = 0.75f;
    float modulationFeedbackAmount = 0.2f;
    float attenuationFactor = 0.35f;
    float longSubdivisionsFactor = 1.3f;

    std::array<juce::dsp::IIR::Filter<float>, 4> allPassFiltersLong;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterLeft, reverbWashLowpassFilterRight;

    std::vector<std::vector<float>> delayBufferLeft;
    std::vector<std::vector<float>> delayBufferRight;
    inline float clearDenormals(float value);
    float getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples);
    void updateDelayBuffer(float inputLeft, float inputRight);
    std::array<float, 8> applyHadamardMixing(const std::array<float, 8>& input);
    std::array<float, 4> irregularDelayFactors;
};
