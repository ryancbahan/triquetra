#pragma once

#include <JuceHeader.h>
#include <array>
#include <vector>

class ShortDelayProcessor
{
public:
    ShortDelayProcessor();
    void prepare(double sampleRate, int numChannels, float feedback, float diffusionAmount, float modulationFeedbackAmount);
    
    // Return processed arrays with Hadamard mixed data
    void process(const std::array<float, 4>& shortDelayTimes,
                 const std::array<float, 4>& shortFeedbackLeft,
                 const std::array<float, 4>& shortFeedbackRight,
                 float modulationValue, float stereoOffset,
                 float inputSampleLeft, float inputSampleRight,  // Pass input samples
                 std::array<float, 4>& shortHadamardLeft,
                 std::array<float, 4>& shortHadamardRight);

private:
    float getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples);
    void updateDelayBuffer(float inputLeft, float inputRight);
    std::array<float, 4> applyHadamardMixing(const std::array<float, 4>& input);

    double sampleRate = 44100.0;
    float feedback = 0.0f;
    float diffusionAmount = 0.0f;
    float modulationFeedbackAmount = 0.0f;

    std::array<juce::dsp::IIR::Filter<float>, 4> allPassFiltersShort;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterLeft;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterRight;

    int delayBufferSize = 44100;
    int writePosition = 0;

    std::vector<std::vector<float>> delayBufferLeft;
    std::vector<std::vector<float>> delayBufferRight;
};
