#pragma once

#include <JuceHeader.h>
#include <array>
#include <vector>

class ShortDelayProcessor
{
public:
    ShortDelayProcessor();
    void prepare(double sampleRate, int numChannels, float feedback, float diffusionAmount, float modulationFeedbackAmount);
    void reset();
    // Return processed arrays with Hadamard mixed data
    void process(const std::array<float, 8>& shortDelayTimes,
                                      const std::array<float, 8>& shortFeedbackLeft,
                                      const std::array<float, 8>& shortFeedbackRight,
                                      float modulationValue, float stereoOffset,
                                      std::array<float, 8>& shortDelayOutputLeft,
                                      std::array<float, 8>& shortDelayOutputRight,
                                      float inputSampleLeft, float inputSampleRight,
                                      float currentFeedback);

private:
    float getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples);
    void updateDelayBuffer(float inputLeft, float inputRight);
    std::array<float, 8> applyHadamardMixing(const std::array<float, 8>& input);
    float modulationFrequency = 2.66;
     float modulationPhase;
    double sampleRate = 44100.0;
    float feedback = 0.0f;
    float diffusionAmount = 0.0f;
    float modulationFeedbackAmount = 0.0f;

    std::array<juce::dsp::IIR::Filter<float>, 8> allPassFiltersShort;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterLeft;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterRight;

    int delayBufferSize = 44100;
    int writePosition = 0;

    std::vector<std::vector<float>> delayBufferLeft;
    std::vector<std::vector<float>> delayBufferRight;
};
