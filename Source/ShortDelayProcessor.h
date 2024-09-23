#pragma once

#include <JuceHeader.h>
#include <array>
#include <vector>

class ShortDelayProcessor
{
public:
    ShortDelayProcessor();
    void reset();
    void prepare(double newSampleRate, int numChannels, float newFeedback, float newDiffusionAmount, float newModulationFeedbackAmount);
    void process(const std::array<float, 8>& shortDelayTimes,
                 const std::array<float, 8>& shortFeedbackLeft,
                 const std::array<float, 8>& shortFeedbackRight,
                 float modulationValue, float stereoOffset,
                 std::array<float, 8>& shortDelayOutputLeft,
                 std::array<float, 8>& shortDelayOutputRight,
                 float inputSampleLeft, float inputSampleRight,
                 float currentFeedback);
    float getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples);
    void updateDelayBuffer(float inputLeft, float inputRight);
    std::array<float, 8> applyHadamardMixing(const std::array<float, 8>& input);

private:
    double sampleRate;
    float feedback;
    float diffusionAmount;
    float modulationFeedbackAmount;
    int delayBufferSize;
    int writePosition;
    
    std::array<float, 8> modulationPhases;  // Phases for each delay line
    std::array<float, 8> phaseOffsets;      // Phase offsets for each delay line
    float modulationDepth;         

    std::array<float, 8> modulationFrequencies;  
    std::array<juce::dsp::IIR::Filter<float>, 8> allPassFiltersShort;
    std::vector<std::vector<float>> delayBufferLeft;
    std::vector<std::vector<float>> delayBufferRight;

    // Tremolo-related variables
    float modulationFrequency;  // Controls the rate of the tremolo effect
    float modulationPhase;      // Keeps track of the LFO phase for modulation
};
