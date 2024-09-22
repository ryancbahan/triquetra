#pragma once

#include <JuceHeader.h>
#include <array>

class ReverbProcessor
{
public:
    ReverbProcessor();

    void prepare(double sampleRate, int samplesPerBlock);

    void process(const std::array<float, 8>& shortHadamardLeft,
                 const std::array<float, 8>& shortHadamardRight,
                 const std::array<float, 8>& longHadamardLeft,
                 const std::array<float, 8>& longHadamardRight,
                 std::array<float, 8>& outLeft,
                 std::array<float, 8>& outRight);

private:
    void updateModulation();

    double sampleRate;
    float feedbackGain;

    std::array<juce::dsp::IIR::Filter<float>, 8> allPassFiltersLong;
    std::array<juce::dsp::IIR::Filter<float>, 8> allPassFiltersShort;
    juce::dsp::IIR::Filter<float> lowpassFilter;
    juce::dsp::IIR::Filter<float> highpassFilter;

    std::array<float, 8> reverbWashLeft;
    std::array<float, 8> reverbWashRight;

    float reverbWashDecay = 0.95f;
    float reverbWashPhase = 0.0f;
    float reverbWashModulationFreq = 0.1f;
    float reverbWashModulationDepth = 0.001f;
    float reverbWashModulation = 0.0f;
};
