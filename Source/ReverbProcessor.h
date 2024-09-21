#pragma once

#include <JuceHeader.h>
#include <array>

class ReverbProcessor
{
public:
    ReverbProcessor();
    void prepare(double sampleRate, int samplesPerBlock);
    void process(const std::array<float, 4>& shortHadamardLeft,
                 const std::array<float, 4>& shortHadamardRight,
                 const std::array<float, 8>& longHadamardLeft,
                 const std::array<float, 8>& longHadamardRight,
                 float& outLeft, float& outRight);

    // Parameter setters
    void setReverbWashDecay(float newDecay) { reverbWashDecay = newDecay; }
    void setReverbWashFeedbackGain(float newGain) { reverbWashFeedbackGain = newGain; }
    void setReverbWashModulationFreq(float newFreq) { reverbWashModulationFreq = newFreq; }
    void setReverbWashModulationDepth(float newDepth) { reverbWashModulationDepth = newDepth; }
    
    // Self-feedback setter
    void setFeedbackGain(float newGain) { feedbackGain = newGain; }

private:
    void updateModulation();

    double sampleRate = 44100.0;
    float reverbWashPhase = 0.0f;
    float reverbWashModulation = 0.0f;
    float reverbWashModulationFreq = 0.1f;
    float reverbWashModulationDepth = 0.1f;
    float reverbWashDecay = 0.99f;
    float reverbWashFeedbackGain = 0.6f;
    float feedbackGain = 0.3f; // Self-feedback gain

    std::array<float, 8> reverbWashLeft{};
    std::array<float, 8> reverbWashRight{};

    std::array<juce::dsp::IIR::Filter<float>, 4> allPassFiltersLong;
    std::array<juce::dsp::IIR::Filter<float>, 6> allPassFiltersShort; // Increased diffusion stages

    juce::dsp::IIR::Filter<float> lowpassFilter;
    juce::dsp::IIR::Filter<float> highpassFilter;

    class DCBlocker {
    public:
        float process(float x);
    private:
        float xm1 = 0.0f, ym1 = 0.0f;
    };
};

