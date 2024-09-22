#pragma once
#include "ReverbProcessor.h"
#include "ShortDelayProcessor.h"
#include "LongDelayProcessor.h"
#include <array>
#include <utility>
#include <JuceHeader.h>
struct AllPassFilter {
    float a = 0.0f;
    float z1 = 0.0f;

    float process(float input) {
        float output = a * input + z1;
        z1 = input - a * output;
        return output;
    }

    void setCoefficient(float coefficient) {
        a = coefficient;
    }
};


class TriquetraAudioProcessor  : public juce::AudioProcessor
{
public:
    TriquetraAudioProcessor();
    ~TriquetraAudioProcessor() override;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    const juce::String getName() const override;
    bool isBusesLayoutSupported (const BusesLayout& layouts) const;
    void initializeLowpassFilter();
    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;
    
    std::array<float, 4> lfoPhases;
    std::array<float, 4> lfoFrequencies;
    float pitchShiftDepth;
    std::array<std::vector<float>, 4> delayLines;
    std::array<float, 4> fractionalDelays;

    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

private:
    std::array<float, 8> shortFeedbackLeft;
    std::array<float, 8> shortFeedbackRight;
    std::array<float, 8> longFeedbackLeft;
    std::array<float, 8> longFeedbackRight;

    int preDelayWritePos = 0;
    std::array<float, 8> reverbWashLeft;
    std::array<float, 8> reverbWashRight;
    float modulationPhase = 0.0f;
    std::vector<float> preDelayBufferLeft;
    std::vector<float> preDelayBufferRight;
    int preDelaySamples = 0;
    float diffusionAmount = 0.0f;
    float modulationFeedbackAmount = 0.0f;
    float bloomFeedbackGain = 0.0f;
    float attenuationFactor = 0.0f;
    float longSubdivisionsFactor = 0.0f;
    float previousInputLeft = 0.0f;
    float previousOutputLeft = 0.0f;
    float previousInputRight = 0.0f;
    float previousOutputRight = 0.0f;
    float modulationFrequency;  // Add this declaration

    ReverbProcessor reverbProcessor;
    ShortDelayProcessor shortDelayProcessor;
    LongDelayProcessor longDelayProcessor;

    std::vector<float> delayBuffer;
    int delayBufferSize;
    int writePosition;
    float lowpassFilter(float input, float cutoff, float sampleRate);
    std::array<float, 8> shortDelayTimes;
    std::array<float, 4> longDelayTimes;
    std::array<float, 8> modulatedShortDelayTimes;
    std::array<float, 8> modulatedLongDelayTimes;
    
    float getCubicInterpolatedSample(float delayTime);
    std::array<AllPassFilter, 4> longAllPassFilters;
    std::pair<float, float> processAndSumSignals(
        const std::array<float, 8>& shortDelayOutputLeft,
        const std::array<float, 8>& shortDelayOutputRight,
        const std::array<float, 8>& longDelayOutputLeft,
        const std::array<float, 8>& longDelayOutputRight,
        const std::array<float, 8>& reverbOutputLeft,
        const std::array<float, 8>& reverbOutputRight,
        float inputSampleLeft,
        float inputSampleRight,
        float dryMix,
        float wetMix,
     float outputGain);

    float globalFeedback;
    float lastOutputSampleLeft;
    float lastOutputSampleRight;
    float feedback;
    float shortModulationDepth;
    float longModulationDepth;
    std::array<float, 8> modulationFrequencies;
    std::array<float, 8> modulationPhases;
    float applyCompression(float sample, float threshold, float ratio);
    std::pair<float, float> processAndSumSignals(
        const std::array<float, 4>& shortDelayOutputLeft,
        const std::array<float, 4>& shortDelayOutputRight,
        const std::array<float, 8>& longDelayOutputLeft,
        const std::array<float, 8>& longDelayOutputRight,
        const std::array<float, 8>& reverbOutputLeft,
        const std::array<float, 8>& reverbOutputRight,
        float inputSampleLeft,
        float inputSampleRight,
        float dryMix,
        float wetMix,
        float outputGain);

    std::array<std::array<float, 4>, 4> hadamardMatrix;
    juce::dsp::IIR::Filter<float> lowpassFilterLeft;
    juce::dsp::IIR::Filter<float> lowpassFilterRight;
    std::array<AllPassFilter, 4> allPassFilters;
    
    std::array<float, 4> basedelayTimes;
    std::array<float, 4> modulatedDelayTimes;
    void updateLowpassCoefficients();
    
    juce::dsp::IIR::Filter<float> allPassFilterLeft;
      juce::dsp::IIR::Filter<float> allPassFilterRight;
    
    float lowpassFilterRate = 0.95f; // Controls how quickly the lowpass filter frequency decreases
    std::array<float, 8> diffusionFeedback = {0}; // 4 for left, 4 for right
    float diffusionFeedbackAmount = 0.6f; // Adjust this value to control the amount of diffusion feedback
    float diffusionMix = 0.7f; // Adjust this to control the mix of diffused signal and input
    float diffusionToLongMix = 0.3f; // Adjust this to control how much diffusion is fed into long delays
    float longFeedback = 0.5f;

    float calculateAmplitude(const std::array<float, 4>& signal);

    // LFO-related members
    float lfoDepth;
    void initializeLowpassFilter(double sampleRate);
    // Arrays for All-Pass Filters
    std::array<juce::dsp::IIR::Filter<float>, 4> allPassFiltersShort;
    std::array<juce::dsp::IIR::Filter<float>, 4> allPassFiltersLong;
    inline float clearDenormals(float value);

    std::array<float, 4> phaseOffsets;
     std::array<float, 4> phaseIncrements;
     float modulationDepth;
    void initializeHadamardMatrix();
    float getInterpolatedSample(float delayTime);
    float generateLFOSample(int lfoIndex);
    void updateLFOs();
    void applyHadamardToLFOs(std::array<float, 4>& lfoOutputs);
    float removeDCOffset(float input, float& previousInput, float& previousOutput);
    template<size_t N>
    std::array<float, N> applyHadamardMixing(const std::array<float, N>& inputs)
    {
        std::array<float, N> outputs;
        for (size_t i = 0; i < N; ++i)
        {
            outputs[i] = 0.0f;
            for (size_t j = 0; j < N; ++j)
            {
                outputs[i] += hadamardMatrix[i % 4][j % 4] * inputs[j];
            }
        }
        return outputs;
    }
    
    float reverbWashDucking = 0.0f;  // Amount of ducking (0.0 to 1.0)
    float reverbWashDecay = 0.99f;   // Adjustable decay factor
    float reverbWashThreshold = 0.01f; 
    
    float softClip(float sample);
    float applyGain(float sample, float gainFactor);
    float highpassFilter(float input, float cutoff, float sampleRate);
    float inputGain = 1.0f;
    float shortDelayGain = 1.0f;
    float longDelayGain = 1.0f;
    float outputGain = 1.0f;
    float dryMix = 0.5f;
    float wetMix = 0.5f;
    float shortDelayMix = 0.5f;
    float longDelayMix = 0.3f;
    std::array<float, 4> longModulationPhases = {0.0f, 0.0f, 0.0f, 0.0f};
    float currentFeedbackLevel = 0.0f;
    juce::dsp::IIR::Filter<float> reverbWashHighpassFilterLeft;
    juce::dsp::IIR::Filter<float> reverbWashHighpassFilterRight;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterLeft;
    juce::dsp::IIR::Filter<float> reverbWashLowpassFilterRight;
    
    // Band-pass filters for midrange emphasis
    juce::dsp::IIR::Filter<float> bandPassFilterLeft;
    juce::dsp::IIR::Filter<float> bandPassFilterRight;


    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessor)
};
