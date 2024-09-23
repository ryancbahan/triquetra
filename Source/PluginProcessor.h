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
    juce::AudioProcessorValueTreeState parameters;
    juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    std::atomic<float>* mixParameter = nullptr;
    std::atomic<float>* delayTimeParameter = nullptr;
    std::atomic<float>* feedbackParameter = nullptr;
    std::atomic<float>* depthParameter = nullptr;
    std::atomic<float>* clockParameter = nullptr;

    juce::SmoothedValue<float> delayTimeSmoothed;

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
    ReverbProcessor reverbProcessor;
    ShortDelayProcessor shortDelayProcessor;
    LongDelayProcessor longDelayProcessor;
    juce::dsp::BallisticsFilter<float> envelopeFollower;
    float previousPeakAmplitude = 0.0f;
    int silentSampleCount = 0;
    const int silentSampleThreshold = 4410; // About 100ms at 44.1kHz
    const float noiseGateThreshold = 0.01f;
    const float amplitudeJumpThreshold = 0.1f;
    std::array<float, 8> shortFeedbackLeft;
    std::array<float, 8> shortFeedbackRight;
    std::array<float, 8> longFeedbackLeft;
    std::array<float, 8> longFeedbackRight;

    std::array<float, 8> reverbWashLeft;
    std::array<float, 8> reverbWashRight;
    float modulationPhase = 0.0f;
    float diffusionAmount = 0.0f;
    float modulationFeedbackAmount = 0.0f;
    float bloomFeedbackGain = 0.0f;
    float attenuationFactor = 0.0f;
    float longSubdivisionsFactor = 0.0f;
    float previousInputLeft = 0.0f;
    float previousOutputLeft = 0.0f;
    float previousInputRight = 0.0f;
    float previousOutputRight = 0.0f;
    float modulationFrequency;
    float modulationValue = 0.0f; 
    void updateModulation(float sampleRate, float depthValue);
    std::vector<float> delayBuffer;
    int delayBufferSize;
    int writePosition;
    std::array<float, 8> shortDelayTimes;
    std::array<float, 4> longDelayTimes;
    std::array<float, 8> modulatedShortDelayTimes;
    std::array<float, 8> modulatedLongDelayTimes;
    
    void updateShortDelayTimes(float delayTime);

    std::tuple<float, float, float, float> processAndSumSignals(
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
    
    std::array<float, 4> basedelayTimes;
    std::array<float, 4> modulatedDelayTimes;
    
    std::array<float, 8> diffusionFeedback = {0}; // 4 for left, 4 for right
    float diffusionFeedbackAmount = 0.6f; // Adjust this value to control the amount of diffusion feedback
    float diffusionMix = 0.7f; // Adjust this to control the mix of diffused signal and input
    float diffusionToLongMix = 0.3f; // Adjust this to control how much diffusion is fed into long delays
    float longFeedback = 0.5f;
    
    inline float clearDenormals(float value);

    std::array<float, 4> phaseOffsets;
    std::array<float, 4> phaseIncrements;
    float modulationDepth;
    
    float softClip(float sample);
    float applyGain(float sample, float gainFactor);

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
    
    std::array<float, 8> shortDelayOutputLeft, shortDelayOutputRight;
    std::array<float, 8> longDelayOutputLeft, longDelayOutputRight;
    std::array<float, 8> reverbOutputLeft, reverbOutputRight;


    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessor)
};
