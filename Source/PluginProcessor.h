#pragma once

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
    std::vector<float> delayBuffer;
    int delayBufferSize;
    int writePosition;

    std::array<float, 4> shortDelayTimes;
    std::array<float, 4> longDelayTimes;
    std::array<float, 4> modulatedShortDelayTimes;
    std::array<float, 4> modulatedLongDelayTimes;

    float globalFeedback;
    float lastOutputSampleLeft;
    float lastOutputSampleRight;
    float feedback;
    float shortModulationDepth;
    float longModulationDepth;
    std::array<float, 8> modulationFrequencies;
    std::array<float, 8> modulationPhases;

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
    
    std::array<float, 4> phaseOffsets;
     std::array<float, 4> phaseIncrements;
     float modulationDepth;
    void initializeHadamardMatrix();
    float getInterpolatedSample(float delayTime);
    float generateLFOSample(int lfoIndex);
    void updateLFOs();
    void applyHadamardToLFOs(std::array<float, 4>& lfoOutputs);
    
    std::array<float, 4> applyHadamardMixing(const std::array<float, 4>& inputs)
     {
         std::array<float, 4> outputs;
         for (int i = 0; i < 4; ++i)
         {
             outputs[i] = 0.0f;
             for (int j = 0; j < 4; ++j)
             {
                 outputs[i] += hadamardMatrix[i][j] * inputs[j];
             }
         }
         return outputs;
     }
    
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


    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessor)
};
