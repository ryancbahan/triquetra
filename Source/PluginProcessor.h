#pragma once

#include <JuceHeader.h>

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
    std::array<juce::dsp::IIR::Filter<float>, 2> lowpassFilter;
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
    float lastOutputSample;
    float feedback;
    float shortModulationDepth;
    float longModulationDepth;
    std::array<float, 8> modulationFrequencies;
    std::array<float, 8> modulationPhases;

    std::array<std::array<float, 4>, 4> hadamardMatrix;
    std::array<juce::dsp::IIR::Filter<float>, 2> lowpassFilters;
    
    std::array<float, 4> basedelayTimes;
    std::array<float, 4> modulatedDelayTimes;

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
    float dryMix = 0.7f;
    float shortDelayMix = 0.15f;
    float longDelayMix = 0.15f;
    std::array<float, 4> longModulationPhases = {0.0f, 0.0f, 0.0f, 0.0f};


    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessor)
};
