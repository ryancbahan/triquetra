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
    std::array<float, 4> basedelayTimes;
    std::array<float, 4> modulatedDelayTimes;
    float feedback;
    std::array<std::array<float, 4>, 4> hadamardMatrix;

    // LFO-related members
    float lfoDepth;
    void initializeLowpassFilter(double sampleRate);

    void initializeHadamardMatrix();
    float getInterpolatedSample(float delayTime);
    float generateLFOSample(int lfoIndex);
    void updateLFOs();
    void applyHadamardToLFOs(std::array<float, 4>& lfoOutputs);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessor)
};
