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

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

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
    std::array<int, 4> readPositions;
    std::array<float, 4> delayTimes;
    float feedback;
    std::array<std::array<float, 4>, 4> hadamardMatrix;

    void initializeHadamardMatrix();
    float getInterpolatedSample(float delayTime);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessor)
};
