/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
TriquetraAudioProcessor::TriquetraAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       )
#endif
{
    initializeHadamardMatrix();
    delayStates.fill(0.0f);
    delayTimeInSeconds = 0.5f; // 0.5 seconds delay
    feedback = 0.3f; // Reduced feedback to avoid excessive distortion
}

void TriquetraAudioProcessor::initializeHadamardMatrix()
{
    // 4x4 Hadamard matrix
    hadamardMatrix = {{
        {1.0f,  1.0f,  1.0f,  1.0f},
        {1.0f, -1.0f,  1.0f, -1.0f},
        {1.0f,  1.0f, -1.0f, -1.0f},
        {1.0f, -1.0f, -1.0f,  1.0f}
    }};

    // Normalize the matrix
    float normalizationFactor = 0.5f; // 1 / sqrt(4)
    for (auto& row : hadamardMatrix)
        for (auto& element : row)
            element *= normalizationFactor;
}

TriquetraAudioProcessor::~TriquetraAudioProcessor()
{
}

//==============================================================================
const juce::String TriquetraAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool TriquetraAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool TriquetraAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool TriquetraAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double TriquetraAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int TriquetraAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int TriquetraAudioProcessor::getCurrentProgram()
{
    return 0;
}

void TriquetraAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String TriquetraAudioProcessor::getProgramName (int index)
{
    return {};
}

void TriquetraAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void TriquetraAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = samplesPerBlock;
    spec.numChannels = 1; // Each delay line is mono

    const float maxDelayInSeconds = 0.2f; // 200ms maximum delay
    const int maxDelaySamples = static_cast<int>(sampleRate * maxDelayInSeconds);

    for (auto& delayLine : delayLines)
    {
        delayLine.prepare(spec);
        delayLine.setMaximumDelayInSamples(maxDelaySamples);
        
        // Ensure delay time doesn't exceed maximum
        float actualDelayTime = juce::jmin(delayTimeInSeconds, maxDelayInSeconds);
        delayLine.setDelay(actualDelayTime * sampleRate);
    }
}

void TriquetraAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool TriquetraAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void TriquetraAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        float inputSample = 0.0f;
        for (int channel = 0; channel < totalNumInputChannels; ++channel)
        {
            inputSample += buffer.getSample(channel, sample);
        }
        inputSample /= static_cast<float>(totalNumInputChannels); // Average input across channels

        // Process through delay network
        std::array<float, 4> delayOutputs;
        for (int i = 0; i < 4; ++i)
        {
            delayOutputs[i] = delayLines[i].popSample(0);
        }

        std::array<float, 4> feedbackSamples;
        for (int i = 0; i < 4; ++i)
        {
            feedbackSamples[i] = 0.0f;
            for (int j = 0; j < 4; ++j)
            {
                feedbackSamples[i] += hadamardMatrix[i][j] * delayOutputs[j];
            }
            feedbackSamples[i] *= feedback;
        }

        for (int i = 0; i < 4; ++i)
        {
            delayLines[i].pushSample(0, inputSample + feedbackSamples[i]);
        }

        float outputSample = inputSample;
        for (int i = 0; i < 4; ++i)
        {
            outputSample += delayOutputs[i] * 0.25f; // Mix in delay outputs
        }

        // Write output to all channels
        for (int channel = 0; channel < totalNumOutputChannels; ++channel)
        {
            buffer.setSample(channel, sample, outputSample);
        }
    }
}

//==============================================================================
bool TriquetraAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* TriquetraAudioProcessor::createEditor()
{
    return new TriquetraAudioProcessorEditor (*this);
}

//==============================================================================
void TriquetraAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.
}

void TriquetraAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new TriquetraAudioProcessor();
}
