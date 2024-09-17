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
    basedelayTimes = {0.0443f * 2, 0.0531f * 2, 0.0667f * 2, 0.0798f * 2}; // Prime number ratios for less repetitive echoes
    feedback = 0.6f;
    modulationDepth = 0.0025f; // Adjust as needed
    modulationFrequencies = {0.1f, 0.13f, 0.17f, 0.19f}; // Different frequencies for each delay line
    phaseOffsets.fill(0.0f);
    phaseIncrements = {0.00001f, 0.000013f, 0.000017f, 0.000019f}; // Very slow changes
    initializeHadamardMatrix();
}

void TriquetraAudioProcessor::initializeLowpassFilter(double sampleRate)
{
    // Initialize lowpass filter coefficients
    const double cutoffFrequency = 15000.0; // 15kHz cutoff
    const double q = 0.707; // Butterworth Q

    auto coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, cutoffFrequency, q);

    for (auto& filter : lowpassFilter)
    {
        filter.coefficients = coefficients;
    }
}


void TriquetraAudioProcessor::initializeHadamardMatrix()
{
    hadamardMatrix = {{
        {1.0f,  1.0f,  1.0f,  1.0f},
        {1.0f, -1.0f,  1.0f, -1.0f},
        {1.0f,  1.0f, -1.0f, -1.0f},
        {1.0f, -1.0f, -1.0f,  1.0f}
    }};

    float normalizationFactor = 0.5f;
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
    const int maxDelaySamples = static_cast<int>(sampleRate * 0.2); // 200ms maximum delay
    delayBufferSize = maxDelaySamples;
    delayBuffer.resize(delayBufferSize, 0.0f);
    writePosition = 0;

    modulatedDelayTimes = basedelayTimes;
    
    initializeLowpassFilter(sampleRate);

    for (auto& delayLine : delayLines)
    {
        delayLine.resize(delayBufferSize);
        std::fill(delayLine.begin(), delayLine.end(), 0.0f);
    }
    // Prepare lowpass filter
    juce::dsp::ProcessSpec spec { sampleRate, static_cast<juce::uint32> (samplesPerBlock), 2 };
    for (auto& filter : lowpassFilter)
    {
        filter.prepare(spec);
    }}

void TriquetraAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

float TriquetraAudioProcessor::generateLFOSample(int lfoIndex)
{
    float phase = lfoPhases[lfoIndex];
    float sample = std::sin(2.0f * juce::MathConstants<float>::pi * phase);
    return sample;
}

void TriquetraAudioProcessor::updateLFOs()
{
    for (int i = 0; i < 4; ++i)
    {
        lfoPhases[i] += lfoFrequencies[i] / getSampleRate();
        if (lfoPhases[i] >= 1.0f)
            lfoPhases[i] -= 1.0f;
    }
}

void TriquetraAudioProcessor::applyHadamardToLFOs(std::array<float, 4>& lfoOutputs)
{
    std::array<float, 4> mixedLFOs;
    for (int i = 0; i < 4; ++i)
    {
        mixedLFOs[i] = 0.0f;
        for (int j = 0; j < 4; ++j)
        {
            mixedLFOs[i] += hadamardMatrix[i][j] * lfoOutputs[j];
        }
    }
    lfoOutputs = mixedLFOs;
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

float TriquetraAudioProcessor::getInterpolatedSample(float delayTime)
{
    float delayTimeInSamples = delayTime * getSampleRate();
    int readPosition = static_cast<int>(writePosition - delayTimeInSamples + delayBufferSize) % delayBufferSize;
    
    float fraction = delayTimeInSamples - static_cast<int>(delayTimeInSamples);
    int nextSample = (readPosition + 1) % delayBufferSize;

    return delayBuffer[readPosition] + fraction * (delayBuffer[nextSample] - delayBuffer[readPosition]);
}

void TriquetraAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        // Update modulation phases
        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i] += modulationFrequencies[i] / sampleRate;
            if (modulationPhases[i] >= 1.0f)
                modulationPhases[i] -= 1.0f;
        }

        float inputSample = 0.0f;
        for (int channel = 0; channel < totalNumInputChannels; ++channel)
        {
            inputSample += buffer.getSample(channel, sample);
        }
        inputSample /= static_cast<float>(totalNumInputChannels);

        std::array<float, 4> delayOutputs;
        for (int i = 0; i < 4; ++i)
        {
            float baseDelay = basedelayTimes[i] * sampleRate;
            
            // Enhanced modulation: use non-linear function for more dramatic effect
            float modulation = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]);
            modulation = std::pow(std::abs(modulation), 0.5f) * std::copysign(1.0f, modulation); // Non-linear shaping
            
            float modulatedDelay = baseDelay + (modulation * modulationDepth * baseDelay);
            
            // Ensure delay doesn't go negative
            modulatedDelay = std::max(modulatedDelay, 0.0f);
            
            int readPos = static_cast<int>(writePosition - modulatedDelay + delayBufferSize) % delayBufferSize;
            int nextPos = (readPos + 1) % delayBufferSize;
            float fraction = modulatedDelay - std::floor(modulatedDelay);
            
            delayOutputs[i] = delayBuffer[readPos] + fraction * (delayBuffer[nextPos] - delayBuffer[readPos]);
        }

        // Apply Hadamard matrix to mix modulated outputs
        std::array<float, 4> mixedOutputs;
        for (int i = 0; i < 4; ++i)
        {
            mixedOutputs[i] = 0.0f;
            for (int j = 0; j < 4; ++j)
            {
                mixedOutputs[i] += hadamardMatrix[i][j] * delayOutputs[j];
            }
        }

        float feedbackSample = 0.0f;
        for (const auto& output : mixedOutputs)
        {
            feedbackSample += output;
        }
        feedbackSample *= feedback * 0.25f;

        float newSample = inputSample + feedbackSample;
        
        // Apply lowpass filter to the wet signal
        newSample = lowpassFilter[0].processSample(newSample);

        delayBuffer[writePosition] = newSample;
        writePosition = (writePosition + 1) % delayBufferSize;

        float outputSample = inputSample;
        for (const auto& output : mixedOutputs)
        {
            outputSample += output * 0.25f;
        }

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
