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
    shortDelayTimes = {0.0443f * 2, 0.0531f * 2, 0.0667f * 2, 0.0798f * 2}; // Prime number ratios for less repetitive echoes
    feedback = 0.6f;
    longDelayTimes = {0.25f, 0.5f, 0.75f, 1.0f};
    shortModulationDepth = 0.0001f;
    longModulationDepth = 0.005f;
    modulationFrequencies = {0.1f, 0.13f, 0.17f, 0.19f, 0.23f, 0.29f, 0.31f, 0.37f};
    phaseOffsets.fill(0.0f);
    phaseIncrements = {0.00001f, 0.000013f, 0.000017f, 0.000019f}; // Very slow changes
    globalFeedback = 0.4f; // Initial value, can be changed later
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

void TriquetraAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    const int maxDelaySamples = static_cast<int>(sampleRate * 1.1); // Slightly over 1 second for safety
    delayBufferSize = maxDelaySamples;
    delayBuffer.resize(delayBufferSize, 0.0f);
    writePosition = 0;

    modulatedShortDelayTimes = shortDelayTimes;
    modulatedLongDelayTimes = longDelayTimes;
    
    initializeLowpassFilter(sampleRate);

    for (auto& delayLine : delayLines)
    {
        delayLine.resize(delayBufferSize);
        std::fill(delayLine.begin(), delayLine.end(), 0.0f);
    }

    // Prepare lowpass filter
    juce::dsp::ProcessSpec spec { sampleRate, static_cast<juce::uint32> (samplesPerBlock), 2 };
    for (auto& filter : lowpassFilters)
    {
        filter.prepare(spec);
    }
}

float TriquetraAudioProcessor::getInterpolatedSample(float delayTime)
{
    float delayTimeInSamples = delayTime * getSampleRate();
    int readPosition = static_cast<int>(writePosition - delayTimeInSamples + delayBufferSize) % delayBufferSize;
    
    float fraction = delayTimeInSamples - static_cast<int>(delayTimeInSamples);
    int nextSample = (readPosition + 1) % delayBufferSize;

    return delayBuffer[readPosition] + fraction * (delayBuffer[nextSample] - delayBuffer[readPosition]);
}

float TriquetraAudioProcessor::softClip(float sample)
{
    // Simple hyperbolic tangent soft clipper
    return std::tanh(sample);
}

float TriquetraAudioProcessor::applyGain(float sample, float gainFactor)
{
    return sample * gainFactor;
}

void TriquetraAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // Ensure we have at least a stereo output
    if (totalNumOutputChannels < 2) return;

    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());

    // Stereo offset for delay times (in samples)
    const float stereoOffset = 0.02f * sampleRate; // 20ms offset

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        // Update modulation phases
        for (int i = 0; i < 8; ++i)
        {
            modulationPhases[i] += modulationFrequencies[i] / sampleRate;
            if (modulationPhases[i] >= 1.0f)
                modulationPhases[i] -= 1.0f;
        }

        float inputSampleLeft = buffer.getSample(0, sample);
        float inputSampleRight = totalNumInputChannels > 1 ? buffer.getSample(1, sample) : inputSampleLeft;

        // Apply input gain
        inputSampleLeft = applyGain(inputSampleLeft, inputGain);
        inputSampleRight = applyGain(inputSampleRight, inputGain);

        // Add feedback to input
        float processedInputLeft = inputSampleLeft + lastOutputSampleLeft * globalFeedback;
        float processedInputRight = inputSampleRight + lastOutputSampleRight * globalFeedback;

        std::array<float, 4> shortDelayOutputsLeft, shortDelayOutputsRight;
        std::array<float, 4> longDelayOutputsLeft, longDelayOutputsRight;

        // Process short delays (early reflections)
        for (int i = 0; i < 4; ++i)
        {
            float baseDelayLeft = shortDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft + stereoOffset;

            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]);
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i] + 0.25f)); // 0.25 phase offset for right channel

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * shortModulationDepth * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * shortModulationDepth * baseDelayRight);

            shortDelayOutputsLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            shortDelayOutputsRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);
        }

        // Apply Hadamard matrix to mix short delay outputs
        std::array<float, 4> mixedShortOutputsLeft = applyHadamardMixing(shortDelayOutputsLeft);
        std::array<float, 4> mixedShortOutputsRight = applyHadamardMixing(shortDelayOutputsRight);

        // Invert polarity of some delays for right channel
        mixedShortOutputsRight[1] *= -1.0f;
        mixedShortOutputsRight[3] *= -1.0f;

        // Process long delays (echoes)
        for (int i = 0; i < 4; ++i)
        {
            float baseDelayLeft = longDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft - stereoOffset; // Opposite offset for long delays

            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i + 4]);
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i + 4] + 0.5f)); // 0.5 phase offset for right channel

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * longModulationDepth * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * longModulationDepth * baseDelayRight);

            float longDelayInputLeft = processedInputLeft * 0.5f;
            float longDelayInputRight = processedInputRight * 0.5f;

            for (int j = 0; j < 4; ++j)
            {
                longDelayInputLeft += mixedShortOutputsLeft[j] * 0.125f;
                longDelayInputRight += mixedShortOutputsRight[j] * 0.125f;
            }

            longDelayOutputsLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            longDelayOutputsRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            longDelayOutputsLeft[i] = longDelayInputLeft * 0.5f + longDelayOutputsLeft[i] * 0.5f;
            longDelayOutputsRight[i] = longDelayInputRight * 0.5f + longDelayOutputsRight[i] * 0.5f;
        }

        // Apply Hadamard matrix to mix long delay outputs
        std::array<float, 4> mixedLongOutputsLeft = applyHadamardMixing(longDelayOutputsLeft);
        std::array<float, 4> mixedLongOutputsRight = applyHadamardMixing(longDelayOutputsRight);

        // Calculate the output samples
        float outputSampleLeft = inputSampleLeft * dryMix;
        float outputSampleRight = inputSampleRight * dryMix;

        for (int i = 0; i < 4; ++i)
        {
            outputSampleLeft += mixedShortOutputsLeft[i] * shortDelayMix + mixedLongOutputsLeft[i] * longDelayMix;
            outputSampleRight += mixedShortOutputsRight[i] * shortDelayMix + mixedLongOutputsRight[i] * longDelayMix;
        }

        // Apply soft clipping and final output gain
        outputSampleLeft = applyGain(softClip(outputSampleLeft), outputGain);
        outputSampleRight = applyGain(softClip(outputSampleRight), outputGain);

        // Store the output samples for the next iteration's feedback
        lastOutputSampleLeft = outputSampleLeft;
        lastOutputSampleRight = outputSampleRight;

        // Write the output samples to the buffer
        buffer.setSample(0, sample, outputSampleLeft);
        buffer.setSample(1, sample, outputSampleRight);

        // Update the delay buffer with the processed output (including feedback)
        delayBuffer[writePosition] = (outputSampleLeft + outputSampleRight) * 0.5f;
        writePosition = (writePosition + 1) % delayBufferSize;
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
