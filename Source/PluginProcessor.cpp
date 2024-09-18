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

    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());

    // Increase modulation rates for long delays
    const std::array<float, 4> longModulationRates = {0.5f, 0.7f, 0.9f, 1.1f};

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        // Update modulation phases
        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i] += modulationFrequencies[i] / sampleRate;
            if (modulationPhases[i] >= 1.0f)
                modulationPhases[i] -= 1.0f;
        }
        // Update long delay modulation phases separately
        for (int i = 0; i < 4; ++i)
        {
            longModulationPhases[i] += longModulationRates[i] / sampleRate;
            if (longModulationPhases[i] >= 1.0f)
                longModulationPhases[i] -= 1.0f;
        }

        float inputSample = 0.0f;
        for (int channel = 0; channel < totalNumInputChannels; ++channel)
        {
            inputSample += buffer.getSample(channel, sample);
        }
        inputSample /= static_cast<float>(totalNumInputChannels);

        // Apply input gain
        inputSample = applyGain(inputSample, inputGain);

        // Add feedback to input
        float processedInput = inputSample + lastOutputSample * globalFeedback;

        std::array<float, 4> shortDelayOutputs;
        std::array<float, 4> longDelayOutputs;

        // Process short delays (early reflections)
        for (int i = 0; i < 4; ++i)
        {
            float baseDelay = shortDelayTimes[i] * sampleRate;
            float modulation = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]);
            modulation = std::pow(std::abs(modulation), 0.5f) * std::copysign(1.0f, modulation);
            float modulatedDelay = baseDelay + (modulation * shortModulationDepth * baseDelay);
            modulatedDelay = std::max(modulatedDelay, 0.0f);
            
            shortDelayOutputs[i] = getInterpolatedSample(modulatedDelay / sampleRate);
        }

        // Apply Hadamard matrix to mix short delay outputs
        std::array<float, 4> mixedShortOutputs = applyHadamardMixing(shortDelayOutputs);

        // Apply gain to short delay outputs
        for (int i = 0; i < 4; ++i)
        {
            mixedShortOutputs[i] = applyGain(mixedShortOutputs[i], shortDelayGain);
        }

        // Process long delays (echoes) with enhanced modulation
        for (int i = 0; i < 4; ++i)
        {
            float baseDelay = longDelayTimes[i] * sampleRate;
            
            // Use a combination of sine and triangle waves for more complex modulation
            float sineMod = std::sin(2.0f * juce::MathConstants<float>::pi * longModulationPhases[i]);
            float triangleMod = 2.0f * std::abs(2.0f * (longModulationPhases[i] - std::floor(longModulationPhases[i] + 0.5f))) - 1.0f;
            float modulation = (sineMod + triangleMod) * 0.5f;
            
            // Increase modulation depth for long delays
            float modulatedDelay = baseDelay + (modulation * longModulationDepth * baseDelay * 2.0f);
            modulatedDelay = std::max(modulatedDelay, 0.0f);
            
            // Mix the processed input with all mixed short delay outputs
            float longDelayInput = processedInput * 0.5f;
            for (int j = 0; j < 4; ++j)
            {
                longDelayInput += mixedShortOutputs[j] * 0.125f; // Distribute the short delay outputs evenly
            }
            
            longDelayOutputs[i] = getInterpolatedSample(modulatedDelay / sampleRate);
            longDelayOutputs[i] = longDelayInput * 0.5f + longDelayOutputs[i] * 0.5f;
        }

        // Apply Hadamard matrix to mix long delay outputs
        std::array<float, 4> mixedLongOutputs = applyHadamardMixing(longDelayOutputs);

        // Apply gain to long delay outputs
        for (int i = 0; i < 4; ++i)
        {
            mixedLongOutputs[i] = applyGain(mixedLongOutputs[i], longDelayGain);
        }

        // Calculate the output sample
        float outputSample = inputSample * dryMix;
        for (int i = 0; i < 4; ++i)
        {
            outputSample += mixedShortOutputs[i] * shortDelayMix + mixedLongOutputs[i] * longDelayMix;
        }

        // Apply soft clipping
        outputSample = softClip(outputSample);

        // Apply final output gain
        outputSample = applyGain(outputSample, outputGain);

        // Store the output sample for the next iteration's feedback
        lastOutputSample = outputSample;

        // Write the output sample to all channels
        for (int channel = 0; channel < totalNumOutputChannels; ++channel)
        {
            buffer.setSample(channel, sample, outputSample);
        }

        // Update the delay buffer with the processed output (including feedback)
        delayBuffer[writePosition] = outputSample;
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
