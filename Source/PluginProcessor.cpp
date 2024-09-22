/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <numeric>

//==============================================================================
TriquetraAudioProcessor::TriquetraAudioProcessor()
:   AudioProcessor (BusesProperties()
                    .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                    .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
    parameters(*this, nullptr, "PARAMETERS", createParameterLayout()),
    shortFeedbackLeft({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
    shortFeedbackRight({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
    longFeedbackLeft({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
    longFeedbackRight({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
    reverbWashLeft({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
    reverbWashRight({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
    modulationPhase(0.0f),
    modulationFrequency(0.05f),
    diffusionAmount(0.7f),
    modulationFeedbackAmount(0.2f),
    bloomFeedbackGain(0.75f),
    attenuationFactor(0.35f),
    longSubdivisionsFactor(1.3f),
    previousInputLeft(0.0f),
    previousOutputLeft(0.0f),
    previousInputRight(0.0f),
    previousOutputRight(0.0f),
    delayTimeSmoothed(0.002f)
{
    // Initialize short delay times (prime number ratios for less repetitive echoes)
    shortDelayTimes = {0.0443f * 2, 0.0531f, 0.0667f, 0.0798f * 2, 0.0143f, 0.0531f * 2, 0.09 * 2, 0.12};

    // Initialize feedback and delay times
    feedback = 0.6f;
    longDelayTimes = {0.5f, 1.0f, 1.5f, 2.0f};  // Long delay times for cascading bloom
    shortModulationDepth = 0.0001f;
    longModulationDepth = 0.05f;

    // Initialize LFO modulation frequencies and phases
    modulationFrequencies = {0.1f, 0.13f, 0.17f, 0.19f};  // Slow-moving modulations
    phaseOffsets.fill(0.0f);
    phaseIncrements = {0.00001f, 0.000013f, 0.000017f, 0.000019f};  // Slow-changing modulation

    // Initialize feedback and diffusion parameters
    globalFeedback = 0.6f;  // Bloom effect from feedback
    diffusionMix = 0.8f;    // More diffusion
    diffusionFeedbackAmount = 0.6f;
    longFeedback = 0.7f;    // Adjust for blooming effect

    // Initialize dry/wet mix and input/output gains
    inputGain = 1.0f;
    outputGain = 1.0f;
    
    mixParameter = parameters.getRawParameterValue("mix");
    delayTimeParameter = parameters.getRawParameterValue("delayTime");
}

TriquetraAudioProcessor::~TriquetraAudioProcessor()
{
}

juce::AudioProcessorValueTreeState::ParameterLayout TriquetraAudioProcessor::createParameterLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;
    
    params.push_back(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID("mix", 1), "mix",
        juce::NormalisableRange<float>(0.0f, 1.0f), 0.5f));
    
    params.push_back(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID("delayTime", 2), "Delay time",
        juce::NormalisableRange<float>(0.0f, 2.0f), 0.5f));
    
    return { params.begin(), params.end() };
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

void TriquetraAudioProcessor::prepareToPlay(double sampleRate, int samplesPerBlock)
{
    // Prepare the delay buffer
    const int maxDelaySamples = static_cast<int>(sampleRate * 4.0); // 4 seconds of delay
    delayBufferSize = maxDelaySamples;
    delayBuffer.resize(delayBufferSize, 0.0f);
    writePosition = 0;

    // Zero-out feedback buffers to ensure they start clean on session reload
    shortFeedbackLeft.fill(0.0f);
    shortFeedbackRight.fill(0.0f);
    longFeedbackLeft.fill(0.0f);
    longFeedbackRight.fill(0.0f);

    // Reset processors (e.g., filters and delay lines)
    reverbProcessor.prepare(sampleRate, samplesPerBlock);
    shortDelayProcessor.prepare(sampleRate, getTotalNumOutputChannels(), feedback, diffusionAmount, modulationFeedbackAmount);
    longDelayProcessor.prepare(sampleRate, getTotalNumOutputChannels(), feedback, bloomFeedbackGain, modulationFeedbackAmount, attenuationFactor, longSubdivisionsFactor, 0.9);

    // Reset filters
    reverbWashLowpassFilterLeft.reset();
    reverbWashLowpassFilterRight.reset();
    reverbWashHighpassFilterLeft.reset();
    reverbWashHighpassFilterRight.reset();

    // Reset modulation phase and other time-dependent variables
    modulationPhase = 0.0f;

    // The Hadamard matrix is typically static, so no need to reset it here unless necessary for feedback.
    // If you still need to reset it, you can uncomment the line below.
    // std::fill(hadamardMatrix.begin(), hadamardMatrix.end(), 0.0f);
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

void TriquetraAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // Get and smooth delayTimeParameter to avoid abrupt changes
    float targetDelayTimeValue = delayTimeParameter->load();
    delayTimeSmoothed = 0.99f * delayTimeSmoothed + 0.01f * targetDelayTimeValue;  // Smoothing

    float mixValue = mixParameter->load();

    if (totalNumOutputChannels < 2) return;

    // Clear unused channels
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());
    const float stereoOffset = 0.02f * sampleRate;

    // Dynamically calculate longDelayTimes based on smoothed delay time
    longDelayTimes[0] = delayTimeSmoothed;  // First value is the smoothed delay time
    for (int i = 1; i < longDelayTimes.size(); ++i)
    {
        longDelayTimes[i] = longDelayTimes[i - 1] * 1.25f;  // Each subsequent value is 25% more
    }

    // Ensure that all delay times are within valid bounds
    for (float& delayTime : longDelayTimes)
    {
        delayTime = std::min(delayTime, 4.0f);  // Ensure delay time does not exceed the buffer length (4 seconds)
    }

    const float feedbackGain = 0.7f;

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        float inputSampleLeft = buffer.getSample(0, sample);
        float inputSampleRight = totalNumInputChannels > 1 ? buffer.getSample(1, sample) : inputSampleLeft;

        // Process delays and feedback without matrix modulation
        shortDelayProcessor.process(shortDelayTimes, shortFeedbackLeft, shortFeedbackRight, 0.0f, stereoOffset, shortDelayOutputLeft, shortDelayOutputRight, inputSampleLeft, inputSampleRight);

        longDelayProcessor.process(longDelayTimes, longFeedbackLeft, longFeedbackRight, 0.0f, stereoOffset, longDelayOutputLeft, longDelayOutputRight, inputSampleLeft, inputSampleRight);

        reverbProcessor.process(shortDelayOutputLeft, shortDelayOutputRight,
                                longDelayOutputLeft, longDelayOutputRight,
                                reverbOutputLeft, reverbOutputRight);

        // Combine outputs from the 3 processors for the final mix
        auto [outputSampleLeft, outputSampleRight, wetSignalLeft, wetSignalRight] = processAndSumSignals(
            shortDelayOutputLeft, shortDelayOutputRight,
            longDelayOutputLeft, longDelayOutputRight,
            reverbOutputLeft, reverbOutputRight,
            inputSampleLeft, inputSampleRight,
            1.0 - mixValue, mixValue, outputGain
        );

        // Write final output to buffer
        buffer.setSample(0, sample, outputSampleLeft);
        buffer.setSample(1, sample, outputSampleRight);

        // Update delay buffer for feedback
        delayBuffer[writePosition] = (wetSignalLeft + wetSignalRight) * 0.5f * feedbackGain;
        writePosition = (writePosition + 1) % delayBufferSize;
    }
}



std::tuple<float, float, float, float> TriquetraAudioProcessor::processAndSumSignals(
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
    float outputGain)
{
    // Combine short, long delay, and reverb output for wet signal
    float wetSignalLeft = std::accumulate(shortDelayOutputLeft.begin(), shortDelayOutputLeft.end(), 0.0f) * 0.25f
                        + std::accumulate(longDelayOutputLeft.begin(), longDelayOutputLeft.end(), 0.0f) * 0.25f
                        + std::accumulate(reverbOutputLeft.begin(), reverbOutputLeft.end(), 0.0f) * 0.25f;

    float wetSignalRight = std::accumulate(shortDelayOutputRight.begin(), shortDelayOutputRight.end(), 0.0f) * 0.25f
                        + std::accumulate(longDelayOutputRight.begin(), longDelayOutputRight.end(), 0.0f) * 0.25f
                        + std::accumulate(reverbOutputRight.begin(), reverbOutputRight.end(), 0.0f) * 0.25f;

    // Combine wet and dry signals, apply final output mix
    float outputSampleLeft = inputSampleLeft * dryMix + wetSignalLeft * wetMix;
    float outputSampleRight = inputSampleRight * dryMix + wetSignalRight * wetMix;

    // Apply soft clipping and output gain
    outputSampleLeft = softClip(applyGain(outputSampleLeft, outputGain));
    outputSampleRight = softClip(applyGain(outputSampleRight, outputGain));

    // Safety clipping
    outputSampleLeft = juce::jlimit(-1.0f, 1.0f, outputSampleLeft);
    outputSampleRight = juce::jlimit(-1.0f, 1.0f, outputSampleRight);

    return {outputSampleLeft, outputSampleRight, wetSignalLeft, wetSignalRight};
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
