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
    longDelayTimes = {0.5f, 1.0f, 1.5f, 2.0f};  // Long delay times for cascading bloom
    shortModulationDepth = 0.0001f;
    longModulationDepth = 0.05f;
    modulationFrequencies = {0.1f, 0.13f, 0.17f, 0.19f};  // Slow-moving modulations
    phaseOffsets.fill(0.0f);
    phaseIncrements = {0.00001f, 0.000013f, 0.000017f, 0.000019f};  // Slow-changing modulation
    globalFeedback = 0.6f;  // Bloom effect from feedback
    diffusionMix = 0.8f;  // More diffusion
    diffusionFeedbackAmount = 0.6f;
    longFeedback = 0.7f;  // Adjust for blooming effect
    initializeHadamardMatrix();
}


void TriquetraAudioProcessor::initializeHadamardMatrix()
{
    hadamardMatrix = { {
        { 1.0f,  1.0f,  1.0f,  1.0f },
        { 1.0f, -1.0f,  1.0f, -1.0f },
        { 1.0f,  1.0f, -1.0f, -1.0f },
        { 1.0f, -1.0f, -1.0f,  1.0f }
    } };

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

void TriquetraAudioProcessor::initializeLowpassFilter(double sampleRate)
{
    // Set the cutoff frequency to 8kHz
    const double cutoffFrequency = 8000.0;
    const double q = 0.707; // Butterworth Q for smooth response

    auto coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, cutoffFrequency, q);

    lowpassFilterLeft.coefficients = coefficients;
    lowpassFilterRight.coefficients = coefficients;

    // Prepare the filters
    juce::dsp::ProcessSpec spec { sampleRate, static_cast<juce::uint32> (512), 1 };
    lowpassFilterLeft.prepare(spec);
    lowpassFilterRight.prepare(spec);
}


void TriquetraAudioProcessor::prepareToPlay(double sampleRate, int samplesPerBlock)
{
    // Prepare the delay buffer
    const int maxDelaySamples = static_cast<int>(sampleRate * 6.1); // 6 seconds of delay
    delayBufferSize = maxDelaySamples;
    delayBuffer.resize(delayBufferSize, 0.0f);
    writePosition = 0;

    modulatedShortDelayTimes = shortDelayTimes;
    modulatedLongDelayTimes = { 0.5f, 1.0f, 1.5f, 2.0f };  // Extend long delays for cascading trails

    initializeLowpassFilter(sampleRate);

    for (auto& delayLine : delayLines)
    {
        delayLine.resize(delayBufferSize);
        std::fill(delayLine.begin(), delayLine.end(), 0.0f);
    }

    // Prepare lowpass filter
    initializeLowpassFilter(sampleRate);
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

void TriquetraAudioProcessor::updateLowpassCoefficients()
{
    float cutoffFrequency = 15000.0f * lowpassFilterRate;
    auto coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(getSampleRate(), cutoffFrequency, 0.707f);
    lowpassFilterLeft.coefficients = coefficients;
    lowpassFilterRight.coefficients = coefficients;
}

float TriquetraAudioProcessor::getInterpolatedSample(float delayTime)
{
    float delayTimeInSamples = delayTime * getSampleRate();
    int readPosition = static_cast<int>(writePosition - delayTimeInSamples + delayBufferSize) % delayBufferSize;
    
    float fraction = delayTimeInSamples - static_cast<int>(delayTimeInSamples);
    int nextSample = (readPosition + 1) % delayBufferSize;

    // Linear interpolation
    return delayBuffer[readPosition] + fraction * (delayBuffer[nextSample] - delayBuffer[readPosition]);
}

void TriquetraAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    if (totalNumOutputChannels < 2) return;

    // Clear output channels that are not used
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());
    const float stereoOffset = 0.01f * sampleRate;  // Narrower stereo field for tighter reflections

    const float earlyReflectionGain = 1.0f;  // Boost early reflection volume
    const float feedbackGain = 0.6f;         // Adjust feedback for smooth cascading effect
    const float longDelayReverbMix = 0.4f;   // More reflection mix into long delays for reverb-like effect

    const float initialIrregularity = 1.4f;  // Irregularity for short delays
    const float bloomIrregularity = 1.2f;    // Blooming irregularity for longer delays
    const float bloomFactor = 1.1f;          // Slight increase in modulation for blooming

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        float inputSampleLeft = buffer.getSample(0, sample);
        float inputSampleRight = totalNumInputChannels > 1 ? buffer.getSample(1, sample) : inputSampleLeft;

        // Apply input gain
        inputSampleLeft = applyGain(inputSampleLeft, inputGain);
        inputSampleRight = applyGain(inputSampleRight, inputGain);

        float processedInputLeft = inputSampleLeft + lastOutputSampleLeft * globalFeedback;
        float processedInputRight = inputSampleRight + lastOutputSampleRight * globalFeedback;

        std::array<float, 4> shortDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> shortDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };

        // Process short delays with early irregularity and stronger reverb-like presence
        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i] += (modulationFrequencies[i] * initialIrregularity) / sampleRate;
            if (modulationPhases[i] >= 1.0f) modulationPhases[i] -= 1.0f;

            float baseDelayLeft = shortDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft + stereoOffset;

            // Early irregularity and modulation for short delays
            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]) * (shortModulationDepth * 1.5f);
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i] + 0.25f)) * (shortModulationDepth * 1.5f);

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * baseDelayRight);

            // Get interpolated samples
            shortDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            shortDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            // Apply more diffusion and stronger early reflections
            shortDelayOutputLeft[i] += diffusionFeedback[i] * (diffusionFeedbackAmount * earlyReflectionGain);
            shortDelayOutputRight[i] += diffusionFeedback[i + 4] * (diffusionFeedbackAmount * earlyReflectionGain);

            // Hadamard transform for better diffusion spreading
            float hadamardLeft = 0.0f, hadamardRight = 0.0f;
            for (int j = 0; j < 4; ++j)
            {
                hadamardLeft += hadamardMatrix[i][j] * shortDelayOutputLeft[j];
                hadamardRight += hadamardMatrix[i][j] * shortDelayOutputRight[j];
            }

            // Apply lowpass filtering and update feedback
            diffusionFeedback[i] = lowpassFilterLeft.processSample(hadamardLeft);
            diffusionFeedback[i + 4] = lowpassFilterRight.processSample(hadamardRight);

            // More diffusion for short delays
            shortDelayOutputLeft[i] = diffusionFeedback[i] * diffusionMix + processedInputLeft * (1.0f - diffusionMix);
            shortDelayOutputRight[i] = diffusionFeedback[i + 4] * diffusionMix + processedInputRight * (1.0f - diffusionMix);
        }

        // Process long delays for bloom effect and smooth reverb-like qualities
        std::array<float, 4> longDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> longDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };

        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i + 4] += (modulationFrequencies[i + 4] * bloomIrregularity) / sampleRate;
            if (modulationPhases[i + 4] >= 1.0f) modulationPhases[i + 4] -= 1.0f;

            float baseDelayLeft = longDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft - stereoOffset;

            // Blooming modulation for long delays
            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i + 4]) * (longModulationDepth * bloomFactor);
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i + 4] + 0.5f)) * (longModulationDepth * bloomFactor);

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * baseDelayRight);

            // Input from short delays and add feedback
            float longDelayInputLeft = shortDelayOutputLeft[i] * (1.0f - diffusionToLongMix) + shortDelayOutputLeft[i] * longDelayReverbMix;
            float longDelayInputRight = shortDelayOutputRight[i] * (1.0f - diffusionToLongMix) + shortDelayOutputRight[i] * longDelayReverbMix;

            longDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            longDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            // Feedback with bloom effect
            longDelayOutputLeft[i] = longDelayInputLeft * (1.0f - longFeedback) + longDelayOutputLeft[i] * feedbackGain;
            longDelayOutputRight[i] = longDelayInputRight * (1.0f - longFeedback) + longDelayOutputRight[i] * feedbackGain;
        }

        // Mix the wet signal from both short and long delay outputs
        float wetSignalLeft = (shortDelayOutputLeft[0] + shortDelayOutputLeft[1] + shortDelayOutputLeft[2] + shortDelayOutputLeft[3]) * 0.25f
                            + (longDelayOutputLeft[0] + longDelayOutputLeft[1] + longDelayOutputLeft[2] + longDelayOutputLeft[3]) * 0.25f;

        float wetSignalRight = (shortDelayOutputRight[0] + shortDelayOutputRight[1] + shortDelayOutputRight[2] + shortDelayOutputRight[3]) * 0.25f
                             + (longDelayOutputRight[0] + longDelayOutputRight[1] + longDelayOutputRight[2] + longDelayOutputRight[3]) * 0.25f;

        // Apply dry/wet mix and output gain
        float outputSampleLeft = inputSampleLeft * dryMix + wetSignalLeft * wetMix;
        float outputSampleRight = inputSampleRight * dryMix + wetSignalRight * wetMix;

        // Soft clipping and gain adjustment
        outputSampleLeft = softClip(applyGain(outputSampleLeft, outputGain));
        outputSampleRight = softClip(applyGain(outputSampleRight, outputGain));

        // Store last output samples for feedback
        lastOutputSampleLeft = outputSampleLeft;
        lastOutputSampleRight = outputSampleRight;

        // Write final output to buffer
        buffer.setSample(0, sample, outputSampleLeft);
        buffer.setSample(1, sample, outputSampleRight);

        // Update delay buffer
        delayBuffer[writePosition] = (outputSampleLeft + outputSampleRight) * 0.5f;
        writePosition = (writePosition + 1) % delayBufferSize;
    }
}


// Compression function for managing peaks
float TriquetraAudioProcessor::applyCompression(float sample, float threshold, float ratio)
{
    if (std::abs(sample) > threshold)
    {
        sample = threshold + (sample - threshold) / ratio;
    }
    return sample;
}


float TriquetraAudioProcessor::calculateAmplitude(const std::array<float, 4>& signal) {
    float sum = 0.0f;
    for (float s : signal) {
        sum += std::abs(s);
    }
    return sum / 4.0f;
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
