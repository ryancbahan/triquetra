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
    const float stereoOffset = 0.02f * sampleRate;  // Adjust stereo offset for clearer separation

    const float earlyReflectionGain = 1.2f;  // Boost early reflection presence
    const float feedbackGain = 0.6f;         // Slightly reduced feedback for smooth blooming
    const float longDelayReverbMix = 0.6f;   // Increased reflection mix into long delays for better reverb effect
    const float allPassMix = 0.4f;           // Reduced all-pass mix to prevent feedback loops

    const float initialIrregularityFactor = 2.5f;  // Increased randomness factor for immediate irregularity
    const float fastBloomFactor = 1.8f;           // Faster modulation for blooming
    const float shortDelayModDepth = 0.003f;      // More noticeable depth for short delays

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

        // Process short delays with immediate irregularity and subdivision
        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i] += (modulationFrequencies[i] * initialIrregularityFactor) / sampleRate;
            if (modulationPhases[i] >= 1.0f) modulationPhases[i] -= 1.0f;

            float baseDelayLeft = shortDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft + stereoOffset;

            // Larger modulation for immediate irregularity in short delays
            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]) * (shortDelayModDepth);
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i] + 0.25f)) * (shortDelayModDepth);

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * baseDelayRight);

            // Get interpolated samples
            shortDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            shortDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            // Stronger feedback and early diffusion for more noticeable delays
            shortDelayOutputLeft[i] += diffusionFeedback[i] * (diffusionFeedbackAmount * earlyReflectionGain);
            shortDelayOutputRight[i] += diffusionFeedback[i + 4] * (diffusionFeedbackAmount * earlyReflectionGain);

            // Hadamard transform for diffusion
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

        // Apply independent all-pass filters to short delay outputs
        std::array<float, 4> allPassOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> allPassOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };
        for (int i = 0; i < 4; ++i)
        {
            float amplitudeLeft = (std::abs(shortDelayOutputLeft[i]) * 0.25f);
            float amplitudeRight = (std::abs(shortDelayOutputRight[i]) * 0.25f);

            float coefficientLeft = std::clamp(0.2f + 0.6f * amplitudeRight, 0.01f, 0.99f);
            float coefficientRight = std::clamp(0.2f + 0.6f * amplitudeLeft, 0.01f, 0.99f);

            allPassFilters[i].setCoefficient(coefficientLeft);
            allPassOutputLeft[i] = allPassFilters[i].process(shortDelayOutputLeft[i]);

            allPassFilters[i].setCoefficient(coefficientRight);
            allPassOutputRight[i] = allPassFilters[i].process(shortDelayOutputRight[i]);
        }

        // Process long delays with cascading bloom and more immediate irregularity
        std::array<float, 4> longDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> longDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> longAllPassOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> longAllPassOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };

        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i + 4] += modulationFrequencies[i + 4] / sampleRate;
            if (modulationPhases[i + 4] >= 1.0f) modulationPhases[i + 4] -= 1.0f;

            float baseDelayLeft = longDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft - stereoOffset;

            // More pronounced modulation for long delays for faster blooming effect
            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i + 4]) * (longModulationDepth * fastBloomFactor);
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i + 4] + 0.5f)) * (longModulationDepth * fastBloomFactor);

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * baseDelayRight);

            // Input from short delay + feedback
            float longDelayInputLeft = shortDelayOutputLeft[i] * (1.0f - diffusionToLongMix) + shortDelayOutputLeft[i] * longDelayReverbMix;
            float longDelayInputRight = shortDelayOutputRight[i] * (1.0f - diffusionToLongMix) + shortDelayOutputRight[i] * longDelayReverbMix;

            longDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            longDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            // Feedback with bloom effect
            longDelayOutputLeft[i] = longDelayInputLeft * (1.0f - longFeedback) + longDelayOutputLeft[i] * feedbackGain;
            longDelayOutputRight[i] = longDelayInputRight * (1.0f - longFeedback) + longDelayOutputRight[i] * feedbackGain;

            // Apply independent all-pass filters to the long delay outputs
            longAllPassOutputLeft[i] = longAllPassFilters[i].process(longDelayOutputLeft[i]);
            longAllPassOutputRight[i] = longAllPassFilters[i].process(longDelayOutputRight[i]);

            // Mix the all-pass output and original long delay output
            longDelayOutputLeft[i] = longDelayOutputLeft[i] * (1.0f - allPassMix) + longAllPassOutputLeft[i] * allPassMix;
            longDelayOutputRight[i] = longDelayOutputRight[i] * (1.0f - allPassMix) + longAllPassOutputRight[i] * allPassMix;
        }

        // Final wet signal mix from short and long delay outputs
        float wetSignalLeft = (allPassOutputLeft[0] + allPassOutputLeft[1] + allPassOutputLeft[2] + allPassOutputLeft[3]) * 0.25f
                            + (longDelayOutputLeft[0] + longDelayOutputLeft[1] + longDelayOutputLeft[2] + longDelayOutputLeft[3]) * 0.25f;

        float wetSignalRight = (allPassOutputRight[0] + allPassOutputRight[1] + allPassOutputRight[2] + allPassOutputRight[3]) * 0.25f
                             + (longDelayOutputRight[0] + longDelayOutputRight[1] + longDelayOutputRight[2] + longDelayOutputRight[3]) * 0.25f;

        // Apply dry/wet mix and output gain
        float outputSampleLeft = inputSampleLeft * dryMix + wetSignalLeft * wetMix;
        float outputSampleRight = inputSampleRight * dryMix + wetSignalRight * wetMix;

        // Soft clipping to prevent distortion
        outputSampleLeft = softClip(applyGain(outputSampleLeft, outputGain));
        outputSampleRight = softClip(applyGain(outputSampleRight, outputGain));

        // Update feedback samples for next iteration
        lastOutputSampleLeft = outputSampleLeft;
        lastOutputSampleRight = outputSampleRight;

        // Write final output to buffer
        buffer.setSample(0, sample, outputSampleLeft);
        buffer.setSample(1, sample, outputSampleRight);

        // Update delay buffer for feedback
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
