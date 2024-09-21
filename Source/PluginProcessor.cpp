/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
TriquetraAudioProcessor::TriquetraAudioProcessor()
    : shortFeedbackLeft({0.0f, 0.0f, 0.0f, 0.0f}),
      shortFeedbackRight({0.0f, 0.0f, 0.0f, 0.0f}),
      longFeedbackLeft({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
      longFeedbackRight({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
      reverbWashLeft({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
      reverbWashRight({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
      modulationPhase(0.0f),
      modulationFrequency(0.05f),
      preDelayWritePos(0),
      preDelaySamples(4410), // Default pre-delay of 100ms at 44.1kHz sample rate
      diffusionAmount(0.7f),
      modulationFeedbackAmount(0.2f),
      bloomFeedbackGain(0.75f),
      attenuationFactor(0.35f),
      longSubdivisionsFactor(1.3f),
      previousInputLeft(0.0f),
      previousOutputLeft(0.0f),
      previousInputRight(0.0f),
      previousOutputRight(0.0f)
{
    // Initialize short delay times (prime number ratios for less repetitive echoes)
    shortDelayTimes = {0.0443f * 2, 0.0531f * 2, 0.0667f * 2, 0.0798f * 2};

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
    dryMix = 0.5f;
    wetMix = 0.5f;
    inputGain = 1.0f;
    outputGain = 1.0f;

    // Initialize Hadamard matrix for feedback routing
    initializeHadamardMatrix();

    // Resize and zero-out pre-delay buffers (assuming a default size)
    preDelayBufferLeft.resize(preDelaySamples, 0.0f);
    preDelayBufferRight.resize(preDelaySamples, 0.0f);
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

    // Set pre-delay based on sample rate (e.g., 100ms pre-delay)
    preDelaySamples = static_cast<int>(0.1 * sampleRate);  // 100ms in samples
    preDelayBufferLeft.resize(preDelaySamples, 0.0f);
    preDelayBufferRight.resize(preDelaySamples, 0.0f);
    preDelayWritePos = 0;

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
    lowpassFilterLeft.reset();
    lowpassFilterRight.reset();
    reverbWashLowpassFilterLeft.reset();
    reverbWashLowpassFilterRight.reset();
    reverbWashHighpassFilterLeft.reset();
    reverbWashHighpassFilterRight.reset();

    // Prepare and initialize lowpass filters based on the current sample rate
    initializeLowpassFilter(sampleRate);

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



float TriquetraAudioProcessor::lowpassFilter(float input, float cutoff, float sampleRate)
{
    static float lastOutput = 0.0f;
    float alpha = 2.0f * juce::MathConstants<float>::pi * cutoff / sampleRate;
    float output = lastOutput + alpha * (input - lastOutput);
    lastOutput = output;
    return output;
}

float TriquetraAudioProcessor::highpassFilter(float input, float cutoff, float sampleRate)
{
    static float lastInput = 0.0f, lastOutput = 0.0f;
    float rc = 1.0f / (cutoff * 2.0f * juce::MathConstants<float>::pi);
    float dt = 1.0f / sampleRate;
    float alpha = dt / (rc + dt);

    float output = alpha * (lastOutput + input - lastInput);  // High-pass filter equation
    lastInput = input;
    lastOutput = output;

    return output;
}

void TriquetraAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    if (totalNumOutputChannels < 2) return;

    // Clear unused channels
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());
    const float stereoOffset = 0.02f * sampleRate;

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        float inputSampleLeft = buffer.getSample(0, sample);
        float inputSampleRight = totalNumInputChannels > 1 ? buffer.getSample(1, sample) : inputSampleLeft;

        // Apply DC offset removal
        float processedInputLeft = removeDCOffset(inputSampleLeft, previousInputLeft, previousOutputLeft);
        float processedInputRight = removeDCOffset(inputSampleRight, previousInputRight, previousOutputRight);

        processedInputLeft = applyGain(processedInputLeft, inputGain);
        processedInputRight = applyGain(processedInputRight, inputGain);

        std::array<float, 4> shortDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> shortDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> longDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> longDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> reverbOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> reverbOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        
        // Update modulation
        modulationPhase += modulationFrequency / sampleRate;
        if (modulationPhase >= 1.0f) modulationPhase -= 1.0f;
        float modulationValue = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhase) * modulationDepth;

        // Define LFO-modulated matrix for cross-feedback
        std::array<std::array<float, 8>, 4> shortToLongMatrix;
        std::array<std::array<float, 8>, 4> shortToReverbMatrix;
        std::array<std::array<float, 4>, 8> longToShortMatrix;
        std::array<std::array<float, 8>, 8> longToReverbMatrix;

        // Initialize LFO-modulated matrix values (simple sine modulation)
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 8; ++j) {
                float lfoMod = 0.5f + 0.5f * std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhase + i * 0.1f + j * 0.05f));
                shortToLongMatrix[i][j] = lfoMod;
                shortToReverbMatrix[i][j] = lfoMod * 0.8f; // slightly lower influence
            }
        }

        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 4; ++j) {
                float lfoMod = 0.5f + 0.5f * std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhase + i * 0.07f + j * 0.03f));
                longToShortMatrix[i][j] = lfoMod;
            }
            for (int j = 0; j < 8; ++j) {
                float lfoMod = 0.5f + 0.5f * std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhase + i * 0.08f + j * 0.06f));
                longToReverbMatrix[i][j] = lfoMod * 0.9f; // slightly attenuate
            }
        }

        // Process delays and feedback
        shortDelayProcessor.process(shortDelayTimes, shortFeedbackLeft, shortFeedbackRight, modulationValue, stereoOffset, shortDelayOutputLeft, shortDelayOutputRight, processedInputLeft, processedInputRight);
        longDelayProcessor.process(longDelayTimes, longFeedbackLeft, longFeedbackRight, modulationValue, stereoOffset, longDelayOutputLeft, longDelayOutputRight, processedInputLeft, processedInputRight);

        reverbProcessor.process(shortDelayOutputLeft, shortDelayOutputRight,
                                longDelayOutputLeft, longDelayOutputRight,
                                reverbOutputLeft, reverbOutputRight);

        // Apply cross-feedback from short delays to long delays and reverb
        for (int i = 0; i < 8; ++i) {
            longDelayOutputLeft[i] += shortToLongMatrix[i % 4][i] * shortDelayOutputLeft[i % 4];
            longDelayOutputRight[i] += shortToLongMatrix[i % 4][i] * shortDelayOutputRight[i % 4];
            
            reverbOutputLeft[i] += shortToReverbMatrix[i % 4][i] * shortDelayOutputLeft[i % 4];
            reverbOutputRight[i] += shortToReverbMatrix[i % 4][i] * shortDelayOutputRight[i % 4];
        }

        // Cross-feedback from long delays into short delays
        for (int i = 0; i < 4; ++i) {
            shortDelayOutputLeft[i] += longToShortMatrix[i][i] * longDelayOutputLeft[i % 8];
            shortDelayOutputRight[i] += longToShortMatrix[i][i] * longDelayOutputRight[i % 8];
        }

        // Apply cross-feedback from long delays to reverb
        for (int i = 0; i < 8; ++i) {
            reverbOutputLeft[i] += longToReverbMatrix[i][i] * longDelayOutputLeft[i];
            reverbOutputRight[i] += longToReverbMatrix[i][i] * longDelayOutputRight[i];
        }

        auto [outputSampleLeft, outputSampleRight] = processAndSumSignals(
            shortDelayOutputLeft, shortDelayOutputRight,
            longDelayOutputLeft, longDelayOutputRight,
            reverbOutputLeft, reverbOutputRight,
            processedInputLeft, processedInputRight,
            dryMix, wetMix, outputGain
        );

        // Write final output to buffer
        buffer.setSample(0, sample, outputSampleLeft);
        buffer.setSample(1, sample, outputSampleRight);

        // Update delay buffer for feedback
        delayBuffer[writePosition] = (outputSampleLeft + outputSampleRight) * 0.5f;
        writePosition = (writePosition + 1) % delayBufferSize;
    }
}


std::pair<float, float> TriquetraAudioProcessor::processAndSumSignals(
    const std::array<float, 4>& shortDelayOutputLeft,
    const std::array<float, 4>& shortDelayOutputRight,
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
    float wetSignalLeft = (shortDelayOutputLeft[0] + shortDelayOutputLeft[1] + shortDelayOutputLeft[2] + shortDelayOutputLeft[3]) * 0.25f
                        + (longDelayOutputLeft[0] + longDelayOutputLeft[1] + longDelayOutputLeft[2] + longDelayOutputLeft[3]
                        + longDelayOutputLeft[4] + longDelayOutputLeft[5] + longDelayOutputLeft[6] + longDelayOutputLeft[7]) * 0.125f
                        + (reverbOutputRight[0] + reverbOutputRight[1] + reverbOutputRight[2] + reverbOutputRight[3]);

    float wetSignalRight = (shortDelayOutputRight[0] + shortDelayOutputRight[1] + shortDelayOutputRight[2] + shortDelayOutputRight[3]) * 0.25f
                         + (longDelayOutputRight[0] + longDelayOutputRight[1] + longDelayOutputRight[2] + longDelayOutputRight[3]
                         + longDelayOutputRight[4] + longDelayOutputRight[5] + longDelayOutputRight[6] + longDelayOutputRight[7]) * 0.125f
                         + (reverbOutputLeft[0] + reverbOutputLeft[1] + reverbOutputLeft[2] + reverbOutputLeft[3]);

    // Combine wet and dry signals, apply final output mix
    float outputSampleLeft = inputSampleLeft * dryMix + wetSignalLeft * wetMix;
    float outputSampleRight = inputSampleRight * dryMix + wetSignalRight * wetMix;

    // Apply soft clipping and output gain
    outputSampleLeft = softClip(applyGain(outputSampleLeft, outputGain));
    outputSampleRight = softClip(applyGain(outputSampleRight, outputGain));

    // Safety clipping
    outputSampleLeft = juce::jlimit(-1.0f, 1.0f, outputSampleLeft);
    outputSampleRight = juce::jlimit(-1.0f, 1.0f, outputSampleRight);

    return {outputSampleLeft, outputSampleRight};
}


inline float TriquetraAudioProcessor::clearDenormals(float value)
{
    return std::abs(value) < 1.0e-15f ? 0.0f : value;
}

float TriquetraAudioProcessor::removeDCOffset(float input, float& previousInput, float& previousOutput)
{
    const float alpha = 0.99f;  // Filter coefficient, adjust this to control the cutoff
    float output = input - previousInput + alpha * previousOutput;
    previousInput = input;
    previousOutput = output;
    return output;
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
