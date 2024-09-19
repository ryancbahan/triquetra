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
    
    juce::dsp::ProcessSpec spec;
       spec.sampleRate = sampleRate;
       spec.maximumBlockSize = samplesPerBlock;
       spec.numChannels = getTotalNumOutputChannels();
    
    juce::dsp::IIR::Coefficients<float>::Ptr lowpassCoefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 8000.0f);
      
      lowpassFilterLeft.coefficients = lowpassCoefficients;
      lowpassFilterRight.coefficients = lowpassCoefficients;

      lowpassFilterLeft.prepare(spec);
      lowpassFilterRight.prepare(spec);
    
    for (auto& filter : allPassFiltersShort)
    {
        filter.prepare(spec);
        filter.coefficients = juce::dsp::IIR::Coefficients<float>::makeAllPass(sampleRate, 800.0); // Example frequency, tweak as needed
    }

    // Initialize All-Pass Filters (long delays)
    for (auto& filter : allPassFiltersLong)
    {
        filter.prepare(spec);
        filter.coefficients = juce::dsp::IIR::Coefficients<float>::makeAllPass(sampleRate, 400.0); // Example frequency, tweak as needed
    }
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

    // Clear unused channels
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    float sampleRate = static_cast<float>(getSampleRate());
    const float stereoOffset = 0.02f * sampleRate;

    // Updated parameters for irregularity and long feedback
    const float bloomDelayModDepth = 0.15f;   // More depth for subdividing delays
    const float bloomFeedbackGain = 0.85f;    // Longer sustain for bloom effect
    const float feedbackIncreaseRate = 0.04f; // Increase feedback more gradually for sustain
    const float longSubdivisionsFactor = 1.75f; // Factor for subdividing long delays irregularly

    // Variables for DC Offset Removal
    float previousInputLeft = 0.0f, previousOutputLeft = 0.0f;
    float previousInputRight = 0.0f, previousOutputRight = 0.0f;

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        float inputSampleLeft = buffer.getSample(0, sample);
        float inputSampleRight = totalNumInputChannels > 1 ? buffer.getSample(1, sample) : inputSampleLeft;

        // Apply DC offset removal only to the wet signal path, not the dry signal
        float processedInputLeft = inputSampleLeft;
        float processedInputRight = inputSampleRight;

        processedInputLeft = applyGain(processedInputLeft, inputGain);
        processedInputRight = applyGain(processedInputRight, inputGain);

        processedInputLeft = removeDCOffset(processedInputLeft, previousInputLeft, previousOutputLeft);
        processedInputRight = removeDCOffset(processedInputRight, previousInputRight, previousOutputRight);

        std::array<float, 4> shortDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> shortDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> longDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // 8 taps for long delays
        std::array<float, 8> longDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // 8 taps for long delays

        // Process short delays and apply all-pass filtering with light modulation
        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i] += (modulationFrequencies[i] * 2.0f) / sampleRate;
            if (modulationPhases[i] >= 1.0f) modulationPhases[i] -= 1.0f;

            float baseDelayLeft = shortDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft + stereoOffset;

            float modulatedDelayLeft = baseDelayLeft + (std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]) * shortModulationDepth * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i] + 0.25f) * shortModulationDepth * baseDelayRight);

            shortDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            shortDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            // Apply all-pass filtering to short delays for diffusion
            shortDelayOutputLeft[i] = allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
            shortDelayOutputRight[i] = allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);
        }

        // Apply Hadamard matrix for short delays
        std::array<float, 4> shortHadamardLeft, shortHadamardRight;
        for (int i = 0; i < 4; ++i)
        {
            shortHadamardLeft[i] = 0.0f;
            shortHadamardRight[i] = 0.0f;
            for (int j = 0; j < 4; ++j)
            {
                shortHadamardLeft[i] += hadamardMatrix[i][j] * shortDelayOutputLeft[j];
                shortHadamardRight[i] += hadamardMatrix[i][j] * shortDelayOutputRight[j];
            }
        }

        // Process long delays with bloom effect, subdivisions, and stronger modulation
        for (int i = 0; i < 4; ++i)
        {
            modulationPhases[i + 4] += modulationFrequencies[i + 4] * 2.0f / sampleRate;
            if (modulationPhases[i + 4] >= 1.0f) modulationPhases[i + 4] -= 1.0f;

            float baseDelayLeft = longDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft - stereoOffset;

            float modulationLeft = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i + 4]) * longModulationDepth;
            float modulationRight = std::sin(2.0f * juce::MathConstants<float>::pi * (modulationPhases[i + 4] + 0.5f)) * longModulationDepth;

            float modulatedDelayLeft = baseDelayLeft + (modulationLeft * baseDelayLeft);
            float modulatedDelayRight = baseDelayRight + (modulationRight * baseDelayRight);

            longDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate);
            longDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate);

            // Introduce smaller and larger subdivisions for blooming effect
            float bloomLeftSmall = getInterpolatedSample((modulatedDelayLeft * longSubdivisionsFactor) / sampleRate);
            float bloomRightSmall = getInterpolatedSample((modulatedDelayRight * longSubdivisionsFactor) / sampleRate);
            
            float bloomLeftLarge = getInterpolatedSample((modulatedDelayLeft / longSubdivisionsFactor) / sampleRate);
            float bloomRightLarge = getInterpolatedSample((modulatedDelayRight / longSubdivisionsFactor) / sampleRate);

            longDelayOutputLeft[i + 4] += bloomLeftSmall * bloomFeedbackGain + bloomLeftLarge * bloomFeedbackGain;
            longDelayOutputRight[i + 4] += bloomRightSmall * bloomFeedbackGain + bloomRightLarge * bloomFeedbackGain;

            // Apply all-pass filtering with strong modulation to long delays
            longDelayOutputLeft[i] = allPassFiltersLong[i].processSample(longDelayOutputLeft[i]);
            longDelayOutputRight[i] = allPassFiltersLong[i].processSample(longDelayOutputRight[i]);
        }

        // Apply Hadamard matrix to long delays
        std::array<float, 8> longHadamardLeft, longHadamardRight;
        for (int i = 0; i < 8; ++i)
        {
            longHadamardLeft[i] = 0.0f;
            longHadamardRight[i] = 0.0f;
            for (int j = 0; j < 8; ++j)
            {
                longHadamardLeft[i] += hadamardMatrix[i % 4][j % 4] * longDelayOutputLeft[j];
                longHadamardRight[i] += hadamardMatrix[i % 4][j % 4] * longDelayOutputRight[j];
            }
        }

        // Final wet signal from short and long delays
        float wetSignalLeft = (shortHadamardLeft[0] + shortHadamardLeft[1] + shortHadamardLeft[2] + shortHadamardLeft[3]) * 0.25f
                            + (longHadamardLeft[0] + longHadamardLeft[1] + longHadamardLeft[2] + longHadamardLeft[3]
                            + longHadamardLeft[4] + longHadamardLeft[5] + longHadamardLeft[6] + longHadamardLeft[7]) * 0.125f;
                            
        float wetSignalRight = (shortHadamardRight[0] + shortHadamardRight[1] + shortHadamardRight[2] + shortHadamardRight[3]) * 0.25f
                             + (longHadamardRight[0] + longHadamardRight[1] + longHadamardRight[2] + longHadamardRight[3]
                             + longHadamardRight[4] + longHadamardRight[5] + longHadamardRight[6] + longHadamardRight[7]) * 0.125f;

        // Apply dry/wet mix and output gain
        float outputSampleLeft = inputSampleLeft * dryMix + wetSignalLeft * wetMix;
        float outputSampleRight = inputSampleRight * dryMix + wetSignalRight * wetMix;

        // Soft clipping and final gain
        outputSampleLeft = softClip(applyGain(outputSampleLeft, outputGain));
        outputSampleRight = softClip(applyGain(outputSampleRight, outputGain));

        // Store for feedback
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
