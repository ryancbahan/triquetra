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
    
    reverbProcessor.prepare(sampleRate, samplesPerBlock);
    
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
    
    // Band-pass filter (centered around 1kHz with a bandwidth of 1 octave)
    auto bandPassCoefficients = juce::dsp::IIR::Coefficients<float>::makeBandPass(sampleRate, 1000.0f, 1.0f);  // Adjust freq and Q
    bandPassFilterLeft.coefficients = bandPassCoefficients;
    bandPassFilterRight.coefficients = bandPassCoefficients;

    bandPassFilterLeft.prepare(spec);
    bandPassFilterRight.prepare(spec);
    
    juce::dsp::IIR::Coefficients<float>::Ptr lowpassCoefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);
    
    reverbWashLowpassFilterLeft.coefficients = lowpassCoefficients;
    reverbWashLowpassFilterRight.coefficients = lowpassCoefficients;

    reverbWashLowpassFilterLeft.prepare(spec);
    reverbWashLowpassFilterRight.prepare(spec);
      
      lowpassFilterLeft.coefficients = lowpassCoefficients;
      lowpassFilterRight.coefficients = lowpassCoefficients;

      lowpassFilterLeft.prepare(spec);
      lowpassFilterRight.prepare(spec);
    reverbWashHighpassFilterLeft.prepare(spec);
      reverbWashHighpassFilterRight.prepare(spec);

      // Initialize the high-pass filter with a cutoff of 150Hz
      reverbWashHighpassFilterLeft.coefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass(sampleRate, 150.0f);
      reverbWashHighpassFilterRight.coefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass(sampleRate, 150.0f);
      
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

    // Parameters
    const float bloomDelayModDepth = 0.1f;
    const float bloomFeedbackGain = 0.75f;
    const float longSubdivisionsFactor = 1.3f;
    const float modulationFeedbackAmount = 0.2f;
    const float crossFeedbackAmount = 0.1f;
    const float attenuationFactor = 0.35f;
    const float diffusionAmount = 0.7f;
    const float bloomRegenerationGain = 0.8f;
    const float bloomModulationAmount = 0.5f;

    const float reverbWashFeedbackGain = 0.5f;
    const float reverbWashLowpassFreq = 10000.0f;
    const float reverbWashModulationFreq = 0.1f;
    const float reverbWashModulationDepth = 0.1f;
    const float reverbWashMixAmount = 0.9f;

    // Pre-delay setup (30ms)
    const float reverbPreDelayTime = 0.250f; // 30ms pre-delay
    int preDelaySamples = static_cast<int>(reverbPreDelayTime * sampleRate);
    static std::vector<float> preDelayBufferLeft(preDelaySamples, 0.0f);
    static std::vector<float> preDelayBufferRight(preDelaySamples, 0.0f);
    static int preDelayWritePos = 0;

    // Modulation parameters
    static float modulationPhase = 0.0f;
    const float modulationFrequency = 0.05f;
    const float modulationDepth = 0.005f;

    // Variables for DC Offset Removal
    static float previousInputLeft = 0.0f, previousOutputLeft = 0.0f;
    static float previousInputRight = 0.0f, previousOutputRight = 0.0f;

    // Feedback buffers
    static std::array<float, 4> shortFeedbackLeft = {0.0f, 0.0f, 0.0f, 0.0f};
    static std::array<float, 4> shortFeedbackRight = {0.0f, 0.0f, 0.0f, 0.0f};
    static std::array<float, 8> longFeedbackLeft = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static std::array<float, 8> longFeedbackRight = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // New reverb wash buffers
    static std::array<float, 8> reverbWashLeft = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static std::array<float, 8> reverbWashRight = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static float reverbWashPhase = 0.0f;

    // New cross-feedback and modulation buffers for extended diffusion
    static std::array<float, 4> crossFeedbackLeft = {0.0f, 0.0f, 0.0f, 0.0f};
    static std::array<float, 4> crossFeedbackRight = {0.0f, 0.0f, 0.0f, 0.0f};

    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
    {
        float inputSampleLeft = buffer.getSample(0, sample);
        float inputSampleRight = totalNumInputChannels > 1 ? buffer.getSample(1, sample) : inputSampleLeft;

        // Apply DC offset removal
        float processedInputLeft = removeDCOffset(inputSampleLeft, previousInputLeft, previousOutputLeft);
        float processedInputRight = removeDCOffset(inputSampleRight, previousInputRight, previousOutputRight);

        processedInputLeft = applyGain(processedInputLeft, inputGain);
        processedInputRight = applyGain(processedInputRight, inputGain);

        // Pre-delay handling
        float delayedInputLeft = preDelayBufferLeft[preDelayWritePos];
        float delayedInputRight = preDelayBufferRight[preDelayWritePos];

        // Write current sample into the pre-delay buffer
        preDelayBufferLeft[preDelayWritePos] = processedInputLeft;
        preDelayBufferRight[preDelayWritePos] = processedInputRight;

        // Update pre-delay write position
        preDelayWritePos = (preDelayWritePos + 1) % preDelaySamples;

        std::array<float, 4> shortDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 4> shortDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> longDelayOutputLeft = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 8> longDelayOutputRight = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Update modulation
        modulationPhase += modulationFrequency / sampleRate;
        if (modulationPhase >= 1.0f) modulationPhase -= 1.0f;
        float modulationValue = std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhase) * modulationDepth;

        // Process short delays with feedback and modulation
        for (int i = 0; i < 4; ++i)
        {
            float baseDelayLeft = shortDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft + stereoOffset;

            float modulatedDelayLeft = baseDelayLeft * (1.0f + modulationValue);
            float modulatedDelayRight = baseDelayRight * (1.0f + modulationValue);

            shortDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate) + shortFeedbackLeft[i] * feedback;
            shortDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate) + shortFeedbackRight[i] * feedback;

            // Apply all-pass filtering and update feedback
            shortDelayOutputLeft[i] = allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
            shortDelayOutputRight[i] = allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);

            // Diffusion from additional all-pass filters
            shortDelayOutputLeft[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
            shortDelayOutputRight[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);
            
            shortDelayOutputLeft[i] = reverbWashLowpassFilterLeft.processSample(shortDelayOutputLeft[i]);
            shortDelayOutputRight[i] = reverbWashLowpassFilterRight.processSample(shortDelayOutputRight[i]);

            shortFeedbackLeft[i] = shortDelayOutputLeft[i] * modulationFeedbackAmount;
            shortFeedbackRight[i] = shortDelayOutputRight[i] * modulationFeedbackAmount;
        }

        // Apply Hadamard matrix for short delays
        std::array<float, 4> shortHadamardLeft = applyHadamardMixing(shortDelayOutputLeft);
        std::array<float, 4> shortHadamardRight = applyHadamardMixing(shortDelayOutputRight);

        // Process long delays with bloom effect, subdivisions, and modulation
        for (int i = 0; i < 4; ++i)
        {
            float baseDelayLeft = longDelayTimes[i] * sampleRate;
            float baseDelayRight = baseDelayLeft - stereoOffset;

            float modulatedDelayLeft = baseDelayLeft * (1.0f + modulationValue);
            float modulatedDelayRight = baseDelayRight * (1.0f + modulationValue);

            longDelayOutputLeft[i] = getInterpolatedSample(modulatedDelayLeft / sampleRate) + longFeedbackLeft[i] * longFeedback;
            longDelayOutputRight[i] = getInterpolatedSample(modulatedDelayRight / sampleRate) + longFeedbackRight[i] * longFeedback;

            // Introduce smaller and larger subdivisions for blooming effect
            float bloomLeftSmall = getInterpolatedSample((modulatedDelayLeft * longSubdivisionsFactor) / sampleRate);
            float bloomRightSmall = getInterpolatedSample((modulatedDelayRight * longSubdivisionsFactor) / sampleRate);

            float bloomLeftLarge = getInterpolatedSample((modulatedDelayLeft / longSubdivisionsFactor) / sampleRate);
            float bloomRightLarge = getInterpolatedSample((modulatedDelayRight / longSubdivisionsFactor) / sampleRate);

            longDelayOutputLeft[i + 4] = bloomLeftSmall * bloomFeedbackGain + bloomLeftLarge * bloomFeedbackGain + longFeedbackLeft[i + 4] * longFeedback;
            longDelayOutputRight[i + 4] = bloomRightSmall * bloomFeedbackGain + bloomRightLarge * bloomFeedbackGain + longFeedbackRight[i + 4] * longFeedback;

            // Apply all-pass filtering and update feedback
            longDelayOutputLeft[i] = allPassFiltersLong[i].processSample(longDelayOutputLeft[i]);
            longDelayOutputRight[i] = allPassFiltersLong[i].processSample(longDelayOutputRight[i]);

            // Apply attenuation to the long delays for further diffusion
            longDelayOutputLeft[i] *= attenuationFactor;
            longDelayOutputRight[i] *= attenuationFactor;
            
            longDelayOutputLeft[i] = reverbWashLowpassFilterLeft.processSample(longDelayOutputLeft[i]);
            longDelayOutputRight[i] = reverbWashLowpassFilterRight.processSample(longDelayOutputRight[i]);

            longFeedbackLeft[i] = longDelayOutputLeft[i] * modulationFeedbackAmount;
            longFeedbackRight[i] = longDelayOutputRight[i] * modulationFeedbackAmount;
        }

        // Apply Hadamard matrix to long delays
        std::array<float, 8> longHadamardLeft = applyHadamardMixing(longDelayOutputLeft);
        std::array<float, 8> longHadamardRight = applyHadamardMixing(longDelayOutputRight);

        // Controlled cross-feedback between channels (for added stereo width and depth)
        for (int i = 0; i < 4; ++i)
        {
            crossFeedbackLeft[i] = longHadamardRight[i] * crossFeedbackAmount;
            crossFeedbackRight[i] = longHadamardLeft[i] * crossFeedbackAmount;

            // Feed cross-feedback back into the main feedback buffers
            longFeedbackLeft[i] += crossFeedbackLeft[i];
            longFeedbackRight[i] += crossFeedbackRight[i];
        }

        // Regenerative bloom: Introduce feedback from long delays and reverb wash back into short delays with modulation and gain
        for (int i = 0; i < 4; ++i)
        {
            float combinedBloomLeft = (longHadamardLeft[i + 4]) * bloomRegenerationGain
                                      * (1.0f + std::sin(juce::MathConstants<float>::twoPi * modulationPhases[i]) * bloomModulationAmount);
            float combinedBloomRight = (longHadamardRight[i + 4]) * bloomRegenerationGain
                                       * (1.0f + std::sin(juce::MathConstants<float>::twoPi * modulationPhases[i]) * bloomModulationAmount);

            combinedBloomLeft = highpassFilter(combinedBloomLeft, 150.0f, sampleRate);
            combinedBloomRight = highpassFilter(combinedBloomRight, 150.0f, sampleRate);

            // Feed the combined signal back into the short delays
            shortFeedbackLeft[i] += combinedBloomLeft;
            shortFeedbackRight[i] += combinedBloomRight;

            modulationPhases[i] += phaseIncrements[i];
            if (modulationPhases[i] >= 1.0f)
                modulationPhases[i] -= 1.0f;
        }
        
        float reverbWashOutputLeft = 0.0f;
        float reverbWashOutputRight = 0.0f;


        reverbProcessor.process(shortHadamardLeft, shortHadamardRight,
                                longHadamardLeft, longHadamardRight,
                                reverbWashOutputLeft, reverbWashOutputRight);
        
//        // Mix reverb wash back into main signal
//        for (int i = 0; i < 8; ++i)
//        {
//            reverbWashOutputLeft += reverbWashLeft[i];
//            reverbWashOutputRight += reverbWashRight[i];
//        }
//        reverbWashOutputLeft *= 0.5f * reverbWashMixAmount;
//        reverbWashOutputRight *= 0.5f * reverbWashMixAmount;
//
//        currentFeedbackLevel = 0.0f;
//        for (int i = 0; i < 8; ++i)
//        {
//            currentFeedbackLevel += std::abs(reverbWashLeft[i]) + std::abs(reverbWashRight[i]);
//        }
//        currentFeedbackLevel /= 16.0f;
//
//        float feedbackScaleFactor = 1.0f / (1.0f + currentFeedbackLevel);
//        for (int i = 0; i < 8; ++i)
//        {
//            reverbWashLeft[i] *= feedbackScaleFactor;
//            reverbWashRight[i] *= feedbackScaleFactor;
//        }
        
//        // Drastically intermix long Hadamard and reverb wash outputs with delayed cross-feedback for extended decay
//        for (int i = 0; i < 8; ++i)
//        {
//            // Inputs for blending: long Hadamard output and reverb wash output
//            float longHadamardLeftOutput = longHadamardLeft[i];
//            float longHadamardRightOutput = longHadamardRight[i];
//            float reverbWashLeftOutput = reverbWashLeft[i];
//            float reverbWashRightOutput = reverbWashRight[i];
//
//            // Drastically blend the long Hadamard and reverb wash outputs
//            float blendedLeft = longHadamardLeftOutput * 0.6f + reverbWashLeftOutput * 0.9f;
//            float blendedRight = longHadamardRightOutput * 0.6f + reverbWashRightOutput * 0.9f;
//
//            // Introduce delayed cross-feedback for extended decay
//            static std::array<float, 8> crossFeedbackLeftBuffer = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//            static std::array<float, 8> crossFeedbackRightBuffer = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//
//            float delayedCrossFeedbackLeft = crossFeedbackLeftBuffer[i] * 0.5f;  // Cross-feedback applied with delay
//            float delayedCrossFeedbackRight = crossFeedbackRightBuffer[i] * 0.5f;
//
//            // Store the current cross-feedback for use in future iterations
//            crossFeedbackLeftBuffer[i] = reverbWashRight[i] * 0.3f;  // Capture current reverb output for future feedback
//            crossFeedbackRightBuffer[i] = reverbWashLeft[i] * 0.3f;
//
//            // Add the delayed cross-feedback to the blended signal
//            blendedLeft += delayedCrossFeedbackLeft * (1.0f + std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]) * bloomModulationAmount * 1.5f);
//            blendedRight += delayedCrossFeedbackRight * (1.0f + std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i]) * bloomModulationAmount * 1.5f);
//
//            // Apply the blend back into the reverb wash and Hadamard paths with heavier feedback
//            reverbWashLeft[i] = blendedLeft * 0.7f + reverbWashLeft[i] * 0.3f;  // Gradual blending into reverb wash
//            reverbWashRight[i] = blendedRight * 0.7f + reverbWashRight[i] * 0.3f;
//
//            longHadamardLeft[i] = blendedLeft * 0.7f + longHadamardLeft[i] * 0.3f;  // Gradual blending into Hadamard
//            longHadamardRight[i] = blendedRight * 0.7f + longHadamardRight[i] * 0.3f;
//
//            // Increment modulation phase for the next sample
//            modulationPhases[i] += phaseIncrements[i];
//            if (modulationPhases[i] >= 1.0f)
//                modulationPhases[i] -= 1.0f;
//        }


        // Final wet signal
        // Uncomment below to include short/long delays in addition to the reverb wash
        
//        float wetSignalLeft = (shortHadamardLeft[0] + shortHadamardLeft[1] + shortHadamardLeft[2] + shortHadamardLeft[3]) * 0.25f
//                            + (longHadamardLeft[0] + longHadamardLeft[1] + longHadamardLeft[2] + longHadamardLeft[3]
//                            + longHadamardLeft[4] + longHadamardLeft[5] + longHadamardLeft[6] + longHadamardLeft[7]) * 0.125f
//                            + reverbWashOutputLeft;
//        float wetSignalRight = (shortHadamardRight[0] + shortHadamardRight[1] + shortHadamardRight[2] + shortHadamardRight[3]) * 0.25f
//                                     + (longHadamardRight[0] + longHadamardRight[1] + longHadamardRight[2] + longHadamardRight[3]
//                                     + longHadamardRight[4] + longHadamardRight[5] + longHadamardRight[6] + longHadamardRight[7]) * 0.125f
//                                     + reverbWashOutputRight;
        

//        // Isolate reverb wash for testing
        float wetSignalLeft = reverbWashOutputLeft;
        float wetSignalRight = reverbWashOutputRight;

        // Apply dry/wet mix and output gain
        float outputSampleLeft = inputSampleLeft * dryMix + wetSignalLeft * wetMix;
        float outputSampleRight = inputSampleRight * dryMix + wetSignalRight * wetMix;

        // Soft clipping and final gain
        outputSampleLeft = softClip(applyGain(outputSampleLeft, outputGain));
        outputSampleRight = softClip(applyGain(outputSampleRight, outputGain));

        // Additional safety clipping
        outputSampleLeft = juce::jlimit(-1.0f, 1.0f, outputSampleLeft);
        outputSampleRight = juce::jlimit(-1.0f, 1.0f, outputSampleRight);

        // Write final output to buffer
        buffer.setSample(0, sample, wetSignalLeft);
        buffer.setSample(1, sample, wetSignalRight);

        // Update delay buffer for feedback
        delayBuffer[writePosition] = (outputSampleLeft + outputSampleRight) * 0.5f;
        writePosition = (writePosition + 1) % delayBufferSize;
    }

    // Clear any denormals in the feedback buffers
    for (int i = 0; i < 4; ++i)
    {
        shortFeedbackLeft[i] = clearDenormals(shortFeedbackLeft[i]);
        shortFeedbackRight[i] = clearDenormals(shortFeedbackRight[i]);
    }
    for (int i = 0; i < 8; ++i)
    {
        longFeedbackLeft[i] = clearDenormals(longFeedbackLeft[i]);
        longFeedbackRight[i] = clearDenormals(longFeedbackRight[i]);
        reverbWashLeft[i] = clearDenormals(reverbWashLeft[i]);
        reverbWashRight[i] = clearDenormals(reverbWashRight[i]);
    }
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
