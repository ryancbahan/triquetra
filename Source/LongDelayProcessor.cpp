#include "LongDelayProcessor.h"
#include <cmath>
#include <random>

LongDelayProcessor::LongDelayProcessor()
{
    // Initialize modulationPhaseOffsets with staggered values
    modulationPhaseOffsets = { 0.0f, 0.25f, 0.5f, 0.75f };  // Different offsets for each delay line

    // Initialize the modulationPhase array
    modulationPhase.fill(0.0f);  // Start with all phases at 0
    isDelayLineModulated = { true, false, true, false };

    // Constructor for irregular delay factors
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.9, 1.1);
    
    lowPassFilterLeft.setType(juce::dsp::StateVariableTPTFilterType::lowpass);
    lowPassFilterRight.setType(juce::dsp::StateVariableTPTFilterType::lowpass);
    currentCutoffFreq = 15000.0f;
    
    for (int i = 0; i < 4; ++i)
    {
        // Initialize the oscillator with a sine wave
        lfoOscillators[i].initialise([](float x) { return std::sin(x); }, 128);

        // Assign different frequencies to each modulated delay line
        if (isDelayLineModulated[i])
        {
            // Frequencies in Hz for each modulated delay line
            float frequencies[] = { 0.1f, 0.0f, 0.15f, 0.0f }; // Zero frequency for unmodulated lines
            lfoOscillators[i].setFrequency(frequencies[i]);
        }
    }
}

void LongDelayProcessor::reset()
{
    // Clear the delay buffers
    for (auto& buffer : delayBufferLeft)
        std::fill(buffer.begin(), buffer.end(), 0.0f);

    for (auto& buffer : delayBufferRight)
        std::fill(buffer.begin(), buffer.end(), 0.0f);

    // Reset the write position to the start
    writePosition = 0;

    // Reset the all-pass filters to their initial state
    for (auto& filter : allPassFiltersLong)
        filter.reset();

    // Reset the envelope followers
    envelopeFollowerLeft.reset();
    envelopeFollowerRight.reset();
}

void LongDelayProcessor::prepare(double newSampleRate, int numChannels, float newFeedback, float newBloomFeedbackGain,
                                 float newModulationFeedbackAmount, float newAttenuationFactor, float newLongSubdivisionsFactor,
                                 float newDecayRate)
{
    sampleRate = newSampleRate;
    feedback = newFeedback;
    bloomFeedbackGain = newBloomFeedbackGain;
    modulationFeedbackAmount = newModulationFeedbackAmount;
    attenuationFactor = newAttenuationFactor;
    longSubdivisionsFactor = newLongSubdivisionsFactor;
    decayRate = newDecayRate;

    juce::dsp::ProcessSpec spec{ sampleRate, static_cast<juce::uint32>(512), static_cast<juce::uint32>(numChannels) };

    for (auto& filter : allPassFiltersLong)
    {
        filter.reset();
        filter.prepare(spec);
    }
    
    for (int i = 0; i < 4; ++i)
    {
        if (isDelayLineModulated[i])
        {
            lfoOscillators[i].prepare(spec);
            lfoOscillators[i].reset();
        }
    }

    delayBufferSize = static_cast<int>(sampleRate * 4.0);

    // Resize each vector in the arrays individually
    for (auto& buffer : delayBufferLeft)
    {
        buffer.resize(delayBufferSize, 0.0f);
    }

    for (auto& buffer : delayBufferRight)
    {
        buffer.resize(delayBufferSize, 0.0f);
    }
    
    lowPassFilterLeft.prepare(spec);
    lowPassFilterRight.prepare(spec);

    writePosition = 0;

    // Prepare the envelope followers for input samples
    envelopeFollowerLeft.prepareToPlay(sampleRate, 1);
    envelopeFollowerLeft.reset();
    envelopeFollowerLeft.setAttackTime(0.0f);              // Set desired attack time
    envelopeFollowerLeft.setAmplitudeJumpThreshold(0.015f); // Adjust as needed
    envelopeFollowerLeft.setNoiseGateThreshold(0.01f);     // Adjust as needed

    envelopeFollowerRight.prepareToPlay(sampleRate, 1);
    envelopeFollowerRight.reset();
    envelopeFollowerRight.setAttackTime(0.0f);              // Set desired attack time
    envelopeFollowerRight.setAmplitudeJumpThreshold(0.015f); // Adjust as needed
    envelopeFollowerRight.setNoiseGateThreshold(0.01f);     // Adjust as needed
}

void LongDelayProcessor::process(
    const std::array<float, 4>& longDelayTimes,
    std::array<float, 8>& longFeedbackLeft,
    std::array<float, 8>& longFeedbackRight,
    float modulationDepth, // Modulation depth as a percentage (e.g., 0.0f to 0.05f)
    float stereoOffset,
    std::array<float, 8>& longHadamardLeft,
    std::array<float, 8>& longHadamardRight,
    float inputSampleLeft,
    float inputSampleRight,
    float currentFeedback,
    float smearValue,
    float dampValue, float spreadValue)
{
    // Update smear value if it has changed
    if (currentSmearValue != smearValue)
    {
        envelopeFollowerLeft.setAttackTime(smearValue);
        envelopeFollowerRight.setAttackTime(smearValue);
        currentSmearValue = smearValue;
    }

    // Process input samples through envelope followers if smear is active
    if (smearValue > 0.0f)
    {
        inputSampleLeft = envelopeFollowerLeft.processSample(0, inputSampleLeft);
        inputSampleRight = envelopeFollowerRight.processSample(0, inputSampleRight);
    }

    // Apply attenuation to the input samples
    const float attenuationFactor = 1.0f / std::sqrt(4.0f); // Adjust based on the number of delay lines
    float attenuatedInputLeft = inputSampleLeft * attenuationFactor;
    float attenuatedInputRight = inputSampleRight * attenuationFactor;

    // Damping using a low-pass filter
    const float maxCutoffFreq = 15000.0f; // Maximum cutoff frequency (minimal damping)
    const float minCutoffFreq = 250.0f;   // Minimum cutoff frequency (maximum damping)
    float targetCutoffFreq = juce::jmap(dampValue, 0.0f, 1.0f, maxCutoffFreq, minCutoffFreq);

    // Smoothly adjust the current cutoff frequency towards the target
    currentCutoffFreq += (targetCutoffFreq - currentCutoffFreq) * 0.001f;
    lowPassFilterLeft.setCutoffFrequency(currentCutoffFreq);
    lowPassFilterRight.setCutoffFrequency(currentCutoffFreq);

    // Apply the low-pass filter to the attenuated input samples
    attenuatedInputLeft = lowPassFilterLeft.processSample(0, attenuatedInputLeft);
    attenuatedInputRight = lowPassFilterRight.processSample(1, attenuatedInputRight);

    // Clear the Hadamard arrays for the current processing block
    std::fill(longHadamardLeft.begin(), longHadamardLeft.end(), 0.0f);
    std::fill(longHadamardRight.begin(), longHadamardRight.end(), 0.0f);

    float cumulativeIrregularDelayLeft = 0.0f;
    float cumulativeIrregularDelayRight = 0.0f;

    // Process each delay line
    for (int i = 0; i < 4; ++i)
    {
        // Base delay times in samples
        float baseDelaySamplesLeft = longDelayTimes[i] * static_cast<float>(sampleRate);
        float baseDelaySamplesRight = baseDelaySamplesLeft - stereoOffset;

        // Ensure delay times are positive
        baseDelaySamplesLeft = std::max(0.0f, baseDelaySamplesLeft);
        baseDelaySamplesRight = std::max(0.0f, baseDelaySamplesRight);

        // Calculate maximum modulation depth in samples (as a percentage of base delay time)
        float maxModulationSamples = baseDelaySamplesLeft * modulationDepth;

        float modulatedDelayLeft = baseDelaySamplesLeft;
        float modulatedDelayRight = baseDelaySamplesRight;

        // Check if this delay line should be modulated
        if (isDelayLineModulated[i])
        {
            // Get the next LFO sample (ranges from -1 to 1)
            float lfoSample = lfoOscillators[i].processSample(0.0f);

            // Scale the LFO sample to the modulation depth
            float modulationAmount = lfoSample * maxModulationSamples;

            // Apply modulation to delay times
            modulatedDelayLeft += modulationAmount;
            modulatedDelayRight += modulationAmount;
        }

        // Ensure modulated delay times are within buffer bounds
        modulatedDelayLeft = juce::jlimit(0.0f, static_cast<float>(delayBufferSize - 1), modulatedDelayLeft);
        modulatedDelayRight = juce::jlimit(0.0f, static_cast<float>(delayBufferSize - 1), modulatedDelayRight);

        // Retrieve interpolated samples from the delay buffers
        float delayedSampleLeft = getInterpolatedSample(delayBufferLeft[i], modulatedDelayLeft);
        float delayedSampleRight = getInterpolatedSample(delayBufferRight[i], modulatedDelayRight);

        // Store the delayed samples in the Hadamard arrays
        longHadamardLeft[i] = delayedSampleLeft;
        longHadamardRight[i] = delayedSampleRight;

        // --- Irregular Delay Behavior ---

        // Calculate irregular delay
        float irregularFactor = irregularityFactors[i];

        float irregularDelayLeft = baseDelaySamplesLeft * (1.0f + spreadValue * (irregularFactor - 1.0f));
        float irregularDelayRight = baseDelaySamplesRight * (1.0f + spreadValue * (irregularFactor - 1.0f));

        // Alternate between shorter and longer irregular delays
        if (i % 2 == 0)
        {
            irregularDelayLeft /= longSubdivisionsFactor;
            irregularDelayRight /= longSubdivisionsFactor;
        }
        else
        {
            irregularDelayLeft *= longSubdivisionsFactor;
            irregularDelayRight *= longSubdivisionsFactor;
        }

        // Add to cumulative irregular delay and clamp to avoid overflow
        cumulativeIrregularDelayLeft += irregularDelayLeft;
        cumulativeIrregularDelayRight += irregularDelayRight;

        cumulativeIrregularDelayLeft = juce::jlimit(0.0f, static_cast<float>(delayBufferSize - 1), cumulativeIrregularDelayLeft);
        cumulativeIrregularDelayRight = juce::jlimit(0.0f, static_cast<float>(delayBufferSize - 1), cumulativeIrregularDelayRight);

        // Get samples for irregular delays
        float irregularSampleLeft = getInterpolatedSample(delayBufferLeft[i], cumulativeIrregularDelayLeft);
        float irregularSampleRight = getInterpolatedSample(delayBufferRight[i], cumulativeIrregularDelayRight);

        // Mix irregular delays into Hadamard arrays
        longHadamardLeft[i + 4] = irregularSampleLeft * bloomFeedbackGain;
        longHadamardRight[i + 4] = irregularSampleRight * bloomFeedbackGain;

        // --- End of Irregular Delay Behavior ---

        // Apply all-pass filters for diffusion
        longHadamardLeft[i] = allPassFiltersLong[i].processSample(longHadamardLeft[i]);
        longHadamardRight[i] = allPassFiltersLong[i].processSample(longHadamardRight[i]);

        // Apply attenuation to the filtered samples
        longHadamardLeft[i] *= attenuationFactor;
        longHadamardRight[i] *= attenuationFactor;

        // Calculate feedback signals, combining original and irregular delayed samples
        float combinedFeedbackLeft = longHadamardLeft[i] + longHadamardLeft[i + 4];
        float combinedFeedbackRight = longHadamardRight[i] + longHadamardRight[i + 4];

        // Apply feedback gain and limit to prevent excessive buildup
        longFeedbackLeft[i] = juce::jlimit(-0.95f, 0.95f, combinedFeedbackLeft * currentFeedback);
        longFeedbackRight[i] = juce::jlimit(-0.95f, 0.95f, combinedFeedbackRight * currentFeedback);

        // Write the input sample plus feedback into the delay buffers
        delayBufferLeft[i][writePosition] = attenuatedInputLeft + longFeedbackLeft[i];
        delayBufferRight[i][writePosition] = attenuatedInputRight + longFeedbackRight[i];
    }

    // Increment the write position for the delay buffers, wrapping around if necessary
    writePosition = (writePosition + 1) % delayBufferSize;
}

float LongDelayProcessor::getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples)
{
    int readPosition = (writePosition - static_cast<int>(delayInSamples) + delayBufferSize) % delayBufferSize;
    float fraction = delayInSamples - static_cast<float>(static_cast<int>(delayInSamples));

    int nextPosition = (readPosition + 1) % delayBufferSize;

    float currentSample = buffer[readPosition];
    float nextSample = buffer[nextPosition];

    return currentSample + fraction * (nextSample - currentSample);
}
