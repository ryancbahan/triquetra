#include "LongDelayProcessor.h"
#include <cmath>
#include <random>

LongDelayProcessor::LongDelayProcessor()
{
    // Initialize modulationPhaseOffsets with staggered values
    modulationPhaseOffsets = { 0.0f, 0.25f, 0.5f, 0.75f };  // Different offsets for each delay line

    // Initialize the modulationPhase array
    modulationPhase.fill(0.0f);  // Start with all phases at 0

    // Constructor for irregular delay factors
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.9, 1.1);
    
    // Initialize irregular delay factors
    for (auto& factor : irregularDelayFactors) {
        factor = static_cast<float>(dis(gen));
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
}

void LongDelayProcessor::prepare(double newSampleRate, int numChannels, float newFeedback, float newBloomFeedbackGain, float newModulationFeedbackAmount, float newAttenuationFactor, float newLongSubdivisionsFactor, float newDecayRate)
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

    delayBufferSize = static_cast<int>(sampleRate * 4.0);
    delayBufferLeft.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    delayBufferRight.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    
    envelopeFollower.prepareToPlay(sampleRate, 1);
    envelopeFollower.reset();
    envelopeFollower.setAttackTime(5.0f);
    envelopeFollower.setAmplitudeJumpThreshold(0.05f);
    envelopeFollower.setNoiseGateThreshold(0.01f);

    writePosition = 0;
}

void LongDelayProcessor::process(const std::array<float, 4>& longDelayTimes,
                                 std::array<float, 8>& longFeedbackLeft,
                                 std::array<float, 8>& longFeedbackRight,
                                 float modulationValue, float stereoOffset,
                                 std::array<float, 8>& longHadamardLeft,
                                 std::array<float, 8>& longHadamardRight,
                                 float inputSampleLeft, float inputSampleRight,
                                 float currentFeedback)
{
    const float attenuationFactor = 1.0f / sqrt(4.0f);

    float attenuatedInputLeft = inputSampleLeft * attenuationFactor;
    float attenuatedInputRight = inputSampleRight * attenuationFactor;

    std::fill(longHadamardLeft.begin(), longHadamardLeft.end(), 0.0f);
    std::fill(longHadamardRight.begin(), longHadamardRight.end(), 0.0f);

    float cumulativeIrregularDelayLeft = 0.0f;
    float cumulativeIrregularDelayRight = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        // Increment the modulation phase for each delay line
        modulationPhase[i] += modulationFrequency / sampleRate;
        if (modulationPhase[i] >= 1.0f)
            modulationPhase[i] -= 1.0f;

        // Calculate the final modulation phase, adding staggered offsets
        float finalModulationPhase = modulationPhase[i] + modulationPhaseOffsets[i];
        if (finalModulationPhase >= 1.0f)
            finalModulationPhase -= 1.0f;

        // Calculate the modulated delay time using the staggered modulation
        float modulatedDelay = std::sin(2.0f * juce::MathConstants<float>::pi * finalModulationPhase) * modulationValue;

        float baseDelayLeft = std::max(0.0f, longDelayTimes[i] * static_cast<float>(sampleRate));
        float baseDelayRight = std::max(0.0f, baseDelayLeft - stereoOffset);

        // Modulate delay times and clamp them to prevent overflow
        float originalDelayLeft = juce::jlimit(0.0f, static_cast<float>(delayBufferSize - 1), baseDelayLeft * (1.0f + modulatedDelay));
        float originalDelayRight = juce::jlimit(0.0f, static_cast<float>(delayBufferSize - 1), baseDelayRight * (1.0f + modulatedDelay));

        // Get samples for original delays
        longHadamardLeft[i] = envelopeFollower.processSample(0, getInterpolatedSample(delayBufferLeft[i], originalDelayLeft));
        longHadamardRight[i] = envelopeFollower.processSample(1, getInterpolatedSample(delayBufferRight[i], originalDelayRight));

        // Calculate irregular delay
        float irregularDelayLeft = baseDelayLeft * irregularDelayFactors[i];
        float irregularDelayRight = baseDelayRight * irregularDelayFactors[i];

        // Alternate between shorter and longer irregular delays
        if (i % 2 == 0) {
            irregularDelayLeft /= longSubdivisionsFactor;
            irregularDelayRight /= longSubdivisionsFactor;
        } else {
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

        // Apply decay to both original and irregular delays
        float decayFactor = std::pow(decayRate, static_cast<float>(i));
        longHadamardLeft[i] *= decayFactor;
        longHadamardRight[i] *= decayFactor;
        irregularSampleLeft *= decayFactor;
        irregularSampleRight *= decayFactor;

        // Mix irregular delays into Hadamard arrays
        longHadamardLeft[i + 4] = envelopeFollower.processSample(0, irregularSampleLeft * bloomFeedbackGain);
        longHadamardRight[i + 4] = envelopeFollower.processSample(1, irregularSampleRight * bloomFeedbackGain);

        // Apply all-pass filtering for diffusion
        longHadamardLeft[i] = allPassFiltersLong[i].processSample(longHadamardLeft[i]);
        longHadamardRight[i] = allPassFiltersLong[i].processSample(longHadamardRight[i]);
        
        longHadamardLeft[i] = envelopeFollower.processSample(0, longHadamardLeft[i]);
        longHadamardRight[i] = envelopeFollower.processSample(0, longHadamardRight[i]);

        // Apply attenuation
        longHadamardLeft[i] *= attenuationFactor;
        longHadamardRight[i] *= attenuationFactor;

        // Calculate feedback using the dynamic feedback parameter
        longFeedbackLeft[i] = juce::jlimit(-0.95f, 0.95f, (longHadamardLeft[i] + longHadamardLeft[i + 4]) * currentFeedback);
        longFeedbackRight[i] = juce::jlimit(-0.95f, 0.95f, (longHadamardRight[i] + longHadamardRight[i + 4]) * currentFeedback);
        
        longFeedbackLeft[i] = envelopeFollower.processSample(0, longFeedbackLeft[i]);
        longFeedbackRight[i] = envelopeFollower.processSample(0, longFeedbackRight[i]);
        
        // Update the delay buffer with the attenuated input sample and feedback
        delayBufferLeft[i][writePosition] = attenuatedInputLeft + longFeedbackLeft[i];
        delayBufferRight[i][writePosition] = attenuatedInputRight + longFeedbackRight[i];
    }

    // Increment write position in circular delay buffer
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

