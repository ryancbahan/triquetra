#include "LongDelayProcessor.h"
#include <cmath>
#include <random>

LongDelayProcessor::LongDelayProcessor()
{
    // Constructor
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.75, 1.25);
    
    // Initialize irregular delay factors
    for (auto& factor : irregularDelayFactors) {
        factor = static_cast<float>(dis(gen));
    }
}

void LongDelayProcessor::prepare(double newSampleRate, int numChannels, float newFeedback, float newBloomFeedbackGain, float newModulationFeedbackAmount, float newAttenuationFactor, float newLongSubdivisionsFactor)
{
    sampleRate = newSampleRate;
    feedback = newFeedback;
    bloomFeedbackGain = newBloomFeedbackGain;
    modulationFeedbackAmount = newModulationFeedbackAmount;
    attenuationFactor = newAttenuationFactor;
    longSubdivisionsFactor = newLongSubdivisionsFactor;

    juce::dsp::ProcessSpec spec{ sampleRate, static_cast<juce::uint32>(512), static_cast<juce::uint32>(numChannels) };

    for (auto& filter : allPassFiltersLong)
    {
        filter.reset();
        filter.prepare(spec);
    }

    reverbWashLowpassFilterLeft.reset();
    reverbWashLowpassFilterRight.reset();
    reverbWashLowpassFilterLeft.prepare(spec);
    reverbWashLowpassFilterRight.prepare(spec);

    *reverbWashLowpassFilterLeft.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);
    *reverbWashLowpassFilterRight.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);

    delayBufferSize = static_cast<int>(sampleRate * 2.0);
    delayBufferLeft.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    delayBufferRight.resize(8, std::vector<float>(delayBufferSize, 0.0f));

    writePosition = 0;
}

void LongDelayProcessor::process(const std::array<float, 4>& longDelayTimes,
                                 std::array<float, 8>& longFeedbackLeft,
                                 std::array<float, 8>& longFeedbackRight,
                                 float modulationValue, float stereoOffset,
                                 std::array<float, 8>& longHadamardLeft,
                                 std::array<float, 8>& longHadamardRight,
                                 float inputSampleLeft, float inputSampleRight)
{
    const float attenuationFactor = 1.0f / sqrt(4.0f);

    float attenuatedInputLeft = inputSampleLeft * attenuationFactor;
    float attenuatedInputRight = inputSampleRight * attenuationFactor;

    std::fill(longHadamardLeft.begin(), longHadamardLeft.end(), 0.0f);
    std::fill(longHadamardRight.begin(), longHadamardRight.end(), 0.0f);

    for (int i = 0; i < 4; ++i)
    {
        float baseDelayLeft = std::max(0.0f, longDelayTimes[i] * static_cast<float>(sampleRate));
        float baseDelayRight = std::max(0.0f, baseDelayLeft - stereoOffset);

        float modulatedDelayLeft = baseDelayLeft * (1.0f + modulationValue);
        float modulatedDelayRight = baseDelayRight * (1.0f + modulationValue);

        // Regular delay
        longHadamardLeft[i] = getInterpolatedSample(delayBufferLeft[i], modulatedDelayLeft);
        longHadamardRight[i] = getInterpolatedSample(delayBufferRight[i], modulatedDelayRight);

        // Irregular delays (carefully introduced)
        float irregularDelayLeft = modulatedDelayLeft * irregularDelayFactors[i];
        float irregularDelayRight = modulatedDelayRight * irregularDelayFactors[i];

        float irregularSampleLeft = getInterpolatedSample(delayBufferLeft[i], irregularDelayLeft);
        float irregularSampleRight = getInterpolatedSample(delayBufferRight[i], irregularDelayRight);

        // Mix irregular delays into Hadamard arrays (with reduced gain)
        longHadamardLeft[i + 4] = irregularSampleLeft * bloomFeedbackGain * 0.5f;
        longHadamardRight[i + 4] = irregularSampleRight * bloomFeedbackGain * 0.5f;

        // Apply all-pass filtering for diffusion
        longHadamardLeft[i] = allPassFiltersLong[i].processSample(longHadamardLeft[i]);
        longHadamardRight[i] = allPassFiltersLong[i].processSample(longHadamardRight[i]);

        // Apply attenuation
        longHadamardLeft[i] *= attenuationFactor;
        longHadamardRight[i] *= attenuationFactor;

        // Apply lowpass filtering
        longHadamardLeft[i] = reverbWashLowpassFilterLeft.processSample(longHadamardLeft[i]);
        longHadamardRight[i] = reverbWashLowpassFilterRight.processSample(longHadamardRight[i]);
    }

    // Calculate feedback (similar to original implementation)
    for (int i = 0; i < 4; ++i)
    {
        longFeedbackLeft[i] = juce::jlimit(-0.95f, 0.95f, longFeedbackLeft[i] + longHadamardLeft[i] * feedback);
        longFeedbackRight[i] = juce::jlimit(-0.95f, 0.95f, longFeedbackRight[i] + longHadamardRight[i] * feedback);
    }

    // Update the delay buffer with the attenuated input sample (no feedback yet)
    for (int i = 0; i < 4; ++i)
    {
        delayBufferLeft[i][writePosition] = attenuatedInputLeft;
        delayBufferRight[i][writePosition] = attenuatedInputRight;
    }

    // Increment write position in circular delay buffer
    writePosition = (writePosition + 1) % delayBufferSize;
}

float LongDelayProcessor::getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples)
{
    int readPosition = (writePosition - static_cast<int>(delayInSamples) + delayBufferSize) % delayBufferSize;
    float fraction = delayInSamples - static_cast<int>(delayInSamples);

    int nextPosition = (readPosition + 1) % delayBufferSize;

    float currentSample = buffer[readPosition];
    float nextSample = buffer[nextPosition];

    return clearDenormals(currentSample + fraction * (nextSample - currentSample));
}

inline float LongDelayProcessor::clearDenormals(float value)
{
    return std::abs(value) < 1.0e-15f ? 0.0f : value;
}
