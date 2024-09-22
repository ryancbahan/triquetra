#include "LongDelayProcessor.h"
#include <cmath>
#include <random>

LongDelayProcessor::LongDelayProcessor()
{
    // Constructor
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.9, 1.1);
    
    // Initialize irregular delay factors
    for (auto& factor : irregularDelayFactors) {
        factor = static_cast<float>(dis(gen));
    }
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

//    reverbWashLowpassFilterLeft.reset();
//    reverbWashLowpassFilterRight.reset();
//    reverbWashLowpassFilterLeft.prepare(spec);
//    reverbWashLowpassFilterRight.prepare(spec);
//
//    *reverbWashLowpassFilterLeft.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);
//    *reverbWashLowpassFilterRight.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);

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

    float cumulativeIrregularDelayLeft = 0.0f;
    float cumulativeIrregularDelayRight = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        float baseDelayLeft = std::max(0.0f, longDelayTimes[i] * static_cast<float>(sampleRate));
        float baseDelayRight = std::max(0.0f, baseDelayLeft - stereoOffset);

        // Original delay line
        float originalDelayLeft = baseDelayLeft * (1.0f + modulationValue);
        float originalDelayRight = baseDelayRight * (1.0f + modulationValue);

        // Get samples for original delays
        longHadamardLeft[i] = getInterpolatedSample(delayBufferLeft[i], originalDelayLeft);
        longHadamardRight[i] = getInterpolatedSample(delayBufferRight[i], originalDelayRight);

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

        // Add to cumulative irregular delay
        cumulativeIrregularDelayLeft += irregularDelayLeft;
        cumulativeIrregularDelayRight += irregularDelayRight;

        // Ensure delays don't exceed buffer size
        cumulativeIrregularDelayLeft = std::min(static_cast<float>(delayBufferSize - 1), cumulativeIrregularDelayLeft);
        cumulativeIrregularDelayRight = std::min(static_cast<float>(delayBufferSize - 1), cumulativeIrregularDelayRight);

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
        longHadamardLeft[i + 4] = irregularSampleLeft * bloomFeedbackGain;
        longHadamardRight[i + 4] = irregularSampleRight * bloomFeedbackGain;

        // Apply all-pass filtering for diffusion
        longHadamardLeft[i] = allPassFiltersLong[i].processSample(longHadamardLeft[i]);
        longHadamardRight[i] = allPassFiltersLong[i].processSample(longHadamardRight[i]);

        // Apply attenuation
        longHadamardLeft[i] *= attenuationFactor;
        longHadamardRight[i] *= attenuationFactor;

        // Apply lowpass filtering
//        longHadamardLeft[i] = reverbWashLowpassFilterLeft.processSample(longHadamardLeft[i]);
//        longHadamardRight[i] = reverbWashLowpassFilterRight.processSample(longHadamardRight[i]);
        
        // Calculate feedback using the global feedback control
        longFeedbackLeft[i] = juce::jlimit(-0.95f, 0.95f, (longHadamardLeft[i] + longHadamardLeft[i + 4]) * feedback);
        longFeedbackRight[i] = juce::jlimit(-0.95f, 0.95f, (longHadamardRight[i] + longHadamardRight[i + 4]) * feedback);
        
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

    return clearDenormals(currentSample + fraction * (nextSample - currentSample));
}

inline float LongDelayProcessor::clearDenormals(float value)
{
    return std::abs(value) < 1.0e-15f ? 0.0f : value;
}
