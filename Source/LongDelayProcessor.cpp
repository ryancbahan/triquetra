#include "LongDelayProcessor.h"

LongDelayProcessor::LongDelayProcessor()
{
    // Constructor
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

    // Prepare the all-pass filters for diffusion
    for (auto& filter : allPassFiltersLong)
    {
        filter.reset();
        filter.prepare(spec);
    }

    // Prepare and reset the lowpass filters
    reverbWashLowpassFilterLeft.reset();
    reverbWashLowpassFilterRight.reset();
    reverbWashLowpassFilterLeft.prepare(spec);
    reverbWashLowpassFilterRight.prepare(spec);

    // Set lowpass coefficients
    *reverbWashLowpassFilterLeft.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);
    *reverbWashLowpassFilterRight.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);

    // Resize delay buffers and initialize to zero
    delayBufferSize = static_cast<int>(sampleRate * 2.0); // 2 seconds of delay buffer
    delayBufferLeft.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    delayBufferRight.resize(8, std::vector<float>(delayBufferSize, 0.0f));

    writePosition = 0; // Initialize write position for circular buffer
}

void LongDelayProcessor::process(const std::array<float, 4>& longDelayTimes,
                                 std::array<float, 8>& longFeedbackLeft,
                                 std::array<float, 8>& longFeedbackRight,
                                 float modulationValue, float stereoOffset,
                                 std::array<float, 8>& longHadamardLeft,
                                 std::array<float, 8>& longHadamardRight,
                                 float inputSampleLeft, float inputSampleRight)
{
    // Define the attenuation factor based on the number of delay taps (4 in this case)
    const float attenuationFactor = 1.0f / sqrt(4.0f);

    // Apply input attenuation
    float attenuatedInputLeft = inputSampleLeft * attenuationFactor;
    float attenuatedInputRight = inputSampleRight * attenuationFactor;

    // Zero out the Hadamard arrays
    std::fill(longHadamardLeft.begin(), longHadamardLeft.end(), 0.0f);
    std::fill(longHadamardRight.begin(), longHadamardRight.end(), 0.0f);

    for (int i = 0; i < 4; ++i)
    {
        // Ensure positive delay time
        float baseDelayLeft = std::max(0.0f, longDelayTimes[i] * static_cast<float>(sampleRate)); // Ensure positive delay time
        float baseDelayRight = std::max(0.0f, baseDelayLeft - stereoOffset); // Stereo offset is subtracted

        float modulatedDelayLeft = baseDelayLeft * (1.0f + modulationValue);
        float modulatedDelayRight = baseDelayRight * (1.0f + modulationValue);

        // Fetch interpolated samples from the delay buffers without feedback
        longHadamardLeft[i] = getInterpolatedSample(delayBufferLeft[i], modulatedDelayLeft);
        longHadamardRight[i] = getInterpolatedSample(delayBufferRight[i], modulatedDelayRight);

        // Apply all-pass filtering for diffusion
        longHadamardLeft[i] = allPassFiltersLong[i].processSample(longHadamardLeft[i]);
        longHadamardRight[i] = allPassFiltersLong[i].processSample(longHadamardRight[i]);

        // Apply attenuation to the input signal for bloom-related taps
        longHadamardLeft[i + 4] = attenuatedInputLeft;
        longHadamardRight[i + 4] = attenuatedInputRight;
    }

    // Apply gain limiting on feedback to prevent overload and clipping
    for (int i = 0; i < 8; ++i)
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
    // Ensure delay is positive and within buffer bounds
    int readPosition = (writePosition - static_cast<int>(delayInSamples) + delayBufferSize) % delayBufferSize;
    float fraction = delayInSamples - static_cast<int>(delayInSamples);

    // Calculate the next position, wrapping around the circular buffer
    int nextPosition = (readPosition + 1) % delayBufferSize;

    // Return interpolated sample between the current and next sample in the buffer
    float currentSample = buffer[readPosition];
    float nextSample = buffer[nextPosition];

    // Linear interpolation between current and next sample
    return clearDenormals(currentSample + fraction * (nextSample - currentSample));
}

inline float LongDelayProcessor::clearDenormals(float value)
{
    // Zero out very small values to prevent denormals (tiny numbers causing performance issues)
    return std::abs(value) < 1.0e-15f ? 0.0f : value;
}
