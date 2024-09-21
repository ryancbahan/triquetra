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

    for (auto& filter : allPassFiltersLong)
        filter.prepare(spec);

    reverbWashLowpassFilterLeft.prepare(spec);
    reverbWashLowpassFilterRight.prepare(spec);

    *reverbWashLowpassFilterLeft.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);
    *reverbWashLowpassFilterRight.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);

    delayBufferSize = static_cast<int>(sampleRate * 2.0); // 2 seconds of delay buffer
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
    // Define the attenuation factor based on the number of delay taps (4 in this case)
    const float attenuationFactor = 1.0f / sqrt(4.0f);

    // Apply input attenuation
    float attenuatedInputLeft = inputSampleLeft * attenuationFactor;
    float attenuatedInputRight = inputSampleRight * attenuationFactor;

    for (int i = 0; i < 4; ++i)
    {
        float baseDelayLeft = longDelayTimes[i] * sampleRate;
        float baseDelayRight = baseDelayLeft - stereoOffset;

        float modulatedDelayLeft = baseDelayLeft * (1.0f + modulationValue);
        float modulatedDelayRight = baseDelayRight * (1.0f + modulationValue);

        // Fetch interpolated samples from the delay buffers without feedback
        longHadamardLeft[i] = getInterpolatedSample(delayBufferLeft[i], modulatedDelayLeft);
        longHadamardRight[i] = getInterpolatedSample(delayBufferRight[i], modulatedDelayRight);

        // Apply attenuation to the input signal for bloom-related taps
        longHadamardLeft[i + 4] = attenuatedInputLeft;
        longHadamardRight[i + 4] = attenuatedInputRight;
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

    return buffer[readPosition] + fraction * (buffer[nextPosition] - buffer[readPosition]);
}

std::array<float, 8> LongDelayProcessor::applyHadamardMixing(const std::array<float, 8>& input)
{
    std::array<float, 8> output;
    output[0] = input[0] + input[1] + input[2] + input[3] + input[4] + input[5] + input[6] + input[7];
    output[1] = input[0] - input[1] + input[2] - input[3] + input[4] - input[5] + input[6] - input[7];
    output[2] = input[0] + input[1] - input[2] - input[3] + input[4] + input[5] - input[6] - input[7];
    output[3] = input[0] - input[1] - input[2] + input[3] + input[4] - input[5] - input[6] + input[7];
    output[4] = input[0] + input[1] + input[2] + input[3] - input[4] - input[5] - input[6] - input[7];
    output[5] = input[0] - input[1] + input[2] - input[3] - input[4] + input[5] - input[6] + input[7];
    output[6] = input[0] + input[1] - input[2] + input[3] - input[4] + input[5] - input[6] + input[7];
    output[7] = input[0] - input[1] + input[2] + input[3] - input[4] - input[5] + input[6] - input[7];
    return output;
}
