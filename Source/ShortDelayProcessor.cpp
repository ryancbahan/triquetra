#include "ShortDelayProcessor.h"

ShortDelayProcessor::ShortDelayProcessor()
{
    // Constructor
}

void ShortDelayProcessor::prepare(double newSampleRate, int numChannels, float newFeedback, float newDiffusionAmount, float newModulationFeedbackAmount)
{
    sampleRate = newSampleRate;
    feedback = newFeedback;
    diffusionAmount = newDiffusionAmount;
    modulationFeedbackAmount = newModulationFeedbackAmount;

    juce::dsp::ProcessSpec spec{ sampleRate, static_cast<juce::uint32>(512), static_cast<juce::uint32>(numChannels) };

    for (auto& filter : allPassFiltersShort)
        filter.prepare(spec);

    reverbWashLowpassFilterLeft.prepare(spec);
    reverbWashLowpassFilterRight.prepare(spec);

    // Set lowpass filter coefficients
    *reverbWashLowpassFilterLeft.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);
    *reverbWashLowpassFilterRight.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 12000.0f);

    // Initialize delay buffers
    delayBufferSize = static_cast<int>(sampleRate * 2.0); // 2 seconds of delay buffer
    delayBufferLeft.resize(4, std::vector<float>(delayBufferSize, 0.0f));
    delayBufferRight.resize(4, std::vector<float>(delayBufferSize, 0.0f));

    writePosition = 0; // Initialize write position for circular buffer
}

void ShortDelayProcessor::process(const std::array<float, 4>& shortDelayTimes,
                                  const std::array<float, 4>& shortFeedbackLeft,
                                  const std::array<float, 4>& shortFeedbackRight,
                                  float modulationValue, float stereoOffset,
                                  std::array<float, 4>& shortDelayOutputLeft,
                                  std::array<float, 4>& shortDelayOutputRight,
                                  float inputSampleLeft, float inputSampleRight)
{
    for (int i = 0; i < 4; ++i)
    {
        float baseDelayLeft = shortDelayTimes[i] * sampleRate;
        float baseDelayRight = baseDelayLeft + stereoOffset;

        float modulatedDelayLeft = baseDelayLeft * (1.0f + modulationValue);
        float modulatedDelayRight = baseDelayRight * (1.0f + modulationValue);

        // Fetch interpolated samples from delay buffer
        shortDelayOutputLeft[i] = getInterpolatedSample(delayBufferLeft[i], modulatedDelayLeft) + shortFeedbackLeft[i] * feedback;
        shortDelayOutputRight[i] = getInterpolatedSample(delayBufferRight[i], modulatedDelayRight) + shortFeedbackRight[i] * feedback;

        // Incorporate input sample to the delay line (important)
        shortDelayOutputLeft[i] += inputSampleLeft;
        shortDelayOutputRight[i] += inputSampleRight;

        // Apply all-pass filtering and update feedback
        shortDelayOutputLeft[i] = allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
        shortDelayOutputRight[i] = allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);

        // Diffusion and additional filtering
        shortDelayOutputLeft[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
        shortDelayOutputRight[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);

        shortDelayOutputLeft[i] = reverbWashLowpassFilterLeft.processSample(shortDelayOutputLeft[i]);
        shortDelayOutputRight[i] = reverbWashLowpassFilterRight.processSample(shortDelayOutputRight[i]);
    }

    // Update the delay buffer for future feedback (left and right)
    for (int i = 0; i < 4; ++i)
    {
        delayBufferLeft[i][writePosition] = shortDelayOutputLeft[i];
        delayBufferRight[i][writePosition] = shortDelayOutputRight[i];
    }

    // Update write position in circular delay buffer
    writePosition = (writePosition + 1) % delayBufferSize;
}



float ShortDelayProcessor::getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples)
{
    int readPosition = (writePosition - static_cast<int>(delayInSamples) + delayBufferSize) % delayBufferSize;
    float fraction = delayInSamples - static_cast<int>(delayInSamples);
    int nextPosition = (readPosition + 1) % delayBufferSize;

    return buffer[readPosition] + fraction * (buffer[nextPosition] - buffer[readPosition]);
}

void ShortDelayProcessor::updateDelayBuffer(float inputLeft, float inputRight)
{
    for (int i = 0; i < 4; ++i)
    {
        delayBufferLeft[i][writePosition] = inputLeft;
        delayBufferRight[i][writePosition] = inputRight;
    }
    writePosition = (writePosition + 1) % delayBufferSize;
}

std::array<float, 4> ShortDelayProcessor::applyHadamardMixing(const std::array<float, 4>& input)
{
    std::array<float, 4> output;
    output[0] = input[0] + input[1] + input[2] + input[3];
    output[1] = input[0] - input[1] + input[2] - input[3];
    output[2] = input[0] + input[1] - input[2] - input[3];
    output[3] = input[0] - input[1] - input[2] + input[3];
    return output;
}

