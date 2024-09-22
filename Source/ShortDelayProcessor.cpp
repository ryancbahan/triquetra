#include "ShortDelayProcessor.h"
#include <random>

ShortDelayProcessor::ShortDelayProcessor()
{
    // Constructor - initialize the modulation phases and offsets
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, juce::MathConstants<float>::twoPi);

    // Initialize the unique phase offsets for each delay line
    for (int i = 0; i < 8; ++i)
    {
        modulationPhases[i] = 0.0f;            // Initial phase for each delay line
        phaseOffsets[i] = static_cast<float>(dis(gen));  // Random phase offset for each delay line
    }
}

void ShortDelayProcessor::reset()
{
    // Clear the delay buffers
    for (auto& buffer : delayBufferLeft)
        std::fill(buffer.begin(), buffer.end(), 0.0f);

    for (auto& buffer : delayBufferRight)
        std::fill(buffer.begin(), buffer.end(), 0.0f);

    // Reset the write position to the start
    writePosition = 0;

    // Reset the all-pass filters to their initial state
    for (auto& filter : allPassFiltersShort)
        filter.reset();
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

    // Initialize delay buffers
    delayBufferSize = static_cast<int>(sampleRate * 4.0); // 4 seconds of delay buffer
    delayBufferLeft.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    delayBufferRight.resize(8, std::vector<float>(delayBufferSize, 0.0f));

    writePosition = 0; // Initialize write position for circular buffer

    modulationFrequency = 5.5f; // Example tremolo rate, can be made a parameter
}

void ShortDelayProcessor::process(const std::array<float, 8>& shortDelayTimes,
                                  const std::array<float, 8>& shortFeedbackLeft,
                                  const std::array<float, 8>& shortFeedbackRight,
                                  float modulationValue, float stereoOffset,
                                  std::array<float, 8>& shortDelayOutputLeft,
                                  std::array<float, 8>& shortDelayOutputRight,
                                  float inputSampleLeft, float inputSampleRight,
                                  float currentFeedback)
{
    currentFeedback = juce::jlimit(0.0f, 1.0f, currentFeedback);  // Ensure feedback is in the valid range

    for (int i = 0; i < 8; ++i)
    {
        // Increment the modulation phase for this delay line, respecting the unique phase offset
        modulationPhases[i] += modulationFrequency / static_cast<float>(sampleRate);
        if (modulationPhases[i] >= 1.0f)
            modulationPhases[i] -= 1.0f;

        // Calculate the tremolo factor using a sine wave and the unique phase offset
        float tremoloFactor = 1.0f + modulationValue * std::sin(2.0f * juce::MathConstants<float>::pi * modulationPhases[i] + phaseOffsets[i]);

        // Modulate the amplitude (tremolo) instead of modulating delay time
        float modulatedInputLeft = inputSampleLeft * tremoloFactor;
        float modulatedInputRight = inputSampleRight * tremoloFactor;

        float baseDelayLeft = shortDelayTimes[i] * sampleRate;
        float baseDelayRight = baseDelayLeft + stereoOffset;

        // Fetch interpolated samples from delay buffer (output will be 100% wet)
        shortDelayOutputLeft[i] = getInterpolatedSample(delayBufferLeft[i], baseDelayLeft) + shortFeedbackLeft[i] * currentFeedback;
        shortDelayOutputRight[i] = getInterpolatedSample(delayBufferRight[i], baseDelayRight) + shortFeedbackRight[i] * currentFeedback;

        // Apply all-pass filtering and update feedback
        shortDelayOutputLeft[i] = allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
        shortDelayOutputRight[i] = allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);

        // Diffusion and additional filtering
        shortDelayOutputLeft[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
        shortDelayOutputRight[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);

        // Write the modulated input sample to the delay buffer (using the modulated input sample)
        delayBufferLeft[i][writePosition] = modulatedInputLeft;
        delayBufferRight[i][writePosition] = modulatedInputRight;
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
    for (int i = 0; i < 8; ++i)
    {
        delayBufferLeft[i][writePosition] = inputLeft;
        delayBufferRight[i][writePosition] = inputRight;
    }
    writePosition = (writePosition + 1) % delayBufferSize;
}

std::array<float, 8> ShortDelayProcessor::applyHadamardMixing(const std::array<float, 8>& input)
{
    std::array<float, 8> output;
    output[0] = input[0] + input[1] + input[2] + input[3] + input[4] + input[5] + input[6] + input[7];
    output[1] = input[0] - input[1] + input[2] - input[3] + input[4] - input[5] + input[6] - input[7];
    output[2] = input[0] + input[1] - input[2] - input[3] + input[4] + input[5] - input[6] - input[7];
    output[3] = input[0] - input[1] - input[2] + input[3] + input[4] - input[5] - input[6] + input[7];
    output[4] = input[0] + input[1] + input[2] + input[3] - input[4] - input[5] - input[6] - input[7];
    output[5] = input[0] - input[1] + input[2] - input[3] - input[4] + input[5] - input[6] + input[7];
    output[6] = input[0] + input[1] - input[2] - input[3] - input[4] - input[5] + input[6] + input[7];
    output[7] = input[0] - input[1] - input[2] + input[3] - input[4] + input[5] + input[6] - input[7];
    return output;
}
