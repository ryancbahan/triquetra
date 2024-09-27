#include "ShortDelayProcessor.h"
#include <random>

ShortDelayProcessor::ShortDelayProcessor()
{
    // Constructor - initialize the modulation phases and offsets
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Prime number ratios for phase offsets and frequency variations
    std::vector<int> primeRatios = {11, 13, 17, 19, 23, 29, 31, 37};  // Larger prime numbers for slower modulation
    currentCutoffFreq = 15000.0f;
    lowPassFilterLeft.setType(juce::dsp::StateVariableTPTFilterType::lowpass);
    lowPassFilterRight.setType(juce::dsp::StateVariableTPTFilterType::lowpass);
    
    for (int i = 0; i < 8; ++i)
    {
        modulationPhases[i] = 0.0f;   // Initialize the modulation phase for each line
        phaseOffsets[i] = static_cast<float>(primeRatios[i]);  // Prime number offsets
        modulationFrequencies[i] = 0.0f;  // Will be calculated during process based on delay time
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
    
    lowPassFilterLeft.prepare(spec);
    lowPassFilterRight.prepare(spec);

    writePosition = 0; // Initialize write position for circular buffer
}

void ShortDelayProcessor::process(const std::array<float, 8>& shortDelayTimes,
                                  const std::array<float, 8>& shortFeedbackLeft,
                                  const std::array<float, 8>& shortFeedbackRight,
                                  float modulationValue, float stereoOffset,
                                  std::array<float, 8>& shortDelayOutputLeft,
                                  std::array<float, 8>& shortDelayOutputRight,
                                  float inputSampleLeft, float inputSampleRight,
                                  float currentFeedback, float dampValue)
{
    currentFeedback = juce::jlimit(0.0f, 1.0f, currentFeedback);  // Ensure feedback is in the valid range

    // Prime number ratios for irregular phase offsets
    std::vector<int> primeRatios = {11, 13, 17, 19, 23, 29, 31, 37};
    
    float maxCutoffFreq = 15000.0f;  // Maximum cutoff frequency (minimal damping)
    float minCutoffFreq = 250.0f;    // Minimum cutoff frequency (maximum damping)
    float targetCutoffFreq = juce::jmap(dampValue, 0.0f, 1.0f, maxCutoffFreq, minCutoffFreq);

    for (int i = 0; i < 8; ++i)
    {
        // Calculate base modulation frequency as a 1/4 note of the current delay time
        float baseModulationTime = shortDelayTimes[i] * 4.0f;  // 1/4 note modulation time, slower and more subtle

        // Multiply the base modulation frequency by prime ratios to get longer tremolo periods
        modulationFrequencies[i] = (1.0f / baseModulationTime) * static_cast<float>(primeRatios[i]);

        // Increment the modulation phase for this delay line
        modulationPhases[i] += modulationFrequencies[i] / static_cast<float>(sampleRate);

        if (modulationPhases[i] >= 1.0f)
            modulationPhases[i] -= 1.0f;

        // Apply the phase offset and calculate the tremolo factor
        // Reduce the modulation depth (0.5 to reduce intensity of tremolo)
        float tremoloFactor = 1.0f + (modulationValue * 0.25f) * std::sin(1.0f * juce::MathConstants<float>::pi * (modulationPhases[i] + phaseOffsets[i]));

        // Modulate the amplitude (tremolo) instead of modulating delay time
        float modulatedInputLeft = inputSampleLeft * tremoloFactor;
        float modulatedInputRight = inputSampleRight * tremoloFactor;
        
        currentCutoffFreq = currentCutoffFreq + (targetCutoffFreq - currentCutoffFreq) * 0.001f;
        lowPassFilterLeft.setCutoffFrequency(currentCutoffFreq);
        lowPassFilterRight.setCutoffFrequency(currentCutoffFreq);
        
        modulatedInputLeft = lowPassFilterLeft.processSample(0, modulatedInputLeft);
        modulatedInputRight = lowPassFilterRight.processSample(1, modulatedInputRight);

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
