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
    isDelayLineModulated = { true, false, true, false, true, false, true, false };
    modulationFrequencies = { 0.1f, 0.0f, 0.13f, 0.0f, 0.16f, 0.0f, 0.19f, 0.0f }; // Frequencies in Hz

    for (int i = 0; i < 8; ++i)
    {
        lfoOscillators[i].initialise([](float x) { return std::sin(x); }, 128);

        if (isDelayLineModulated[i])
        {
            lfoOscillators[i].setFrequency(modulationFrequencies[i]);
        }
        else
        {
            // Set frequency to zero for unmodulated delay lines
            lfoOscillators[i].setFrequency(0.0f);
        }
    }
    
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
    
    for (int i = 0; i < 8; ++i)
    {
        lfoOscillators[i].prepare(spec);
        lfoOscillators[i].reset();
    }
    // Initialize delay buffers
    delayBufferSize = static_cast<int>(sampleRate * 4.0); // 4 seconds of delay buffer
    delayBufferLeft.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    delayBufferRight.resize(8, std::vector<float>(delayBufferSize, 0.0f));
    
    lowPassFilterLeft.prepare(spec);
    lowPassFilterRight.prepare(spec);

    writePosition = 0; // Initialize write position for circular buffer
}

void ShortDelayProcessor::process(
    const std::array<float, 8>& shortDelayTimes,
    std::array<float, 8>& shortFeedbackLeft,
    std::array<float, 8>& shortFeedbackRight,
    float modulationDepth,
    float stereoOffset,
    std::array<float, 8>& shortDelayOutputLeft,
    std::array<float, 8>& shortDelayOutputRight,
    float inputSampleLeft,
    float inputSampleRight,
    float currentFeedback,
    float dampValue)
{
    currentFeedback = juce::jlimit(0.0f, 1.0f, currentFeedback);  // Ensure feedback is in the valid range

    // Damping using a low-pass filter
    const float maxCutoffFreq = 15000.0f;
    const float minCutoffFreq = 250.0f;
    float targetCutoffFreq = juce::jmap(dampValue, 0.0f, 1.0f, maxCutoffFreq, minCutoffFreq);

    // Smoothly adjust the current cutoff frequency towards the target
    currentCutoffFreq += (targetCutoffFreq - currentCutoffFreq) * 0.001f;
    lowPassFilterLeft.setCutoffFrequency(currentCutoffFreq);
    lowPassFilterRight.setCutoffFrequency(currentCutoffFreq);

    // Apply the low-pass filter to the input samples
    float filteredInputLeft = lowPassFilterLeft.processSample(0, inputSampleLeft);
    float filteredInputRight = lowPassFilterRight.processSample(1, inputSampleRight);

    for (int i = 0; i < 8; ++i)
    {
        // Base delay times in samples
        float baseDelaySamplesLeft = shortDelayTimes[i] * static_cast<float>(sampleRate);
        float baseDelaySamplesRight = baseDelaySamplesLeft + stereoOffset;

        // Calculate maximum modulation depth in samples (as a percentage of base delay time)
        float maxModulationSamples = baseDelaySamplesLeft * modulationDepth;

        float modulatedDelayLeft = baseDelaySamplesLeft;
        float modulatedDelayRight = baseDelaySamplesRight;

        // Check if this delay line should be modulated
        if (isDelayLineModulated[i])
        {
            // Get the next LFO sample (ranges from -1 to 1)
            float lfoSample = lfoOscillators[i].processSample(0.0f);

            // Calculate modulation amount in samples
            float modulationAmount = lfoSample * maxModulationSamples;

            // Apply modulation to delay times
            modulatedDelayLeft = baseDelaySamplesLeft + modulationAmount;
            modulatedDelayRight = baseDelaySamplesRight + modulationAmount;

            // Limit modulated delay times
            float minDelaySamples = baseDelaySamplesLeft * 0.95f; // No less than 5% decrease
            float maxDelaySamples = baseDelaySamplesLeft * 1.05f; // No more than 5% increase
            modulatedDelayLeft = juce::jlimit(minDelaySamples, maxDelaySamples, modulatedDelayLeft);
            modulatedDelayRight = juce::jlimit(minDelaySamples, maxDelaySamples, modulatedDelayRight);
        }

        // Ensure modulated delay times are within buffer bounds
        modulatedDelayLeft = juce::jlimit(1.0f, static_cast<float>(delayBufferSize - 1), modulatedDelayLeft);
        modulatedDelayRight = juce::jlimit(1.0f, static_cast<float>(delayBufferSize - 1), modulatedDelayRight);

        // Fetch interpolated samples from delay buffer (already includes feedback)
        shortDelayOutputLeft[i] = getInterpolatedSample(delayBufferLeft[i], modulatedDelayLeft);
        shortDelayOutputRight[i] = getInterpolatedSample(delayBufferRight[i], modulatedDelayRight);

        // Apply all-pass filtering and diffusion
        shortDelayOutputLeft[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputLeft[i]);
        shortDelayOutputRight[i] = diffusionAmount * allPassFiltersShort[i].processSample(shortDelayOutputRight[i]);

        // Calculate feedback samples
        float feedbackSampleLeft = shortDelayOutputLeft[i] * currentFeedback;
        float feedbackSampleRight = shortDelayOutputRight[i] * currentFeedback;

        // Write the filtered input sample plus feedback to the delay buffer
        delayBufferLeft[i][writePosition] = filteredInputLeft + feedbackSampleLeft;
        delayBufferRight[i][writePosition] = filteredInputRight + feedbackSampleRight;

        // Update feedback variables
        shortFeedbackLeft[i] = feedbackSampleLeft;
        shortFeedbackRight[i] = feedbackSampleRight;
    }

    // Update write position in circular delay buffer
    writePosition = (writePosition + 1) % delayBufferSize;
}



float ShortDelayProcessor::getInterpolatedSample(const std::vector<float>& buffer, float delayInSamples)
{
    int delayInt = static_cast<int>(delayInSamples);
    float frac = delayInSamples - delayInt;

    // Wrap read positions around the buffer
    int idx0 = (writePosition - delayInt - 1 + delayBufferSize) % delayBufferSize;
    int idx1 = (writePosition - delayInt + delayBufferSize) % delayBufferSize;
    int idx2 = (writePosition - delayInt + 1 + delayBufferSize) % delayBufferSize;
    int idx3 = (writePosition - delayInt + 2 + delayBufferSize) % delayBufferSize;

    // Fetch samples
    float y0 = buffer[idx0];
    float y1 = buffer[idx1];
    float y2 = buffer[idx2];
    float y3 = buffer[idx3];

    // Cubic interpolation coefficients
    float a0 = y3 - y2 - y0 + y1;
    float a1 = y0 - y1 - a0;
    float a2 = y2 - y0;
    float a3 = y1;

    // Calculate interpolated sample
    float fracSquared = frac * frac;
    float fracCubed = fracSquared * frac;

    return a0 * fracCubed + a1 * fracSquared + a2 * frac + a3;
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
