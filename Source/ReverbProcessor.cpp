#include "ReverbProcessor.h"

ReverbProcessor::ReverbProcessor()
    : feedbackGain(0.3f)  // Control the amount of self-feedback (adjust as needed)
{
    // Constructor is now empty as we set up filters in the prepare method
}

void ReverbProcessor::prepare(double sampleRate, int samplesPerBlock)
{
    this->sampleRate = sampleRate;

    juce::dsp::ProcessSpec spec{ sampleRate, static_cast<juce::uint32> (samplesPerBlock), 2 };

    // Prepare all-pass filters (updated for 8 delays)
    for (auto& filter : allPassFiltersLong)
    {
        filter.prepare(spec);
        *filter.coefficients = *juce::dsp::IIR::Coefficients<float>::makeAllPass(sampleRate, 4000.0f);
    }

    for (auto& filter : allPassFiltersShort)
    {
        filter.prepare(spec);
        *filter.coefficients = *juce::dsp::IIR::Coefficients<float>::makeAllPass(sampleRate, 1000.0f);
    }

    // Set up filter coefficients with steeper slopes
    auto lowCoefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 2000.0f, 0.7071f);
    auto highCoefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass(sampleRate, 40.0f, 0.7071f);
    
    // Prepare steep lowpass and highpass filters
    lowpassFilter.prepare(spec);
    highpassFilter.prepare(spec);
    
    *lowpassFilter.coefficients = *lowCoefficients;
    *highpassFilter.coefficients = *highCoefficients;

    // Reset all filter states
    for (auto& filter : allPassFiltersLong)
        filter.reset();
    for (auto& filter : allPassFiltersShort)
        filter.reset();
    lowpassFilter.reset();
    highpassFilter.reset();

    // Initialize reverbWash arrays for 8 delays
    reverbWashLeft.fill(0.0f);
    reverbWashRight.fill(0.0f);
}

void ReverbProcessor::process(const std::array<float, 8>& shortHadamardLeft,
                              const std::array<float, 8>& shortHadamardRight,
                              const std::array<float, 8>& longHadamardLeft,
                              const std::array<float, 8>& longHadamardRight,
                              std::array<float, 8>& outLeft,
                              std::array<float, 8>& outRight)
{
    updateModulation();

    const float inputScale = 0.15f; // Reduce input gain to prevent buildup
    const float crossFeedbackLeftGain = 0.12f;
    const float crossFeedbackRightGain = 0.18f;

    for (int i = 0; i < 8; ++i)
    {
        // Combine short and long Hadamard matrices and apply input scaling
        float reverbInputLeft = (shortHadamardLeft[i] + longHadamardLeft[i]) * inputScale;
        float reverbInputRight = (shortHadamardRight[i] + longHadamardRight[i]) * inputScale;

        // Add self-feedback from the previous iteration
        reverbInputLeft += reverbWashLeft[i] * feedbackGain;
        reverbInputRight += reverbWashRight[i] * feedbackGain;

        // Process through high-pass and low-pass filters
        reverbInputLeft = highpassFilter.processSample(lowpassFilter.processSample(reverbInputLeft));
        reverbInputRight = highpassFilter.processSample(lowpassFilter.processSample(reverbInputRight));

        // Check if the input is valid, prevent processing if the signal is too small
        if (std::abs(reverbInputLeft) < 1e-6f || std::abs(reverbInputRight) < 1e-6f)
        {
            continue;  // Skip processing if the signal is too small
        }

        // Process the signal through long all-pass filters
        reverbWashLeft[i] = allPassFiltersLong[i].processSample(reverbInputLeft);
        reverbWashRight[i] = allPassFiltersLong[i].processSample(reverbInputRight);
//
//        // Process through additional diffusion stages (make sure we process all 8 stages)
        for (int j = 0; j < 8; ++j)
        {
            reverbWashLeft[i] = allPassFiltersShort[j].processSample(reverbWashLeft[i]);
            reverbWashRight[i] = allPassFiltersShort[j].processSample(reverbWashRight[i]);
        }

//         Introduce cross-feedback with asymmetry for stereo width
        reverbWashLeft[i] += reverbWashRight[(i + 1) % 8] * crossFeedbackLeftGain;
        reverbWashRight[i] += reverbWashLeft[(i + 2) % 8] * crossFeedbackRightGain;

        // Apply decay
        reverbWashLeft[i] *= reverbWashDecay;
        reverbWashRight[i] *= reverbWashDecay;

        // Final high-pass and low-pass filtering
        reverbWashLeft[i] = highpassFilter.processSample(lowpassFilter.processSample(reverbWashLeft[i]));
        reverbWashRight[i] = highpassFilter.processSample(lowpassFilter.processSample(reverbWashRight[i]));

        // Store the processed samples directly in the output arrays
        outLeft[i] = std::clamp(reverbWashLeft[i], -1.0f, 1.0f);
        outRight[i] = std::clamp(reverbWashRight[i], -1.0f, 1.0f);
    }
}

void ReverbProcessor::updateModulation()
{
    reverbWashPhase += reverbWashModulationFreq / sampleRate;
    if (reverbWashPhase >= 1.0f) reverbWashPhase -= 1.0f;
    reverbWashModulation = std::sin(2.0f * juce::MathConstants<float>::pi * reverbWashPhase) * reverbWashModulationDepth;
}
