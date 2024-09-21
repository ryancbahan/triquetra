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

    // Prepare all-pass filters
    for (auto& filter : allPassFiltersLong)
    {
        filter.prepare(spec);
        *filter.coefficients = *juce::dsp::IIR::Coefficients<float>::makeAllPass(sampleRate, 400.0f);
    }

    for (auto& filter : allPassFiltersShort)
    {
        filter.prepare(spec);
        *filter.coefficients = *juce::dsp::IIR::Coefficients<float>::makeAllPass(sampleRate, 200.0f);
    }

    // Set up filter coefficients with steeper slopes
    auto lowCoefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 1000.0f);
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
}

void ReverbProcessor::process(const std::array<float, 4>& shortHadamardLeft,
                              const std::array<float, 4>& shortHadamardRight,
                              const std::array<float, 8>& longHadamardLeft,
                              const std::array<float, 8>& longHadamardRight,
                              float& outLeft, float& outRight)
{
    updateModulation();

    const float inputScale = 0.15f; // Reduce input gain to prevent buildup
    const float crossFeedbackLeftGain = 0.12f;
    const float crossFeedbackRightGain = 0.18f;

    for (int i = 0; i < 8; ++i)
    {
        // Combine short and long Hadamard matrices and apply input scaling
        float reverbInputLeft = (shortHadamardLeft[i % 4] + longHadamardLeft[i]) * inputScale;
        float reverbInputRight = (shortHadamardRight[i % 4] + longHadamardRight[i]) * inputScale;

        // Add self-feedback from the previous iteration
        reverbInputLeft += reverbWashLeft[i] * feedbackGain;
        reverbInputRight += reverbWashRight[i] * feedbackGain;

        // Process through high-pass and low-pass filters
        reverbInputLeft = highpassFilter.processSample(lowpassFilter.processSample(reverbInputLeft));
        reverbInputRight = highpassFilter.processSample(lowpassFilter.processSample(reverbInputRight));

        // Process the signal through long all-pass filters
        reverbWashLeft[i] = allPassFiltersLong[i % 4].processSample(reverbInputLeft);
        reverbWashRight[i] = allPassFiltersLong[i % 4].processSample(reverbInputRight);

        // Process through additional diffusion stages
        for (int j = 0; j < 6; ++j) // Increased diffusion stages
        {
            reverbWashLeft[i] = allPassFiltersShort[j % 4].processSample(reverbWashLeft[i]);
            reverbWashRight[i] = allPassFiltersShort[j % 4].processSample(reverbWashRight[i]);
        }

        // Introduce cross-feedback with asymmetry for stereo width
        reverbWashLeft[i] += reverbWashRight[(i + 1) % 8] * crossFeedbackLeftGain;
        reverbWashRight[i] += reverbWashLeft[(i + 2) % 8] * crossFeedbackRightGain;

        // Apply decay
        reverbWashLeft[i] *= reverbWashDecay;
        reverbWashRight[i] *= reverbWashDecay;

        // Final high-pass and low-pass filtering
        reverbWashLeft[i] = highpassFilter.processSample(lowpassFilter.processSample(reverbWashLeft[i]));
        reverbWashRight[i] = highpassFilter.processSample(lowpassFilter.processSample(reverbWashRight[i]));
    }

    outLeft = 0.0f;
    outRight = 0.0f;
    
    // Sum the reverb wash signals for final output
    for (int i = 0; i < 8; ++i)
    {
        outLeft += reverbWashLeft[i];
        outRight += reverbWashRight[i];
    }

    // Optional normalization (uncomment if needed)
//    outLeft *= 0.125f;
//    outRight *= 0.125f;

    // Final safety clipping
    outLeft = std::clamp(outLeft, -1.0f, 1.0f);
    outRight = std::clamp(outRight, -1.0f, 1.0f);
}

void ReverbProcessor::updateModulation()
{
    reverbWashPhase += reverbWashModulationFreq / sampleRate;
    if (reverbWashPhase >= 1.0f) reverbWashPhase -= 1.0f;
    reverbWashModulation = std::sin(2.0f * juce::MathConstants<float>::pi * reverbWashPhase) * reverbWashModulationDepth;
}

