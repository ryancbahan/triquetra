#include "ReverbProcessor.h"

ReverbProcessor::ReverbProcessor()
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
    auto lowCoefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 1000.0f, 0.7071f);
    auto highCoefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass(sampleRate, 10.0f, 0.7071f);
    
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

    const float inputScale = 0.5f; // Reduce input gain to prevent buildup

    for (int i = 0; i < 8; ++i)
    {
        float reverbInputLeft = (shortHadamardLeft[i % 4] + longHadamardLeft[i]) * inputScale;
        float reverbInputRight = (shortHadamardRight[i % 4] + longHadamardRight[i]) * inputScale;

        // Process through all-pass filters
        reverbWashLeft[i] = allPassFiltersLong[i % 4].processSample(reverbInputLeft);
        reverbWashRight[i] = allPassFiltersLong[i % 4].processSample(reverbInputRight);

        for (int j = 0; j < 4; ++j)
        {
            reverbWashLeft[i] = allPassFiltersShort[j].processSample(reverbWashLeft[i]);
            reverbWashRight[i] = allPassFiltersShort[j].processSample(reverbWashRight[i]);
        }

        // Cross-feedback with attenuation
        reverbWashLeft[i] += reverbWashRight[(i + 1) % 8] * 0.3f;
        reverbWashRight[i] += reverbWashLeft[(i + 1) % 8] * 0.3f;

        // Apply decay
        reverbWashLeft[i] *= reverbWashDecay;
        reverbWashRight[i] *= reverbWashDecay;

        // Apply steep filters and store the results
        reverbWashLeft[i] = highpassFilter.processSample(lowpassFilter.processSample(reverbWashLeft[i]));
        reverbWashRight[i] = highpassFilter.processSample(lowpassFilter.processSample(reverbWashRight[i]));
    }

    outLeft = 0.0f;
    outRight = 0.0f;
    for (int i = 0; i < 8; ++i)
    {
        outLeft += reverbWashLeft[i];
        outRight += reverbWashRight[i];
    }

    // Normalize output
    outLeft *= 0.125f;
    outRight *= 0.125f;

    // Apply DC blocking
    outLeft = dcBlockerLeft.process(outLeft);
    outRight = dcBlockerRight.process(outRight);

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

float ReverbProcessor::DCBlocker::process(float x)
{
    float y = x - xm1 + 0.995f * ym1;
    xm1 = x;
    ym1 = y;
    return y;
}
