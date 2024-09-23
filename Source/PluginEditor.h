/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
/**
*/
class TriquetraAudioProcessorEditor  : public juce::AudioProcessorEditor
{
public:
    TriquetraAudioProcessorEditor (TriquetraAudioProcessor&);
    ~TriquetraAudioProcessorEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;

private:
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    TriquetraAudioProcessor& audioProcessor;
    juce::Slider mixParamSlider;
    juce::Slider delayTimeParamSlider;
    juce::Slider feedbackParamSlider;
    juce::Slider depthParamSlider;
    juce::Slider clockParamSlider;

    void setupKnob(juce::Slider& slider, juce::RangedAudioParameter* parameter,
                       int x, int y, int width, int height, const juce::String& labelText);
    
    std::vector<std::unique_ptr<juce::Label>> sliderLabels;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TriquetraAudioProcessorEditor)
};
