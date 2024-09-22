/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
TriquetraAudioProcessorEditor::TriquetraAudioProcessorEditor (TriquetraAudioProcessor& p)
    : AudioProcessorEditor (&p), audioProcessor (p)
{
    // Make sure that before the constructor has finished, you've set the
    // editor's size to whatever you need it to be.
    setSize (400, 300);
    auto& params = processor.getParameters();
    setupKnob(mixParamSlider, static_cast<juce::AudioParameterFloat*>(params.getUnchecked(0)), 0, 12, 100, 100, "Mix");
    setupKnob(delayTimeParamSlider, static_cast<juce::AudioParameterFloat*>(params.getUnchecked(1)), 100, 12, 100, 100, "Delay Time");

}

TriquetraAudioProcessorEditor::~TriquetraAudioProcessorEditor()
{
}

void TriquetraAudioProcessorEditor::setupKnob(juce::Slider& slider, juce::RangedAudioParameter* parameter,
               int x, int y, int width, int height, const juce::String& labelText)
{
    slider.setBounds(x, y, width, height);
    slider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    slider.setTextBoxStyle(juce::Slider::TextEntryBoxPosition::NoTextBox, true, 0, 0);
    slider.setRange(parameter->getNormalisableRange().start, parameter->getNormalisableRange().end);
    slider.setValue(parameter->getValue());
    addAndMakeVisible(slider);

    slider.onValueChange = [&slider, parameter] {
        parameter->setValueNotifyingHost(parameter->convertTo0to1(slider.getValue()));
    };

    slider.onDragStart = [parameter] {
        parameter->beginChangeGesture();
    };

    slider.onDragEnd = [parameter] {
        parameter->endChangeGesture();
    };
    
    auto label = std::make_unique<juce::Label>();
    label->setText(labelText, juce::NotificationType::dontSendNotification);
    label->attachToComponent(&slider, false);
    label->setJustificationType(juce::Justification::centred);
    label->setBounds(x, y + height + 5, width, 20);

    addAndMakeVisible(*label);
    sliderLabels.push_back(std::move(label));
}

//==============================================================================
void TriquetraAudioProcessorEditor::paint (juce::Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));
}

void TriquetraAudioProcessorEditor::resized()
{
    // This is generally where you'll want to lay out the positions of any
    // subcomponents in your editor..
}
