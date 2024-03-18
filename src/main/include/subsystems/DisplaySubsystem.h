// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/util/Color8Bit.h>
#include <units/time.h>

#include <array>

#include "Constants.h"

using namespace DisplayContants;

class DisplaySubsystem : public frc2::SubsystemBase {
 public:
  DisplaySubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /* The following function is responsible for turning off the LED      */
  /* display.                                                           */
  void DisplayOff(void);

  /* The following function is responsible for turning on the LED       */
  /* display.  The only parameter to this function is the variable      */
  /* containing the color to set the display.                           */
  void DisplayOn(frc::Color8Bit color);

  /* The following function is responsible for setting a pixel of the   */
  /* display to toggle.  The first parameter to this function is the    */
  /* timeout to use in which to toggle the display.  The second         */
  /* parameter to this function is the variable containing the first    */
  /* color to set during the toggle.  The final parameter to this       */
  /* function is the variabvle containing the second color to set during*/
  /* the toggle.                                                        */
  void DisplayToggle(units::second_t toggleTimeout, frc::Color8Bit firstColor, frc::Color8Bit secondColor);

 private:

   /* The following variable holds the display LED controller object.   */
   frc::AddressableLED m_ledController{kDisplayPWMPort};

   /* The following array acts as the LED buffer used to output data to */
   /* the LED display.                                                  */
   std::array<frc::AddressableLED::LEDData, kDisplayLength> m_ledBuffer;

   /* The following variable holds the current toggle enabled state.    */
   bool m_toggleEnabled;

   /* The following variable hold the first and second colors to use    */
   /* when toggling.                                                    */
   frc::Color8Bit m_toggleFirstColor;
   frc::Color8Bit m_toggleSecondColor;

   /* The following variable holds the current toggle color state.      */
   bool m_colorToggler;

   /* The following variable holds the toggle timeout.                  */
   units::second_t m_toggleTimeout;
   units::second_t m_toggleCounter;
};

