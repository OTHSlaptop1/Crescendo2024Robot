// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DisplaySubsystem.h"

using namespace DisplayContants;

DisplaySubsystem::DisplaySubsystem()
{
    /* Initalize the LED Controller by telling it the length of the     */
    /* display.                                                         */
    m_ledController.SetLength(kDisplayLength);

    /* Initialize the display buffer to being off.                      */
    for(int Index=0;Index<kDisplayLength;Index++)
       m_ledBuffer[Index].SetRGB(0, 0, 0);

   /* Set the initial state of the display with the contents of the LED */
   /* buffer.                                                           */
   m_ledController.SetData(m_ledBuffer);

   /* Start the LED controller displaying.                              */
   m_ledController.Start();

   /* Start with toggle disabled.                                       */
   m_toggleEnabled = false;

   /* Initialize the toggle color to off.                               */
   m_toggleFirstColor  = frc::Color8Bit{ 0, 0, 0 };
   m_toggleSecondColor = frc::Color8Bit{ 0, 0, 0 };

   /* Intialize the color toggle to a known state.                      */
   m_colorToggler = true;

   /* Initialize the toggle timeout to a known state.                   */
   m_toggleTimeout = 0_s;
   m_toggleCounter = 0_s;
}

// This method will be called once per scheduler run
void DisplaySubsystem::Periodic()
{
   /* First check to see if toggle is currently enabled.                */
   if(m_toggleEnabled)
   {
      /* Toggling is currently enabled.                                 */

      /* Increase the toggle counter by the amount of time that has pass*/
      /* since the last time thru.                                      */
      m_toggleCounter = m_toggleCounter + 20_ms;

      /* Check to see if the toggle timeout has expired.                */
      if(m_toggleCounter >= m_toggleTimeout)
      {
         /* The toggle timeout has expired.                             */

         /* Determine the current toggler state.                        */
         if(m_colorToggler)
         {
            /* Currently displaying the first toggle color.  Set the    */
            /* display to display the second toggle color.              */
            for(int Index=0;Index<kDisplayLength;Index++)
               m_ledBuffer[Index].SetRGB(m_toggleSecondColor.red, m_toggleSecondColor.green, m_toggleSecondColor.blue);
         }
         else
         {
            /* Currently displaying the second toggle color.  Set the   */
            /* display to the display the first toggle color.           */
            for(int Index=0;Index<kDisplayLength;Index++)
               m_ledBuffer[Index].SetRGB(m_toggleFirstColor.red, m_toggleFirstColor.green, m_toggleFirstColor.blue);
         }

         /* Set the display with the contents of the LED buffer.        */
         m_ledController.SetData(m_ledBuffer);

         /* Reset the toggle counter.                                   */
         m_toggleCounter = 0_s;

         /* Toggle the color toggler.                                   */
         m_colorToggler = !m_colorToggler;
      }
   }
}

   /* The following function is responsible for turning off the LED      */
   /* display.                                                           */
void DisplaySubsystem::DisplayOff(void)
{
   /* First make sure toggle is currently disabled.                     */
   m_toggleEnabled = false;

   /* Set the display buffer to being off.                              */
   for(int Index=0;Index<kDisplayLength;Index++)
      m_ledBuffer[Index].SetRGB(0, 0, 0);

   /* Set the display with the contents of the LED buffer.              */
   m_ledController.SetData(m_ledBuffer);
}

  /* The following function is responsible for turning on the LED       */
  /* display.  The only parameter to this function is the variable      */
  /* containing the color to set the display.                           */
void DisplaySubsystem::DisplayOn(frc::Color8Bit color)
{
   /* First make sure toggle is currently disabled.                     */
   m_toggleEnabled = false;

   /* Set the display buffer to being on and the specified color.       */
   for(int Index=0;Index<kDisplayLength;Index++)
      m_ledBuffer[Index].SetRGB(color.red, color.green, color.blue);

   /* Set the display with the contents of the LED buffer.              */
   m_ledController.SetData(m_ledBuffer);
}

  /* The following function is responsible for setting a pixel of the   */
  /* display to toggle.  The first parameter to this function is the    */
  /* timeout to use in which to toggle the pixel.  The second parameter */
  /* to this function is the variable containing the first color to set */
  /* during the toggle.  The final parameter to this function is the    */
  /* variabvle containing the second color to set during the toggle.    */
void DisplaySubsystem::DisplayToggle(units::second_t toggleTimeout, frc::Color8Bit firstColor, frc::Color8Bit secondColor)
{
   /* Store the toggle timeout to be used for the toggle rate.          */
   m_toggleTimeout = toggleTimeout;
   m_toggleCounter = 0_s;

   /* Save the first and second colors to be displayed during the       */
   /* toggle.                                                           */
   m_toggleFirstColor  = firstColor;
   m_toggleSecondColor = secondColor;

   /* Initialize the display buffer using the first color specified.    */
   for(int Index=0;Index<kDisplayLength;Index++)
      m_ledBuffer[Index].SetRGB(m_toggleFirstColor.red, m_toggleFirstColor.green, m_toggleFirstColor.blue);

   /* Set the display with the contents of the LED buffer.              */
   m_ledController.SetData(m_ledBuffer);

   /* Reset the color toggle to indicate that the first color is        */
   /* currently being displayed.                                        */
   m_colorToggler = true;

   /* Set the toggle to be enabled.                                     */
   m_toggleEnabled = true;
}
