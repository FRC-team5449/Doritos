// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.subsystems.vision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  private final AddressableLED mLed;
  private final AddressableLEDBuffer mLedBuffer;
  private final int length = 220;

  public LedSubsystem() {
    mLed = new AddressableLED(6);
    mLedBuffer = new AddressableLEDBuffer(length);
    mLed.setLength(length);
    mLed.setData(mLedBuffer);
    mLed.start();
  }

  @Override
  public void periodic() {
  }

  public void setPureColor(Color c) {
    for (int i = 0; i < length; ++i) {
      mLedBuffer.setLED(i, c);
    }
    mLed.setData(mLedBuffer);
  }

  // public void setProgressive(Color c, int segmentLength, int timeInterval) {
  //   try {
  //     for (int i = 0; i < segmentLength; ++i) {
  //       for (int j = 0; j < length; j += segmentLength) {
  //         mLedBuffer.setLED(j + i, c);
  //       }
  //       mLed.setData(mLedBuffer);
  //       Thread.sleep(timeInterval);
  //     }
  //   } catch (InterruptedException e) {
  //     System.out.println(e);
  //   }
  // }
}
