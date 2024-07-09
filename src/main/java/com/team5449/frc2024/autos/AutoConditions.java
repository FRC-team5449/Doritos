// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5449.frc2024.autos;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoConditions {
    private static HashMap<String, BooleanSupplier> conditions=new HashMap<>();

    public static void registerCondition(String name, BooleanSupplier condition) {
        conditions.put(name, condition);
    }

    public static boolean hasCondition(String name) {
        return conditions.containsKey(name);
      }

      public static boolean getCondition(String name) {
    if (hasCondition(name)) {
      return conditions.get(name).getAsBoolean();
    } else {
      DriverStation.reportWarning(
          "PathPlanner is requesting a condition that has not been provided.",
          false);
      return false;
    }
  }
}
