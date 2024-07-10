package com.team5449.frc2024.autos;

/** An exception while building autos */
public class AutoBuilderException extends RuntimeException {
  /**
   * Create a new auto builder exception
   *
   * @param message Error message
   */
  public AutoBuilderException(String message) {
    super(message);
  }
}
