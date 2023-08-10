package com.nextinnovation.lib.controllers;

import com.nextinnovation.lib.utils.Util;

/**
 * This class implements a PID Control Loop.
 *
 * <p>Does all computation synchronously (i.e. to calculate() function must be called by the user
 * from his own thread)
 */
public class SynchronousPIDF {
  private double kp; // factor for "proportional" control
  private double ki; // factor for "integral" control
  private double kd; // factor for "derivative" control
  private double kf; // factor for feed forward gain
  private double maxOutput = 1.0; // |maximum output|
  private double minOutput = -1.0; // |minimum output|
  private double maxInput = Double.POSITIVE_INFINITY; // maximum input - limit setpoint to this
  private double minInput = Double.NEGATIVE_INFINITY; // minimum input - limit setpoint to this
  private boolean isContinuous = false; // do the endpoints wrap around? eg. Absolute encoder
  private boolean isInverted = false;
  private double setpoint = 0.0;
  private double error = 0.0;
  private double previousError = 0.0; // the prior sensor input (used to compute velocity)
  private double previousInput = Double.NaN;
  private double accumulativeError = 0.0; // the sum of the errors for use in the integral calc
  private double output = 0.0;
  private double errorTolerance = 0.0;

  /**
   * Allocate a PID object with the given constants for P, I, D
   *
   * @param kp the proportional coefficient
   * @param ki the integral coefficient
   * @param kd the derivative coefficient
   */
  public SynchronousPIDF(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.kf = 0.0;
  }

  /**
   * Allocate a PID object with the given constants for P, I, D
   *
   * @param kp the proportional coefficient
   * @param ki the integral coefficient
   * @param kd the derivative coefficient
   * @param kf the feed forward gain coefficient
   */
  public SynchronousPIDF(double kp, double ki, double kd, double kf) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.kf = kf;
  }

  /**
   * Read the input, calculate the output accordingly, and write to the output. This should be
   * called at a constant rate by the user (ex. in a timed thread)
   *
   * @param input the input
   * @param deltaTime time passed since previous call to calculate
   * @param isFirstCall whether this call is the first call of the function, in which case deltaTime
   *     would be ignored
   */
  public synchronized double calculate(double input, double deltaTime, boolean isFirstCall) {
    previousInput = input;
    deltaTime = Math.max(deltaTime, Double.MIN_NORMAL);
    error =
        isContinuous
            ? Util.boundToScope(input - setpoint, minInput, maxInput)
            : Util.limit(input - setpoint, minInput, maxInput);
    double proportionalError = Math.abs(error) < errorTolerance ? 0.0 : error;
    double differentialError = isFirstCall ? 0.0 : (error - previousError) / deltaTime;
    if (proportionalError * kp < maxOutput && proportionalError * kp > minOutput && !isFirstCall) {
      accumulativeError += error * deltaTime;
    } else {
      accumulativeError = 0.0;
    }
    output =
        Util.limit(
            Util.conditionalInvert(
                -(kp * proportionalError
                    + ki * accumulativeError
                    + kd * differentialError
                    + kf * setpoint),
                isInverted),
            minOutput,
            maxOutput);
    previousError = error;
    return output;
  }

  /**
   * Read the input, calculate the output accordingly, and write to the output. This should be
   * called at a constant rate by the user (ex. in a timed thread)
   *
   * @param input the input
   * @param deltaTime time passed since previous call to calculate
   */
  public synchronized double calculate(double input, double deltaTime) {
    return calculate(input, deltaTime, false);
  }

  /**
   * Set the PID controller gain parameters. Set the proportional, integral, and differential
   * coefficients.
   *
   * @param kp Proportional coefficient
   * @param ki Integral coefficient
   * @param kd Differential coefficient
   */
  public synchronized void setCoefficients(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  /**
   * Set the PID controller gain parameters. Set the proportional, integral, and differential
   * coefficients.
   *
   * @param kp Proportional coefficient
   * @param ki Integral coefficient
   * @param kd Differential coefficient
   * @param kf Feed forward coefficient
   */
  public synchronized void setCoefficients(double kp, double ki, double kd, double kf) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.kf = kf;
  }

  /**
   * Return the current PID result This is always centered on zero and constrained the the max and
   * min outs
   *
   * @return the latest calculated output
   */
  public double getOutput() {
    return output;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then using the max and
   * min in as constraints, it considers them to be the same point and automatically calculates the
   * shortest route to the setpoint.
   *
   * @param continuous Set to true turns on continuous, false turns off continuous
   */
  public synchronized void setContinuous(boolean continuous) {
    isContinuous = continuous;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then using the max and
   * min in as constraints, it considers them to be the same point and automatically calculates the
   * shortest route to the setpoint.
   */
  public synchronized void setContinuous() {
    this.setContinuous(true);
  }

  public synchronized boolean isContinuous() {
    return isContinuous;
  }

  public synchronized void setInvert(boolean invert) {
    isInverted = invert;
  }

  public boolean isInverted() {
    return isInverted;
  }

  public synchronized void setDeadband(double deadband) {
    this.errorTolerance = deadband;
  }

  /**
   * Sets the maximum and minimum values expected from the input.
   *
   * @param minInput the minimum value expected from the input
   * @param maxInput the maximum value expected from the output
   */
  public synchronized void setInputRange(double minInput, double maxInput) {
    if (minInput > maxInput) {
      // throw new IllegalArgumentException("minInput cannot be greater than
      // maxInput");
      return;
    }
    this.minInput = minInput;
    this.maxInput = maxInput;
    setSetpoint(setpoint);
  }

  /**
   * Sets the minimum and maximum values to write.
   *
   * @param minOutput the minimum value to write to the output
   * @param maxOutput the maximum value to write to the output
   */
  public synchronized void setOutputRange(double minOutput, double maxOutput) {
    if (minOutput > maxOutput) {
      // throw new IllegalArgumentException("minOutput cannot be greater than
      // maxOutput");
      return;
    }
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
  }

  /**
   * Set the setpoint for the PID controller
   *
   * @param setpoint the desired setpoint
   */
  public synchronized void setSetpoint(double setpoint) {
    this.setpoint = Math.min(Math.max(setpoint, minInput), maxInput);
  }

  /**
   * Returns the current setpoint of the PID controller
   *
   * @return the current setpoint
   */
  public synchronized double getSetpoint() {
    return setpoint;
  }

  /**
   * Returns the current difference of the input from the setpoint
   *
   * @return the current error
   */
  public double getError() {
    return error;
  }

  /**
   * Return true if the error is within error tolerance
   *
   * @return true if the error is within error tolerance
   */
  public boolean onTarget() {
    return previousInput != Double.NaN && Math.abs(previousInput - setpoint) < errorTolerance;
  }

  /** Reset all internal terms. */
  public synchronized void reset() {
    previousInput = Double.NaN;
    previousError = 0.0;
    accumulativeError = 0.0;
    output = 0.0;
    setpoint = 0.0;
  }

  public synchronized void resetIntegrator() {
    accumulativeError = 0.0;
  }
}
