/*
 * Copyright (c) 2024-2026 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics.spark.config;

import com.revrobotics.config.BaseConfig;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class ClosedLoopConfig extends BaseConfig {
  public final MAXMotionConfig maxMotion = new MAXMotionConfig();
  public final FeedForwardConfig feedForward = new FeedForwardConfig();

  /** Create a new object to configure a ClosedLoopController. */
  public ClosedLoopConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link ClosedLoopConfig} to this one, including all of its nested
   * configurations.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link ClosedLoopConfig} to copy settings from
   * @return The updated {@link ClosedLoopConfig} for method chaining
   */
  public ClosedLoopConfig apply(ClosedLoopConfig config) {
    super.apply(config);
    this.maxMotion.apply(config.maxMotion);
    this.feedForward.apply(config.feedForward);
    return this;
  }

  /**
   * Applies settings from a {@link MAXMotionConfig} to this {@link ClosedLoopConfig}.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link MAXMotionConfig} to copy settings from
   * @return The updated {@link ClosedLoopConfig} for method chaining
   */
  public ClosedLoopConfig apply(MAXMotionConfig config) {
    this.maxMotion.apply(config);
    return this;
  }

  /**
   * Applies settings from a {@link FeedForwardConfig} to this {@link ClosedLoopConfig}.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link FeedForwardConfig} to copy settings from
   * @return The updated {@link ClosedLoopConfig} for method chaining
   */
  public ClosedLoopConfig apply(FeedForwardConfig config) {
    this.feedForward.apply(config);
    return this;
  }

  /**
   * Set the PIDF gains of the controller. This will set the gains for closed loop slot 0.
   *
   * <p>To set the gains for a specific closed loop slot, use {@link ClosedLoopConfig#pidf(double,
   * double, double, double, ClosedLoopSlot)}.
   *
   * @param p The proportional gain value
   * @param i The integral gain value
   * @param d The derivative gain value
   * @param ff The velocity feedforward value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   * @deprecated Use {@link ClosedLoopConfig#feedForward} to set feedforward gains
   */
  @Deprecated(forRemoval = true)
  public ClosedLoopConfig pidf(double p, double i, double d, double ff) {
    return pidf(p, i, d, ff, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the PIDF gains of the controller for a specific closed loop slot.
   *
   * @param p The proportional gain value
   * @param i The integral gain value
   * @param d The derivative gain value
   * @param ff The velocity feedforward value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   * @deprecated Use {@link ClosedLoopConfig#feedForward} to set feedforward gains
   */
  @Deprecated(forRemoval = true)
  public ClosedLoopConfig pidf(double p, double i, double d, double ff, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kP_0.value + slot.value * 8, (float) p);
    putParameter(SparkParameters.kI_0.value + slot.value * 8, (float) i);
    putParameter(SparkParameters.kD_0.value + slot.value * 8, (float) d);
    putParameter(SparkParameters.kV_0.value + slot.value * 8, (float) ff);
    return this;
  }

  /**
   * Set the PID gains of the controller. This will set the gains for closed loop slot 0.
   *
   * <p>To set the gains for a specific closed loop slot, use {@link ClosedLoopConfig#pid(double,
   * double, double, ClosedLoopSlot)}.
   *
   * @param p The proportional gain value
   * @param i The integral gain value
   * @param d The derivative gain value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig pid(double p, double i, double d) {
    return pid(p, i, d, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the PID gains of the controller for a specific closed loop slot.
   *
   * @param p The proportional gain value
   * @param i The integral gain value
   * @param d The derivative gain value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig pid(double p, double i, double d, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kP_0.value + slot.value * 8, (float) p);
    putParameter(SparkParameters.kI_0.value + slot.value * 8, (float) i);
    putParameter(SparkParameters.kD_0.value + slot.value * 8, (float) d);
    return this;
  }

  /**
   * Set the proportional gain of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#p(double, ClosedLoopSlot)}.
   *
   * @param p The proportional gain value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig p(double p) {
    return p(p, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the proportional gain of the controller for a specific closed loop slot.
   *
   * @param p The proportional gain value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig p(double p, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kP_0.value + slot.value * 8, (float) p);
    return this;
  }

  /**
   * Set the integral gain of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#i(double, ClosedLoopSlot)}.
   *
   * @param i The integral gain value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig i(double i) {
    return i(i, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the integral gain of the controller for a specific closed loop slot.
   *
   * @param i The integral gain value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig i(double i, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kI_0.value + slot.value * 8, (float) i);
    return this;
  }

  /**
   * Set the derivative gain of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#d(double, ClosedLoopSlot)}.
   *
   * @param d The derivative gain value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig d(double d) {
    return d(d, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the derivative gain of the controller for a specific closed loop slot.
   *
   * @param d The derivative gain value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig d(double d, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kD_0.value + slot.value * 8, (float) d);
    return this;
  }

  /**
   * Set the velocity feedforward gain of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#velocityFF(double, ClosedLoopSlot)}.
   *
   * @param ff The velocity feedforward gain value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   * @deprecated Use {@link ClosedLoopConfig#feedForward} to set feedforward gains
   */
  @Deprecated(forRemoval = true)
  public ClosedLoopConfig velocityFF(double ff) {
    return velocityFF(ff, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the velocity feedforward gain of the controller for a specific closed loop slot.
   *
   * @param ff The velocity feedforward gain value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   * @deprecated Use {@link ClosedLoopConfig#feedForward} to set feedforward gains
   */
  @Deprecated(forRemoval = true)
  public ClosedLoopConfig velocityFF(double ff, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kV_0.value + slot.value * 8, (float) ff);
    return this;
  }

  /**
   * Set the derivative filter of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#dFilter(double, ClosedLoopSlot)}.
   *
   * @param dFilter The derivative filter value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig dFilter(double dFilter) {
    return dFilter(dFilter, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the derivative filter of the controller for a specific closed loop slot.
   *
   * @param dFilter The derivative filter value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig dFilter(double dFilter, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kDFilter_0.value + slot.value * 8, (float) dFilter);
    return this;
  }

  /**
   * Set the integral zone of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#dFilter(double, ClosedLoopSlot)}.
   *
   * @param iZone The integral zone value
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig iZone(double iZone) {
    return iZone(iZone, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the integral zone of the controller for a specific closed loop slot.
   *
   * @param iZone The integral zone value
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig iZone(double iZone, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kIZone_0.value + slot.value * 8, (float) iZone);
    return this;
  }

  /**
   * Set the minimum output of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#minOutput(double, ClosedLoopSlot)}.
   *
   * @param minOutput The minimum output value in the range [-1, 1]
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig minOutput(double minOutput) {
    return minOutput(minOutput, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the minimum output of the controller for a specific closed loop slot.
   *
   * @param minOutput The minimum output value in the range [-1, 1]
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig minOutput(double minOutput, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kOutputMin_0.value + slot.value * 8, (float) minOutput);
    return this;
  }

  /**
   * Set the maximum output of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#maxOutput(double, ClosedLoopSlot)}.
   *
   * @param maxOutput The maximum output value in the range [-1, 1]
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig maxOutput(double maxOutput) {
    return maxOutput(maxOutput, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the maximum output of the controller for a specific closed loop slot.
   *
   * @param maxOutput The maximum output value in the range [-1, 1]
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig maxOutput(double maxOutput, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kOutputMax_0.value + slot.value * 8, (float) maxOutput);
    return this;
  }

  /**
   * Set the output range of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#outputRange(double, double, ClosedLoopSlot)}.
   *
   * @param minOutput The minimum output value in the range [-1, 1]
   * @param maxOutput The maximum output value in the range [-1, 1]
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig outputRange(double minOutput, double maxOutput) {
    return outputRange(minOutput, maxOutput, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the output range of the controller for a specific closed loop slot.
   *
   * @param minOutput The minimum output value in the range [-1, 1]
   * @param maxOutput The maximum output value in the range [-1, 1]
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig outputRange(double minOutput, double maxOutput, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kOutputMin_0.value + slot.value * 8, (float) minOutput);
    putParameter(SparkParameters.kOutputMax_0.value + slot.value * 8, (float) maxOutput);
    return this;
  }

  /**
   * Set the maximum I accumulator of the controller. This value is used to constrain the I
   * accumulator to help manage integral wind-up.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link ClosedLoopConfig#iMaxAccum(double, ClosedLoopSlot)}.
   *
   * @param iMaxAccum The max value to constrain the I accumulator to
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig iMaxAccum(double iMaxAccum) {
    return iMaxAccum(iMaxAccum, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the maximum I accumulator of the controller for a specific closed loop slot. This value is
   * used to constrain the I accumulator to help manage integral wind-up.
   *
   * @param iMaxAccum The max value to constrain the I accumulator to
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig iMaxAccum(double iMaxAccum, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kIMaxAccum_0.value + slot.value * 4, (float) iMaxAccum);
    return this;
  }

  /**
   * Set the allowed closed loop error for the controller for a specific PID slot. This value is how
   * much deviation from the setpoint is tolerated and is useful in preventing oscillation around
   * the setpoint. Natively, the units are in rotations but will be affected by the position
   * conversion factor.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified ClosedLoopConfig object for method chaining
   */
  public ClosedLoopConfig allowedClosedLoopError(double allowedError, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kAllowedClosedLoopError_0.value + slot.value * 4, (float) allowedError);
    return this;
  }

  /**
   * Enable or disable PID wrapping for position closed loop control.
   *
   * @param enabled True to enable position PID wrapping
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig positionWrappingEnabled(boolean enabled) {
    putParameter(SparkParameters.kPositionPIDWrapEnable.value, enabled);
    return this;
  }

  /**
   * Set the minimum input value for PID wrapping with position closed loop control.
   *
   * @param minInput The value of min input for the position
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig positionWrappingMinInput(double minInput) {
    putParameter(SparkParameters.kPositionPIDMinInput.value, (float) minInput);
    return this;
  }

  /**
   * Set the maximum input value for PID wrapping with position closed loop control
   *
   * @param maxInput The value of max input for the position
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig positionWrappingMaxInput(double maxInput) {
    putParameter(SparkParameters.kPositionPIDMaxInput.value, (float) maxInput);
    return this;
  }

  /**
   * Set the input range for PID wrapping with position closed loop control
   *
   * @param minInput The value of min input for the position
   * @param maxInput The value of max input for the position
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig positionWrappingInputRange(double minInput, double maxInput) {
    putParameter(SparkParameters.kPositionPIDMinInput.value, (float) minInput);
    putParameter(SparkParameters.kPositionPIDMaxInput.value, (float) maxInput);
    return this;
  }

  /**
   * Set an attached sensor as the feedback sensor of the controller. The controller will use this
   * sensor as the source of feedback for its closed loop control.
   *
   * <p>To set a detached encoder, use {@link #feedbackSensor(FeedbackSensor, int)} or {@link
   * #feedbackSensor(FeedbackSensor, DetachedEncoder)} instead. This method will ignore {@code
   * sensor} set to a Detached sensor.
   *
   * <p>The default feedback sensor is assumed to be the primary encoder for either brushless or
   * brushed mode. This can be changed to another feedback sensor for the controller such as an
   * analog sensor, absolute encoder, or alternate/external encoder.
   *
   * @param sensor The feedback sensor
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig feedbackSensor(FeedbackSensor sensor) {
    switch (sensor) {
      case kNoSensor: // fall-through
      case kPrimaryEncoder: // fall-through
      case kAnalogSensor: // fall-through
      case kAlternateOrExternalEncoder: // fall-through
      case kAbsoluteEncoder:
        putParameter(SparkParameters.kClosedLoopControlSensor.value, sensor.value);
        break;
      default:
        break;
    }
    return this;
  }

  /**
   * Set an external CAN Detached Encoder as the feedback sensor of the controller. The controller
   * will use this sensor as the source of feedback for its closed loop control.
   *
   * <p>To set an attached encoder, use the {@link #feedbackSensor(FeedbackSensor)} method instead.
   * This method will ignore {@code sensor} set to an attached sensor.
   *
   * <p>The default feedback sensor is assumed to be the primary encoder for either brushless or
   * brushed mode. This can be changed to another feedback sensor for the controller such as an
   * analog sensor, absolute encoder, or alternate/external encoder.
   *
   * @param sensor The feedback sensor
   * @param detachedEncoderDeviceId The device ID of the detached CAN encoder to use
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig feedbackSensor(FeedbackSensor sensor, int detachedEncoderDeviceId) {
    switch (sensor) {
      case kDetachedAbsoluteEncoder: // fall-through
      case kDetachedRelativeEncoder:
        putParameter(SparkParameters.kClosedLoopControlSensor.value, sensor.value);
        putParameter(SparkParameters.kDetachedEncoderDeviceID.value, detachedEncoderDeviceId);
        break;
      default:
        break;
    }
    return this;
  }

  /**
   * Set an external CAN Detached Encoder as the feedback sensor of the controller. The controller
   * will use this sensor as the source of feedback for its closed loop control.
   *
   * <p>To set an attached encoder, use the {@link #feedbackSensor(FeedbackSensor)} method instead.
   * This method will ignore {@code sensor} set to an attached sensor.
   *
   * <p>The default feedback sensor is assumed to be the primary encoder for either brushless or
   * brushed mode. This can be changed to another feedback sensor for the controller such as an
   * analog sensor, absolute encoder, or alternate/external encoder.
   *
   * @param sensor The feedback sensor
   * @return The modified {@link ClosedLoopConfig} object for method chaining
   */
  public ClosedLoopConfig feedbackSensor(FeedbackSensor sensor, DetachedEncoder detachedEncoder) {
    return feedbackSensor(sensor, detachedEncoder.getDeviceId());
  }

  @Override
  public String flatten() {
    String flattenedString = "";

    flattenedString += super.flatten();
    flattenedString += maxMotion.flatten();
    flattenedString += feedForward.flatten();

    return flattenedString;
  }
}
