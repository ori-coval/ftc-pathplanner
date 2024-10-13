package com.pathplanner.lib.util;

import com.pathplanner.lib.missingWpilibClasses.math.MathSharedStore;
import com.pathplanner.lib.missingWpilibClasses.math.kinematics.ChassisSpeeds;

/**
 * Essentially a slew rate limiter for chassis speeds
 *
 * <p>This will properly apply a linear acceleration limit to the chassis speeds instead of applying
 * it separately with 2 X/Y slew rate limiters
 */
public class ChassisSpeedsRateLimiter {
  private double translationRateLimit;
  private double rotationRateLimit;

  private ChassisSpeeds prevVal;
  private double prevTime;

  /**
   * Create a new chassis speeds limiter
   *
   * @param translationRateLimit The linear acceleration limit
   * @param rotationRateLimit The angular acceleration limit
   * @param initialValue The initial chassis speeds value
   */
  public ChassisSpeedsRateLimiter(
      double translationRateLimit, double rotationRateLimit, ChassisSpeeds initialValue) {
    this.translationRateLimit = translationRateLimit;
    this.rotationRateLimit = rotationRateLimit;
    reset(initialValue);
  }

  /**
   * Create a new chassis speeds limiter
   *
   * @param translationRateLimit The linear acceleration limit
   * @param rotationRateLimit The angular acceleration limit
   */
  public ChassisSpeedsRateLimiter(double translationRateLimit, double rotationRateLimit) {
    this(translationRateLimit, rotationRateLimit, new ChassisSpeeds());
  }

  /**
   * Reset the limiter
   *
   * @param value The chassis speeds to reset with
   */
  public void reset(ChassisSpeeds value) {
    this.prevVal = value;
    this.prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Set the acceleration limits
   *
   * @param translationRateLimit Linear acceleration limit
   * @param rotationRateLimit Angular acceleration limit
   */
  public void setRateLimits(double translationRateLimit, double rotationRateLimit) {
    this.translationRateLimit = translationRateLimit;
    this.rotationRateLimit = rotationRateLimit;
  }
}
