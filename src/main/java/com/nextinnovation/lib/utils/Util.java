package com.nextinnovation.lib.utils;

import com.nextinnovation.lib.geometry.Translation2d;

public class Util {
  public static final double EPSILON_VALUE = 1e-12;

  private Util() {}

  public static int limit(int val, int min, int max) {
    return Math.min(max, Math.max(min, val));
  }

  public static int limit(int val, int maxMagnitude) {
    return limit(val, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double val, double min, double max) {
    return Math.min(max, Math.max(min, val));
  }

  public static double limit(double val, double maxMagnitude) {
    return limit(val, -maxMagnitude, maxMagnitude);
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  public static boolean epsilonEquals(double lhs, double rhs) {
    return epsilonEquals(lhs, rhs, EPSILON_VALUE);
  }

  public static boolean epsilonEquals(int lhs, int rhs, int epsilon) {
    return Math.abs(lhs - rhs) <= epsilon;
  }

  public static boolean epsilonEquals(double lhs, double rhs, double epsilon) {
    return Math.abs(lhs - rhs) <= epsilon;
  }

  public static int roundToInt(double val) {
    return Math.toIntExact(Math.round(val));
  }

  public static int conditionalInvert(int val, boolean invert) {
    return invert ? -val : val;
  }

  public static double conditionalInvert(double val, boolean invert) {
    return invert ? -val : val;
  }

  public static int boundToScope(int val, int scopeFloor, int scopeCeiling) {
    int scopeSpan = scopeCeiling - scopeFloor;
    val %= scopeSpan;
    val +=
        val >= 0.0 ? scopeFloor - scopeFloor % scopeSpan : scopeCeiling - scopeCeiling % scopeSpan;
    if (val < scopeFloor) {
      val += scopeSpan;
    } else if (val >= scopeCeiling) {
      val -= scopeSpan;
    }
    return val;
  }

  public static double boundToScope(double val, double scopeFloor, double scopeCeiling) {
    double scopeSpan = scopeCeiling - scopeFloor;
    val %= scopeSpan;
    val +=
        val >= 0.0 ? scopeFloor - scopeFloor % scopeSpan : scopeCeiling - scopeCeiling % scopeSpan;
    if (val < scopeFloor) {
      val += scopeSpan;
    } else if (val >= scopeCeiling) {
      val -= scopeSpan;
    }
    return val;
  }

  public static int boundToNonnegativeScope(int val, int scopeCeiling) {
    val %= scopeCeiling;
    if (val < 0) {
      val += scopeCeiling;
    }
    return val;
  }

  public static double boundToNonnegativeScope(double val, double scopeCeiling) {
    val %= scopeCeiling;
    if (val < 0.0) {
      val += scopeCeiling;
    }
    return val;
  }

  public static double boundAngleToNegative180To180Degrees(double angle) {
    angle %= 360.0;
    if (angle <= -180.0) {
      angle += 360.0;
    } else if (angle > 180.0) {
      angle -= 360.0;
    }
    return angle;
  }

  public static double boundAngleToNegative180To180DegreesWithCompensate(
      double angle, double compensate_angle) {
    return boundAngleToScopeWithCompensate(angle, -180.0, 180.0, compensate_angle);
  }

  public static double boundAngleToScopeWithCompensate(
      double angle, double start_angle, double end_angle, double compensate_angle) {
    angle %= 360.0;

    if (angle <= start_angle - compensate_angle) {
      angle += 360.0;
    } else if (angle >= end_angle + compensate_angle) {
      angle -= 360.0;
    }

    return angle;
  }

  public static double boundAngleTo0To360Degrees(double angle) {
    return boundToNonnegativeScope(angle, 360.0);
  }

  public static double boundAngleTo0ToNegative360Degrees(double angle) {
    return boundToNonnegativeScope(angle, 360.0) - 360.0;
  }

  public static double boundAngleTo360OrNegative360ByCopySign(
      double target_angle, double currant_angle) {
    if (boundAngleToNegative180To180Degrees(currant_angle) <= 0.0) {
      return boundAngleTo0ToNegative360Degrees(target_angle);
    } else {
      return boundAngleTo0To360Degrees(target_angle);
    }
  }

  public static double boundAngleToClosestScope(double angle, double referenceAngle) {
    return boundAngleToNegative180To180Degrees(angle - referenceAngle) + referenceAngle;
  }

  public static double boundAngleToClosestScopeWithinLimit(
      double angle, double referenceAngle, double minAngle, double maxAngle) {
    return boundToScope(boundAngleToClosestScope(angle, referenceAngle), minAngle, maxAngle);
  }

  public static double applyDeadBand(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static double applyRemappedDeadband(double val, double deadband) {
    double absDeadband = Math.abs(deadband);
    return (Math.abs(val) > absDeadband)
        ? (val - Math.copySign(absDeadband, val)) / (1 - absDeadband)
        : 0.0;
  }

  public static Translation2d applyCircularDeadband(Translation2d input, double deadband) {
    return input.norm() > Math.abs(deadband) ? input : Translation2d.identity();
  }

  public static Translation2d applyRemappedCircularDeadband(Translation2d input, double deadband) {
    double absDeadband = Math.abs(deadband);
    return input.norm() > absDeadband
        ? input
            .translateBy(Translation2d.fromPolar(input.direction(), -absDeadband))
            .scale(1 / (1 - absDeadband))
        : Translation2d.identity();
  }

  public static double applyRestrictedZone(double val, double start, double end) {
    if (start <= val && val <= end) {
      val = val - start < end - val ? start : end;
    }
    return val;
  }

  public static double applyAngularRestrictedZone(double angle, double startAngle, double span) {
    if (span < 0.0) {
      startAngle += span;
      span = -span;
    }
    startAngle = boundAngleTo0To360Degrees(startAngle) + ((long) (angle / 360.0)) * 360.0;
    if (startAngle > angle) {
      startAngle -= 360.0;
    }
    return applyRestrictedZone(angle, startAngle, startAngle + span);
  }

  public static boolean shouldReverseRotation(double targetAngle, double currentAngle) {
    return Math.abs(boundAngleToNegative180To180Degrees(targetAngle - currentAngle)) > 90.0;
  }
}
