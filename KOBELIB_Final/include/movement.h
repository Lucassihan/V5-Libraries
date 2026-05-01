#include "auton.h"
#include "cmath"
#include "globals.h"
#include "movement_globals.h"
#include "odometry.h"
#include "pid.h"
#include "vex.h"

class movement {
private:
  bool first = false;
  bool hasExitedTurnOnly = false;

public:
  float leftpower, rightpower, turndisablelength = 3;
  double movementtype = 0;
  double lastmovementtype = 0;
  bool autondriveenabled = false;
  bool cataEnabled = false;
  bool shoot = false;
  float cataTarg = 136;

  bool neutralMode = false;
  // was 140 (perfect low)
  // was 68 on old rotation
  // was 69 (perfect low)
  // was 315
  float x_point = 0, y_point = 0;
  bool backwards = false;
  int turnDir = 0; // 0 = auto, 1 = CW, 2 = CCW
  bool ptpCosineScaling = false;

  float intakeTarg = 0;

  bool seamlessTransitions = false;

  // ===== Boomerang state =====
  float boomerang_target_x = 0;
  float boomerang_target_y = 0;
  float boomerang_target_heading = 0;
  bool boomerang_backwards = false;
  float boomerang_lead = 0.6;
  float boomerang_setback = 0;
  float boomerang_settle_error = 3;
  float boomerang_drive_max_voltage = 12.7;
  float boomerang_heading_max_voltage = 12.7;
  float boomerang_min_voltage = 0;
  float boomerang_timeout = 0;
  float boomerang_start_time = 0;

  bool boomerang_crossed_end_line = false;
  bool boomerang_prev_end_line_side = false;
  bool boomerang_end_line_initialized = false;
  bool boomerang_crossed_center_line = false;
  bool boomerang_prev_center_side = false;
  bool boomerang_center_initialized = false;
  bool boomerang_settled = false;
  bool boomerang_timed_out = false;
  bool boomerang_in_final_phase = false;
  float boomerang_settle_count = 0;

  // ===== Inception Boomerang state =====
  float ib_target_x = 0;
  float ib_target_y = 0;
  float ib_target_heading = 0;   // desired final heading in degrees
  bool ib_backwards = false;

  // Carrot geometry
  float ib_dLead = 15.0;         // fixed distance behind endpoint for initial carrot (inches)
  float ib_gLead = 0.5;          // ghost interpolation factor (0 = ghost tracks carrot, 1 = ghost stays at initial carrot)

  // Initial carrot position (set once at start, never changes)
  float ib_initial_carrot_x = 0;
  float ib_initial_carrot_y = 0;

  // Current (retracted) carrot position
  float ib_carrot_x = 0;
  float ib_carrot_y = 0;

  // Carrot retraction tracking
  float ib_min_carrot_dist = 0;   // smallest distance ever achieved to carrot
  float ib_init_carrot_dist = 0;  // distance to carrot at movement start

  // Phase thresholds (inches) — tune these
  float ib_close_end_dist = 8.0;     // distance to carrot that triggers "near" phase
  float ib_close_ghost_dist = 20.0;  // distance to ghost that triggers "mid" phase
  float ib_parallel_dist = 8.0;      // distance to endpoint below which heading switches from face(end) to parallel(end)

  // Phase latches
  bool ib_close_end = false;
  bool ib_close_ghost = false;

  // PID voltage limits
  float ib_drive_max_voltage = 12.0;
  float ib_heading_max_voltage = 12.0;
  float ib_min_voltage = 0;

  // Settlement
  float ib_settle_error = 3.0;   // inches — distance threshold for settle counting
  float ib_settle_heading = 5.0; // degrees — heading threshold for settle counting
  float ib_settle_count = 0;
  bool ib_settled = false;

  // Timeout
  float ib_timeout = 0;
  float ib_start_time = 0;
  bool ib_timed_out = false;

  // Perp line cross
  bool ib_crossed_end_line = false;
  bool ib_prev_end_line_side = false;
  bool ib_end_line_initialized = false;

  // ===== Arc state =====
  float arc_cx = 0, arc_cy = 0;
  float arc_radius = 0;
  float arc_speed = 0;
  int arc_direction = 1; // +1 = CW (right turn), -1 = CCW (left turn)
  bool arc_backward = false;

  float turnPIDinput(float targ) { return chassis.wrapangle(targ - chassis.h); }

  float applyTurnDir(float error) {
    if (turnDir != 0 && fabs(error) <= 5) turnDir = 0; // passed through target, stop forcing direction
    if (turnDir == 1 && error < 0) error += 360;      // force CW
    else if (turnDir == 2 && error > 0) error -= 360;  // force CCW
    return error;
  }

  void spotTurnCalc() {
    float pow = turnPID.calculate(applyTurnDir(turnPIDinput(chassis.htarg)));

    if (partialOutputSide == 0) {
      leftpower = pow, rightpower = -1 * pow;
    }

    if (partialOutputSide == 1) {
      leftpower = pow * partialOutput, rightpower = -1 * pow;
    }

    if (partialOutputSide == 2) {
      leftpower = pow, rightpower = -1 * pow * partialOutput;
    }

  }

  void spotTurnCalctoPoint() {
    chassis.htarg = chassis.pointangle(x_point, y_point);
    float pow = 0;
    if (!backwards)
      pow = turnPID.calculate(applyTurnDir(turnPIDinput(chassis.htarg)));
    if (backwards)
      pow = turnPID.calculate(applyTurnDir(turnPIDinput(chassis.htarg + 180)));
    if (partialOutputSide == 0) {
      leftpower = pow, rightpower = -1 * pow;
    } else if (partialOutputSide == 1) {
      leftpower = pow * partialOutput, rightpower = -1 * pow;
    } else if (partialOutputSide == 2) {
      leftpower = pow, rightpower = -1 * pow * partialOutput;
    }
  }
  void swingOnLeftCalc() {
    float pow = swingPID.calculate(applyTurnDir(turnPIDinput(chassis.htarg)));
    leftpower = pow, rightpower = pow * sweep_ratio;
  }
  void swingOnLefttoPointCalc() {
    float pow =
        swingPID.calculate(applyTurnDir(turnPIDinput(chassis.pointangle(x_point, y_point))));
    if (backwards)
      pow = swingPID.calculate(
          applyTurnDir(turnPIDinput(chassis.pointangle(x_point, y_point) + 180)));
    leftpower = -pow * sweep_ratio, rightpower = -pow;
  }
  void swingOnRightCalc() {
    float pow = swingPID.calculate(applyTurnDir(turnPIDinput(chassis.htarg)));
    leftpower = -pow * sweep_ratio, rightpower = -pow;
  }

  void swingOnRighttoPointCalc() {
    float pow =
        swingPID.calculate(applyTurnDir(turnPIDinput(chassis.pointangle(x_point, y_point))));
    if (backwards)
      pow = swingPID.calculate(
          applyTurnDir(turnPIDinput(chassis.pointangle(x_point, y_point) + 180)));
    leftpower = pow, rightpower = pow * sweep_ratio;
  }

  float ypower(float yTarg, float xTarg) {
    float Yerror = yTarg - chassis.y; // error on Y axis
    float Xerror = xTarg - chassis.x; // error on X axis
    return Yerror * cos(chassis.hradians) +
           Xerror * sin(chassis.hradians); // KP base
  }

  float straightpower(float targ, float start) {
    straighterror = (startingtrackervalue + targ) -
                    (chassis.paralleltracker * tyD * M_PI / 360);
    // changed for 2 inch tracking wheel
    // TRACKER WHEEL CONFIGURATION HERE
    return straighterror;
  }

  void straightlinecalc() {
    float pow = drivePID.calculate(
        straightpower(straightlinegoal, startingtrackervalue));
    leftpower = pow, rightpower = pow;
  }

  void straightheadingcalc() {
    float pow = drivePID.calculate(
        straightpower(straightlinegoal, startingtrackervalue));
    float hpow = turnPID.calculate(turnPIDinput(chassis.htarg));
    // TURN-PRIORITY DESATURATION: if drive + turn exceeds 12.7V, reduce drive
    // to make room for turn. Prevents drivePID.minOutput from starving the turn.
    // FUTURE: replace with cosine heading scaling (ypow *= cos(headingError))
    // for smoother behavior, keep this as a hard backstop.
    float maxNeeded = fabs(pow) + fabs(hpow);
    if (maxNeeded > 12.7) {
      float available = 12.7 - fabs(hpow);
      if (available < 0) available = 0;
      pow = available * signum(pow);
    }
    leftpower = pow + hpow, rightpower = pow - hpow;
  }

  void straightheadingcalcToPoint() {
    float pow = drivePID.calculate(
        straightpower(straightlinegoal, startingtrackervalue));
    float hpow =
        turnPID.calculate(turnPIDinput(chassis.pointangle(x_point, y_point)));
    if (backwards)
      hpow = turnPID.calculate(
          turnPIDinput(chassis.pointangle(x_point, y_point) + 180));
    // TURN-PRIORITY DESATURATION (see straightheadingcalc for details)
    float maxNeeded = fabs(pow) + fabs(hpow);
    if (maxNeeded > 12.7) {
      float available = 12.7 - fabs(hpow);
      if (available < 0) available = 0;
      pow = available * signum(pow);
    }
    leftpower = pow + hpow, rightpower = pow - hpow;
  }

  void boomerangCalc() {
    // Step 1: Distance to target
    float dx_target = boomerang_target_x - chassis.x;
    float dy_target = boomerang_target_y - chassis.y;
    float target_distance = sqrt(dx_target * dx_target + dy_target * dy_target);

    // Step 2: End-line check (perpendicular to target heading, through target)
    // Only activate within 15 inches of target
    float target_angle_rad = TO_RAD(boomerang_target_heading);
    if (target_distance < 15) {
      bool end_line_side = (dy_target * cos(target_angle_rad)) <=
                           -(dx_target * sin(target_angle_rad));

      if (!boomerang_end_line_initialized) {
        boomerang_prev_end_line_side = end_line_side;
        boomerang_end_line_initialized = true;
      } else if (end_line_side && !boomerang_prev_end_line_side) {
        boomerang_crossed_end_line = true;
      }
      boomerang_prev_end_line_side = end_line_side;
    }

    // Step 2b: Center-line check (perpendicular to end line, through target)
    float center_angle_rad = TO_RAD(boomerang_target_heading + 90);
    bool center_side = (dy_target * cos(center_angle_rad)) <=
                       -(dx_target * sin(center_angle_rad));

    if (!boomerang_center_initialized) {
      boomerang_prev_center_side = center_side;
      boomerang_center_initialized = true;
    } else if (center_side != boomerang_prev_center_side) {
      boomerang_crossed_center_line = true; // latching
    }

    // Step 3: Compute carrot point (behind target along target heading)
    float pullback = boomerang_lead * target_distance + boomerang_setback;
    float carrot_x = boomerang_target_x - sin(target_angle_rad) * pullback;
    float carrot_y = boomerang_target_y - cos(target_angle_rad) * pullback;

    // Step 4: Drive error and heading error to carrot
    float dx_carrot = carrot_x - chassis.x;
    float dy_carrot = carrot_y - chassis.y;
    float drive_error = sqrt(dx_carrot * dx_carrot + dy_carrot * dy_carrot);

    // atan2(dx, dy) = 0 deg along +Y axis, matching chassis.pointangle()
    // convention
    float angle_to_carrot = TO_DEG(atan2(dx_carrot, dy_carrot));
    float heading_error;
    if (boomerang_backwards) {
      heading_error = chassis.wrapangle(angle_to_carrot + 180 - chassis.h);
    } else {
      heading_error = chassis.wrapangle(angle_to_carrot - chassis.h);
    }

    // Step 5: Phase switch — when close, target final pose directly
    if (drive_error < boomerang_settle_error || boomerang_crossed_center_line ||
        drive_error < boomerang_setback) {
      boomerang_in_final_phase = true;
    }

    if (boomerang_in_final_phase) {
      drive_error = target_distance;
      heading_error = chassis.wrapangle(boomerang_target_heading - chassis.h);
    }

    // Step 6: Drive PID
    float drive_output = drivePID.calculate(drive_error);
    if (boomerang_backwards)
      drive_output = -drive_output;

    // Step 7: Heading scale — reduce drive when pointed wrong direction
    float heading_scale = cos(TO_RAD(heading_error));
    drive_output *= heading_scale;

    // Step 8: Heading PID
    float heading_output = turnPID.calculate(heading_error);

    // Step 9: Clamp outputs
    float drive_clamp = fabs(heading_scale) * boomerang_drive_max_voltage;
    if (drive_clamp < 0.01)
      drive_clamp = 0.01;
    if (fabs(drive_output) > drive_clamp) {
      drive_output = drive_clamp * signum(drive_output);
    }
    if (fabs(heading_output) > boomerang_heading_max_voltage) {
      heading_output = boomerang_heading_max_voltage * signum(heading_output);
    }
    // Min voltage: if drive is nonzero but below min, bump to min
    // Only apply when robot is roughly aligned (heading_scale > 0.7, i.e. ~<45 deg error)
    // to avoid forcing drive output when pointed sideways and bypassing heading scale
    if (boomerang_min_voltage > 0 && fabs(heading_scale) > 0.7 &&
        fabs(drive_output) > 0.01 &&
        fabs(drive_output) < boomerang_min_voltage) {
      drive_output = boomerang_min_voltage * signum(drive_output);
    }

    // Step 10: Combine and saturation scale
    float left_output = drive_output + heading_output;
    float right_output = drive_output - heading_output;

    float max_motor = fabs(left_output);
    if (fabs(right_output) > max_motor)
      max_motor = fabs(right_output);
    if (max_motor > 12.0) {
      float ratio = 12.0 / max_motor;
      left_output *= ratio;
      right_output *= ratio;
    }

    leftpower = left_output;
    rightpower = right_output;

    // Step 11: Settlement tracking
    if (target_distance < boomerang_settle_error &&
        fabs(chassis.wrapangle(boomerang_target_heading - chassis.h)) < 5.0) {
      boomerang_settle_count += 10;
      if (boomerang_settle_count > 200) {
        boomerang_settled = true;
      }
    } else {
      boomerang_settle_count = 0;
    }

    // Timeout check
    if (boomerang_timeout > 0 &&
        (Brain.timer(msec) - boomerang_start_time) > boomerang_timeout) {
      boomerang_timed_out = true;
    }
  }

  void inceptionBoomerangCalc() {
    // ---- Current pose ----
    float dx_target = ib_target_x - chassis.x;
    float dy_target = ib_target_y - chassis.y;
    float target_distance = sqrt(dx_target * dx_target + dy_target * dy_target);

    // ---- Perp line cross (only within 15 in) ----
    if (target_distance < 15) {
      float target_angle_rad_el = TO_RAD(ib_target_heading);
      bool end_line_side = (dy_target * cos(target_angle_rad_el)) <=
                           -(dx_target * sin(target_angle_rad_el));

      if (!ib_end_line_initialized) {
        ib_prev_end_line_side = end_line_side;
        ib_end_line_initialized = true;
      } else if (end_line_side && !ib_prev_end_line_side) {
        ib_crossed_end_line = true;
      }
      ib_prev_end_line_side = end_line_side;
    }

    // ---- Carrot retraction ----
    // Track the minimum distance ever achieved to the carrot
    float dx_carrot = ib_carrot_x - chassis.x;
    float dy_carrot = ib_carrot_y - chassis.y;
    float carrot_distance = sqrt(dx_carrot * dx_carrot + dy_carrot * dy_carrot);

    if (carrot_distance < ib_min_carrot_dist) {
      ib_min_carrot_dist = carrot_distance;
    }

    // Retract carrot toward endpoint proportionally
    float retract_ratio = (ib_init_carrot_dist > 0.001)
                            ? ib_min_carrot_dist / ib_init_carrot_dist
                            : 0.0;
    float target_angle_rad = TO_RAD(ib_target_heading);
    ib_carrot_x = ib_target_x - retract_ratio * sin(target_angle_rad) * ib_dLead;
    ib_carrot_y = ib_target_y - retract_ratio * cos(target_angle_rad) * ib_dLead;

    // Recompute carrot distance after retraction
    dx_carrot = ib_carrot_x - chassis.x;
    dy_carrot = ib_carrot_y - chassis.y;
    carrot_distance = sqrt(dx_carrot * dx_carrot + dy_carrot * dy_carrot);

    // ---- Ghost point ----
    // Interpolate between initial carrot and current (retracted) carrot
    // gLead=0: ghost = carrot (no lag), gLead=1: ghost = initial carrot (max lag)
    float ghost_x = ib_initial_carrot_x + (ib_carrot_x - ib_initial_carrot_x) * (1.0 - ib_gLead);
    float ghost_y = ib_initial_carrot_y + (ib_carrot_y - ib_initial_carrot_y) * (1.0 - ib_gLead);

    float dx_ghost = ghost_x - chassis.x;
    float dy_ghost = ghost_y - chassis.y;
    float ghost_distance = sqrt(dx_ghost * dx_ghost + dy_ghost * dy_ghost);

    // ---- 3-phase target selection ----
    // Phases latch: once a phase activates, the robot never goes back.
    // Phase 1 (far):  steer toward ghost
    // Phase 2 (mid):  steer toward carrot (ghost is close)
    // Phase 3 (near): steer toward endpoint, then parallel to target heading

    float heading_error = 0;
    float drive_error = 0;

    // Check phase transitions (latching)
    if (carrot_distance < ib_close_end_dist) {
      ib_close_end = true;
    }
    if (!ib_close_end && ghost_distance < ib_close_ghost_dist) {
      ib_close_ghost = true;
    }

    if (ib_close_end) {
      // PHASE 3 — Near endpoint
      drive_error = target_distance;

      if (target_distance < ib_parallel_dist) {
        // Very close: align heading parallel to target heading (stop pointing at endpoint)
        heading_error = chassis.wrapangle(ib_target_heading - chassis.h);
      } else {
        // Close but not there yet: face the endpoint
        float angle_to_end = TO_DEG(atan2(dx_target, dy_target));
        if (ib_backwards) {
          heading_error = chassis.wrapangle(angle_to_end + 180 - chassis.h);
        } else {
          heading_error = chassis.wrapangle(angle_to_end - chassis.h);
        }
      }
    } else if (ib_close_ghost) {
      // PHASE 2 — Mid: face the carrot
      drive_error = carrot_distance;

      float angle_to_carrot = TO_DEG(atan2(dx_carrot, dy_carrot));
      if (ib_backwards) {
        heading_error = chassis.wrapangle(angle_to_carrot + 180 - chassis.h);
      } else {
        heading_error = chassis.wrapangle(angle_to_carrot - chassis.h);
      }
    } else {
      // PHASE 1 — Far: face the ghost
      drive_error = carrot_distance;

      float angle_to_ghost = TO_DEG(atan2(dx_ghost, dy_ghost));
      if (ib_backwards) {
        heading_error = chassis.wrapangle(angle_to_ghost + 180 - chassis.h);
      } else {
        heading_error = chassis.wrapangle(angle_to_ghost - chassis.h);
      }
    }

    // ---- Distance error switching ----
    // In closeEnd phase, or if robot has passed the carrot, use distance to endpoint
    if (ib_close_end || target_distance < carrot_distance) {
      drive_error = target_distance;
    } else {
      drive_error = carrot_distance;
    }

    // ---- Drive PID ----
    float drive_output = drivePID.calculate(drive_error);
    if (ib_backwards) drive_output = -drive_output;

    // ---- Cosine heading scaling (ONLY in near/endpoint phase) ----
    if (ib_close_end && target_distance < ib_parallel_dist) {
      float heading_scale = cos(TO_RAD(heading_error));
      drive_output *= heading_scale;
    }

    // ---- Heading PID ----
    float heading_output = turnPID.calculate(heading_error);

    // ---- Clamp outputs ----
    if (fabs(drive_output) > ib_drive_max_voltage) {
      drive_output = ib_drive_max_voltage * signum(drive_output);
    }
    if (fabs(heading_output) > ib_heading_max_voltage) {
      heading_output = ib_heading_max_voltage * signum(heading_output);
    }

    // Min voltage: only when roughly aligned (|heading_error| < ~45 deg)
    if (ib_min_voltage > 0 && fabs(cos(TO_RAD(heading_error))) > 0.7 &&
        fabs(drive_output) > 0.01 && fabs(drive_output) < ib_min_voltage) {
      drive_output = ib_min_voltage * signum(drive_output);
    }

    // ---- Angular priority rescaling ----
    // If combined output exceeds 12V, reduce drive to preserve heading correction
    float rescale = fabs(drive_output) + fabs(heading_output) - 12.0;
    if (rescale > 0) {
      drive_output -= rescale * signum(drive_output);
    }

    // ---- Combine to left/right ----
    float left_output = drive_output + heading_output;
    float right_output = drive_output - heading_output;

    // Ratio-preserving saturation scaling (max speed enforcement)
    float max_motor = fmax(fabs(left_output), fabs(right_output));
    if (max_motor > ib_drive_max_voltage) {
      float ratio = ib_drive_max_voltage / max_motor;
      left_output *= ratio;
      right_output *= ratio;
    }

    leftpower = left_output;
    rightpower = right_output;

    // ---- Settlement tracking ----
    if (target_distance < ib_settle_error &&
        fabs(chassis.wrapangle(ib_target_heading - chassis.h)) < ib_settle_heading) {
      ib_settle_count += 10;
      if (ib_settle_count > 200) {
        ib_settled = true;
      }
    } else {
      ib_settle_count = 0;
    }

    // ---- Timeout ----
    if (ib_timeout > 0 &&
        (Brain.timer(msec) - ib_start_time) > ib_timeout) {
      ib_timed_out = true;
    }
  }

  void simplePPSCalc() {
    float turnError = turnPIDinput(chassis.pointangle());
    float cosVal = 1.0f;
    if (ptpCosineScaling && hasExitedTurnOnly) {
      cosVal = cos(TO_RAD(chassis.relativeangle()));
      if (cosVal < 0 && turnDir != 0) {
        turnError = applyTurnDir(turnError);
      }
    }
    float pow = turnPID.calculate(turnError);
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (fabs(chassis.relativeangle()) > turnMargin && chassis.pointdist() > 5 && !hasExitedTurnOnly) {
      if (turnType == 0) {
        //Spot turn
        leftpower = pow;
        rightpower = -1 * pow;
      } else if (turnType == 1) {
        //Swing on left
        leftpower = 0;
        rightpower = -1 * pow;
      } else if (turnType == 2) {
        //Swing on right
        leftpower = pow;
        rightpower = 0;
      }

    } else if (chassis.pointdist() > turndisablelength) {
      hasExitedTurnOnly = true;
      if (ptpCosineScaling) {
        ypow *= fabs(cosVal);
      }
      leftpower = ypow + pow, rightpower = ypow - pow;
      first = true;

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.cartesianX();
      chassis.ytarg = chassis.y + chassis.cartesianY();
      leftpower = ypow, rightpower = ypow;

      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }
  void simplePPSCalcBackward() {
    float turnError = turnPIDinput(chassis.pointangle() - 180);
    float cosVal = 1.0f;
    if (ptpCosineScaling && hasExitedTurnOnly) {
      cosVal = cos(TO_RAD(chassis.relativeangle() - 180));
      if (cosVal < 0 && turnDir != 0) {
        turnError = applyTurnDir(turnError);
      }
    }
    float pow = turnPID.calculate(turnError);
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (fabs(chassis.relativeangle()) < (180 - turnMargin) &&
        chassis.pointdist() > 5 && !hasExitedTurnOnly) {
      if (turnType == 0) {
        //Spot turn
        leftpower = pow;
        rightpower = -1 * pow;
      } else if (turnType == 1) {
        //Swing on left
        leftpower = 0;
        rightpower = -1 * pow;
      } else if (turnType == 2) {
        //Swing on right
        leftpower = pow;
        rightpower = 0;
      }

    } else if (chassis.pointdist() > turndisablelength) {
      hasExitedTurnOnly = true;
      if (ptpCosineScaling) {
        ypow *= fabs(cosVal);
      }
      leftpower = ypow + pow, rightpower = ypow - pow;
      first = true;

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.bcartesianX();
      chassis.ytarg = chassis.y + chassis.bcartesianY();
      leftpower = ypow, rightpower = ypow;

      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  // Pure pursuit (Boil)
  float BLRSCurvature(float speed) {
    // radius = (0.5 * lookahead distance) / sin(turn error)
    // turn vel = ((W*sin(turn error)) / lookahead distance) * linearVel
    float turnError = chassis.wrapangle(chassis.relativeangle());
    turnError = TO_RAD(turnError);
    float turnVel =
        ((trackwidth * sin(turnError)) / chassis.pointdist()) * (speed);
    float mult = 1;
    float initialSpeed = fabs(turnVel) + fabs(speed);
    if (initialSpeed > 100) {
      mult = 100 / initialSpeed;
    }
    if (mult < 1) {
      turnVel = ((trackwidth * sin(turnError)) / chassis.pointdist()) *
                (speed * mult);
    }

    return turnVel;
  }

  // Pure Pursuit (Boil)
  float BLRSCurvatureBackwards(float speed) {
    // radius = (0.5 * lookahead distance) / sin(turn error)
    // turn vel = ((W*sin(turn error)) / lookahead distance) * linearVel
    float sinTheta = 0;
    float relativeAng = chassis.wrapangle(chassis.relativeangle());
    sinTheta = sin(relativeAng * M_PI / 180);
    float turnVel = ((trackwidth * sinTheta) / chassis.pointdist()) * (speed);
    float mult = 1;
    float initialSpeed = fabs(turnVel) + fabs(speed);
    if (initialSpeed > 100) {
      mult = 100 / initialSpeed;
    }
    if (mult < 1) {
      turnVel =
          ((trackwidth * sinTheta) / chassis.pointdist()) * (speed * mult);
    }

    return turnVel;
  }

  float turnMargin = 80;
  float turnType = 0;
  void PPSCalc() {
    float pow = turnPID.calculate(turnPIDinput(chassis.pointangle()));
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (chassis.pointdist() > turndisablelength) {

      if (fabs(chassis.relativeangle()) > turnMargin &&
          chassis.pointdist() > 5) {
        leftpower = pow;
        rightpower = -1 * pow;
      } else {
        leftpower = ypow + BLRSCurvature(ypow);
        rightpower = ypow - BLRSCurvature(ypow);

        first = true;
      }
    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.cartesianX();
      chassis.ytarg = chassis.y + chassis.cartesianY();
      leftpower = ypow, rightpower = ypow;
      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  void PPSCalcBackward() {
    float pow = turnPID.calculate(turnPIDinput(chassis.pointangle() - 180));
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));

    if (chassis.pointdist() > turndisablelength) {

      if (fabs(chassis.relativeangle()) < (180 - turnMargin)) {
        leftpower = pow;
        rightpower = -1 * pow;
      } else {
        leftpower = ypow + BLRSCurvatureBackwards(ypow);
        rightpower = ypow - BLRSCurvatureBackwards(ypow);
        first = true;
      }

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.bcartesianX();
      chassis.ytarg = chassis.y + chassis.bcartesianY();
      leftpower = ypow, rightpower = ypow;
      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  void simplePPSBezierCalc() {
    float pow = turnPID.calculate(turnPIDinput(chassis.pointangle()));
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (chassis.length(chassis.x, chassis.y, chassis.xtarg2, chassis.ytarg2) >
        turndisablelength) {
      leftpower = ypow + pow;
      rightpower = ypow - pow;

      first = true;

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.cartesianX();
      chassis.ytarg = chassis.y + chassis.cartesianY();
      leftpower = ypow, rightpower = ypow;

      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  void simplePPSBezierCalcBackward() {
    float pow = turnPID.calculate(turnPIDinput(chassis.pointangle() - 180));
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (chassis.length(chassis.x, chassis.y, chassis.xtarg2, chassis.ytarg2) >
        turndisablelength) {
      leftpower = ypow + pow;
      rightpower = ypow - pow;

      first = true;

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.bcartesianX();
      chassis.ytarg = chassis.y + chassis.bcartesianY();
      leftpower = ypow, rightpower = ypow;

      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  void PPSBezierCalc() {
    float pow = turnPID.calculate(turnPIDinput(chassis.pointangle()));
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (chassis.length(chassis.x, chassis.y, chassis.xtarg2, chassis.ytarg2) >
        turndisablelength) {

      leftpower = ypow + BLRSCurvature(ypow);
      rightpower = ypow - BLRSCurvature(ypow);

      // leftpower = ypow + pow;
      // rightpower = ypow - pow;

      first = true;

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.cartesianX();
      chassis.ytarg = chassis.y + chassis.cartesianY();
      leftpower = ypow, rightpower = ypow;

      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  void backwardsPPSBezierCalc() {
    float pow = turnPID.calculate(turnPIDinput(chassis.pointangle() - 180));
    float ypow = drivePID.calculate(ypower(chassis.ytarg, chassis.xtarg));
    if (chassis.length(chassis.x, chassis.y, chassis.xtarg2, chassis.ytarg2) >
        turndisablelength) {
      leftpower = ypow + BLRSCurvature(ypow);
      rightpower = ypow - BLRSCurvature(ypow);

      first = true;

    } else if (first == 1) {
      chassis.xtarg = chassis.x + chassis.bcartesianX();
      chassis.ytarg = chassis.y + chassis.bcartesianY();
      leftpower = ypow, rightpower = ypow;

      first = false;
    } else {
      leftpower = ypow, rightpower = ypow;
    }
  }

  void veldrive(float left, float right) {
    movementtype = 0;
    leftdrive.spin(fwd, left, pct);
    rightdrive.spin(fwd, right, pct);
  }

  void voldrive(float left, float right) {
    movementtype = 0;
    leftdrive.spin(fwd, left, volt);
    rightdrive.spin(fwd, right, volt);
  }

  void computeArc(float x1, float y1, float x2, float y2, float x3, float y3) {
    // circumcenter of 3 points
    float D = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    // collinear or nearly collinear — fall back to a huge radius straight line
    if (fabs(D) < 0.001) {
      arc_cx = (x1 + x3) / 2.0 + 10000.0;
      arc_cy = (y1 + y3) / 2.0 + 10000.0;
      arc_radius = 10000.0;
      arc_direction = 1;
      return;
    }

    float s1 = x1 * x1 + y1 * y1;
    float s2 = x2 * x2 + y2 * y2;
    float s3 = x3 * x3 + y3 * y3;
    arc_cx = (s1 * (y2 - y3) + s2 * (y3 - y1) + s3 * (y1 - y2)) / D;
    arc_cy = (s1 * (x3 - x2) + s2 * (x1 - x3) + s3 * (x2 - x1)) / D;
    arc_radius = sqrt(pow(arc_cx - x1, 2) + pow(arc_cy - y1, 2));

    // cross product determines winding direction
    float cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    arc_direction = (cross < 0) ? 1 : -1; // CW = +1, CCW = -1
  }

  void arcCalc() {
    // cross-track error: positive = outside arc, negative = inside
    float d = sqrt(pow(chassis.x - arc_cx, 2) + pow(chassis.y - arc_cy, 2));
    float crosstrackError = d - arc_radius;

    // heading error: compare actual heading to expected tangent at this position
    float theta = TO_DEG(atan2(chassis.x - arc_cx, chassis.y - arc_cy));
    float expectedHeading = chassis.wrapangle(theta + arc_direction * 90);
    if (arc_backward) expectedHeading = chassis.wrapangle(expectedHeading + 180);
    float headingError = chassis.wrapangle(expectedHeading - chassis.h);

    // arcPID.kp = cross-track P, arcPID.kd = heading P
    // When going backward, positive turnVel steers the opposite way,
    // so corrections must be negated to push the robot back onto the arc.
    float backwardMult = arc_backward ? -1.0f : 1.0f;
    // cross-track: arc_direction flips sign so "outside" always corrects inward
    float ctCorrection = backwardMult * arc_direction * arcPID.kp * crosstrackError;
    // heading: positive headingError = expected is CW from actual = need right turn = positive turnVel
    float hdgCorrection = backwardMult * arcPID.kd * headingError;

    float turnVel = arc_direction * arc_speed * trackwidth / (2.0 * arc_radius)
                   + ctCorrection + hdgCorrection;

    leftpower = arc_speed + turnVel;
    rightpower = arc_speed - turnVel;

    // saturation scaling — clamp to max motor voltage, not arc_speed
    float maxSide = fmax(fabs(leftpower), fabs(rightpower));
    if (maxSide > 12.7) {
      float scale = 12.7 / maxSide;
      leftpower *= scale;
      rightpower *= scale;
    }
  }

  bool movement_stop_hold = true;
  void movementloop() {
    if (autondriveenabled) {
      // printf("(%7.3f) %f", movementtype, (Brain.timer(msec) -
      // starttime)/1000);

      if (movementtype != lastmovementtype) {
        lastmovementtype = movementtype;
        turnPID.newPidCycle = true;
        swingPID.newPidCycle = true;

        if (movementtype == 0) {

        } else {
          //IDK WHY KOBE DEDICATED A CYCLE BETWEEN EACH MOVEMENT TO COASTING MOTORS...
          if (seamlessTransitions) {
            leftpower = 0, rightpower = 0;
            velocitydrive(leftpower, rightpower);
            drivestopping(0);
          }

        }
      } else {
        lastmovementtype = movementtype;
        // printf("(%7.3f) (%7.3f) %f\n", movementtype, lastmovementtype,
        // (Brain.timer(msec) - starttime)/1000);

        if (movementtype == 0) {
        } else if (movementtype < 4.0) {

          if (movementtype == 1) { // 1 = turn only
            spotTurnCalc();
          }
          if (movementtype == 1.1) { // 1 = turn only
            spotTurnCalctoPoint();
          }
          if (movementtype == 2) { // 2 = straight line PD
            swingOnLeftCalc();
          }
          if (movementtype == 2.1) { // 2 = straight line PD
            swingOnLefttoPointCalc();
          }
          if (movementtype == 3) { // 3 = classic odom drive forward
            swingOnRightCalc();
          }
          if (movementtype == 3.2) { // 3 = classic odom drive forward
            swingOnRighttoPointCalc();
          }
          if (movementtype == 3.1) { // 3 = classic odom drive forward
            straightheadingcalc();
          }
          if (movementtype == 3.9) { // 3 = classic odom drive forward
            PPSCalc();
          }
          float voltMult = (1);
          if (fabs(leftpower) < .25) {
            leftdrive.stop(hold);
          } else {
            leftdrive.spin(fwd, leftpower * voltMult * 1, volt);
          }
          if (fabs(rightpower) < .25) {
            rightdrive.stop(hold);
          } else {
            rightdrive.spin(fwd, rightpower * voltMult * 1, volt);
          }

          if (movement_stop_hold)
            drivestopping(2);
          else
            drivestopping(0);

        } else {
          if (movementtype == 4) { // 4 = classic odom drive backward
            simplePPSCalc();
          }
          if (movementtype == 5) { // 5 = drive to point without turn (last few
                                   // inches on og algorithm)
            simplePPSCalcBackward();
          }
          if (movementtype == 6) { // 6 = pure pursuit drive to point forward
            PPSCalc();
          }
          if (movementtype == 7) { // 7 = pure pursuit drive to point backward
            PPSCalcBackward();
          }
          if (movementtype == 8) { // 8 = default bezier forward
            PPSBezierCalc();
          }
          if (movementtype == 8.1) { // 8 = default bezier forward
            // PPSBezierCalc();
            simplePPSBezierCalc();
          }
          if (movementtype == 8.2) { // 8 = default bezier forward
            backwardsPPSBezierCalc();
          }
          if (movementtype == 9) { // 8 = default bezier backward
            simplePPSBezierCalcBackward();
          }
          if (movementtype == 10) { // 10
            straightlinecalc();
          }
          if (movementtype == 11) { // 11
            straightheadingcalc();
          }
          if (movementtype == 11.1) { // 11
            straightheadingcalcToPoint();
          }
          if (movementtype == 12) { // 12 = boomerang drive to pose
            boomerangCalc();
          }
          if (movementtype == 14) { // 14 = inception boomerang
            inceptionBoomerangCalc();
          }
          if (movementtype == 13) { // 13 = circular arc
            arcCalc();
          }
          voltagedrive(leftpower, rightpower * 1);
          drivestopping(0);
        }
      }
    }
  }

  void lWait(float input) { // blocks until the robot is within the a certain
                            // distance of the targets
    while (chassis.pointdist() > input) {
      delay(10);
    }
  }

  void lWaitPerp(float input, bool backward = false) {
    // capture start position to compute travel direction
    float startX = chassis.x;
    float startY = chassis.y;
    float totalDist = chassis.pointdist();

    while (chassis.pointdist() > input) {
      // only check perp cross after robot has covered half the distance
      float traveled = chassis.length(chassis.x, chassis.y, startX, startY);
      if (traveled > totalDist * 0.5) {
        // perpendicular line through target, normal to robot's current heading
        float headingRad = TO_RAD(chassis.h);
        if (backward) headingRad += M_PI;
        float tx = sin(headingRad);
        float ty = cos(headingRad);
        float dx = chassis.x - chassis.xtarg;
        float dy = chassis.y - chassis.ytarg;
        float dot = dx * tx + dy * ty;
        if (dot > 0) break;
      }

      delay(10);
    }
  }

  void turnWaitConditions(float bigError, float smallError, float bigErrorTime,
                          float smallErrorTime, PID &pid) {
    float smallStartTime = 0;
    float bigStartTime = 0;
    float totalSmallTime = 0;
    float totalBigTime = 0;
    while (true) {
      // printf("kp %3.3f ki %3.3f kd %3.3f \n", pid.kp, pid.ki,
      // pid.kd), printf("e %3.3f se %3.3f ki %3.3f \n \n", pid.error,
      // pid.sumerror, pid.sumerror * pid.ki);
      if (fabs(pid.error) < smallError &&
          totalSmallTime == 0) { // sum smallError total time
        smallStartTime = Brain.timer(msec);
        totalSmallTime = 10;
      } else if (fabs(pid.error) < smallError) {
        totalSmallTime = Brain.timer(msec) - smallStartTime;
      }
      if (fabs(pid.error) < bigError &&
          totalBigTime == 0) { // sum bigError total time
        bigStartTime = Brain.timer(msec);
        totalBigTime = 10;
      } else if (fabs(pid.error) < bigError) {
        totalBigTime = Brain.timer(msec) - bigStartTime;
      }
      if (totalSmallTime > smallErrorTime) { // if its within the small margin
                                             // for enough time break
        break;
      }
      if (totalBigTime > bigErrorTime) { // if its with the big margin for enough time break

        break;
      }




      delay(20);
    }
  }

  void turnWaitConditions(float bigError, float smallError, float bigErrorTime,
                          float smallErrorTime) {
    turnWaitConditions(bigError, smallError, bigErrorTime, smallErrorTime, turnPID);
  }

  float bigE = 1;
  float smallE = .5;
  float bigTime = 400;
  float smallTime = 150;
  void face(float ang, float maxSpeed,
            float breakang, int dir = 0) {
    // full 360: chain two 180° turns
    if (ang == 360 || ang == -360) {
      int spinDir = (ang == 360) ? 1 : 2;  // 360 = CW, -360 = CCW
      if (dir != 0) spinDir = dir;          // dir overrides if set
      float mid = chassis.wrapangle(chassis.h + 180);
      face(mid, maxSpeed, breakang, spinDir);
      face(chassis.wrapangle(mid + 180), maxSpeed, breakang, spinDir);
      return;
    }
    chassis.htarg = ang;
    turnDir = dir;
    turnPID.scalePID = true;
    turnPID.headingMode = false;
    // turnPID.setIntegral(400, 2.5);
    turnPID.maxOutput = maxSpeed;
    turnPID.reset();
    leftpower = 0, rightpower = 0;
    movementtype = 1;
    stopdrive(0);
    if (breakang > 0) {
      delay(10);
      turnWaitConditions(breakang, smallE, bigTime, smallTime);
      turnDir = 0;
    }
  }

  float left_total_stopped_time = 0;
  float right_total_stopped_time = 0;
  float left_stopped_start_time = 0;
  float right_stopped_start_time = 0;

  void wait_for_vel(float left, float left_time) {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0, 0, 480, 240);
    delay(100);
    // float drive_vel = fabs(TY.velocity(rpm)); //USING VERTICAL ODOM
    float drive_vel = fabs(TY.velocity(rpm)); // USING MOTOR ENCODER
    while (1) {
      // drive_vel = fabs(TY.velocity(rpm)); //USING VERTICAL ODOM
      drive_vel = fabs(TY.velocity(rpm)); // USING MOTOR ENCODER
      if (drive_vel < left) {
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawRectangle(0, 0, 480, 240);
        if (drive_vel < left &&
            left_total_stopped_time == 0) { // sum smallError total time
          left_stopped_start_time = Brain.timer(msec);
          left_total_stopped_time = 10;
        } else if (drive_vel < left) {
          left_total_stopped_time = Brain.timer(msec) - left_stopped_start_time;
        }
        if (left_total_stopped_time > left_time) {
          break;
        }
      } else {
        left_total_stopped_time = 0;
        Brain.Screen.setFillColor(green);
        Brain.Screen.drawRectangle(0, 0, 480, 240);
      }
      /*
      if (fabs(rightdrive.velocity(pct)) < right) {
        if (fabs(rightdrive.velocity(pct)) < right && right_total_stopped_time
      == 0) { // sum smallError total time right_stopped_start_time =
      Brain.timer(msec); right_total_stopped_time = 10; } else if
      (fabs(rightdrive.velocity(pct)) < right) { right_total_stopped_time =
      Brain.timer(msec) - right_stopped_start_time;
        }
          if (right_total_stopped_time > right_time) {
        break;
          }
      } else {
        right_total_stopped_time = 0;
      }
  */
      delay(10);
    }
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(0, 0, 480, 240);
  }

  void facePoint(float x, float y, float maxSpeed,
                 float breakang, bool backward, int dir = 0) {
    turnPID.reset();
    turnPID.sumerror = 0;
    turnDir = dir;
    if (backward) {
      backwards = true;
      chassis.htarg = chassis.pointangle(x, y) - 180;
    } else {
      backwards = false;
      chassis.htarg = chassis.pointangle(x, y);
    }
    x_point = x;
    y_point = y;
    turnPID.reset();
    turnPID.scalePID = true;
    turnPID.headingMode = false;
    turnPID.maxOutput = maxSpeed;
    leftpower = 0, rightpower = 0;
    movementtype = 1.1;
    stopdrive(0);

    if (breakang > 0) {
      delay(10);
      turnWaitConditions(breakang, smallE, bigTime, smallTime);
      turnDir = 0;
    }
  }

  void swingOnRight(float ang, float maxSpeed,
                    float breakang, int dir = 0) {
    chassis.htarg = ang;
    turnDir = dir;
    swingPID.scalePID = true;
    swingPID.headingMode = false;
    swingPID.setIntegral(400, 2.5);
    swingPID.maxOutput = maxSpeed;
    swingPID.reset();
    leftpower = 0, rightpower = 0;
    movementtype = 2;
    stopdrive(0);
    if (breakang > 0) {
      delay(10);
      turnWaitConditions(breakang, smallE, bigTime, smallTime, swingPID);
      turnDir = 0;
    }
  }

  void swingOnRighttoPoint(float x, float y,
                           float maxSpeed, float breakang, bool backward, int dir = 0) {
    backwards = backward;
    turnDir = dir;

    if (backward) {
      chassis.htarg = chassis.pointangle(x, y) + 180;
    } else {
      chassis.htarg = chassis.pointangle(x, y);
    }
    x_point = x;
    y_point = y;
    if (backward)
      backwards = true;
    swingPID.scalePID = true;
    swingPID.headingMode = false;
    swingPID.maxOutput = maxSpeed;
    swingPID.reset();
    leftpower = 0, rightpower = 0;
    movementtype = 3.2;
    stopdrive(0);
    if (breakang > 0) {
      delay(10);
      turnWaitConditions(breakang, smallE, bigTime, smallTime, swingPID);
      turnDir = 0;
    }
  }

  void swingOnLefttoPoint(float x, float y,
                          float maxSpeed, float breakang, bool backward, int dir = 0) {
    backwards = backward;
    turnDir = dir;

    if (backward) {
      chassis.htarg = chassis.pointangle(x, y) + 180;
    } else {
      chassis.htarg = chassis.pointangle(x, y);
    }
    x_point = x;
    y_point = y;
    if (backward)
      backwards = true;
    swingPID.scalePID = true;
    swingPID.headingMode = false;
    swingPID.maxOutput = maxSpeed;
    swingPID.reset();
    leftpower = 0, rightpower = 0;
    movementtype = 2.1;
    stopdrive(0);
    if (breakang > 0) {
      delay(10);
      turnWaitConditions(breakang, smallE, bigTime, smallTime, swingPID);
      turnDir = 0;
    }
  }

  void swingOnLeft(float ang, float maxSpeed,
                   float breakang, int dir = 0) {
    chassis.htarg = ang;
    turnDir = dir;
    swingPID.scalePID = true;
    swingPID.headingMode = false;
    swingPID.setIntegral(400, 2.5);
    swingPID.maxOutput = maxSpeed;
    swingPID.reset();
    leftpower = 0, rightpower = 0;
    movementtype = 3;
    stopdrive(0);
    if (breakang > 0) {
      delay(10);
      turnWaitConditions(breakang, smallE, bigTime, smallTime, swingPID);
      turnDir = 0;
    }
  }

  float QuadBezier(float p0, float p1, float p2,
                   float t) { // interpolates between the 3 given points
    float pFinal = 0, cp = 0;
    // Quadratic Bezier Formula
    cp = p1 * 2 - (p0 + p2) / 2;
    pFinal = pow((1 - t), 2) * p0 + (1 - t) * 2 * t * cp + t * t * p2;
    return pFinal;
  }

  // calculate the curve points and save them as an array
  float XCurve[70];
  void getXCurve(float p0, float p1, float p2, int fidelity) {
    float t = 0;
    for (int i = 0; i <= fidelity; i++) {
      t = (float)i / fidelity;
      if (t < 1) {
        XCurve[i] = QuadBezier(p0, p1, p2, t);
      }
    }
  }

  // calculate the curve points and save them as an array
  float YCurve[70];
  void getYCurve(float p0, float p1, float p2, int fidelity) {
    float t = 0;
    for (int i = 0; i <= fidelity; i++) {
      t = (float)i / fidelity;
      if (t < 1) {
        YCurve[i] = QuadBezier(p0, p1, p2, t);
      }
    }
  }

  float startingtrackervalue = 0;
  float straightlinegoal = 0;
  float straighterror = 0;
  void straight(float dist, float kp, float kd, float maxSpeed, float slew,
                float breakdist) {
    drivePID.reset();
    turnPID.reset();
    chassis.xtarg = chassis.x;
    chassis.ytarg = chassis.y;
    chassis.htarg = chassis.h;
    startingtrackervalue = (chassis.paralleltracker * tyD * M_PI / 360);
    // changed for 2 inch tracking wheel
    // TRACKER WHEEL CONFIGURATION HERE
    chassis.ytarg2 = 0;
    drivePID.slewAmount = slew;
    straightlinegoal = dist;
    drivePID.setConstants(kp, drivePID.ki, drivePID.kd);
    movementtype = 10;

    if (breakdist > 0) {
      delay(30);
      while (true) {

        if (fabs(straighterror) < breakdist) {
          break;
        }
        delay(10);
      }
    }
    chassis.xtarg = chassis.x;
    chassis.ytarg = chassis.y;
    chassis.htarg = chassis.h;
  }

  void straighth(float dist, float htarget, float kp, float kd, float hkp,
                 float hkd, float maxSpeed, float hmaxspeed, float slew,
                 float breakdist) {
    drivePID.reset();
    turnPID.reset();
    chassis.xtarg = chassis.x;
    chassis.ytarg = chassis.y;
    chassis.htarg = chassis.h;
    startingtrackervalue =
        (chassis.paralleltracker * tyD * M_PI / 360); // Using motor encoder
    // changed for 2 inch tracking wheel
    // TRACKER WHEEL CONFIGURATION HERE
    chassis.ytarg2 = 0;
    drivePID.slewAmount = slew;
    straightlinegoal = dist;
    drivePID.setConstants(kp, drivePID.ki, kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    drivePID.maxOutput = maxSpeed;
    chassis.htarg = htarget;
    turnPID.maxOutput = hmaxspeed;

    movementtype = 11;

    if (breakdist > 0) {
      delay(30);
      while (true) {

        if (fabs(straighterror) < breakdist) {
          break;
        }
        delay(10);
      }
    }
    chassis.xtarg = chassis.x;
    chassis.ytarg = chassis.y;
    chassis.htarg = chassis.h;
  }

  void straighthToPoint(float dist, float x, float y, float kp, float kd,
                        float hkp, float hkd, float maxSpeed, float hmaxspeed,
                        float slew, float breakdist, bool backward) {
    backwards = backward;

    drivePID.reset();
    turnPID.reset();
    if (backward) {
      chassis.htarg = chassis.pointangle(x, y) + 180;
    } else {
      chassis.htarg = chassis.pointangle(x, y);
    }
    x_point = x;
    y_point = y;
    if (backward)
      backwards = true;
    startingtrackervalue = (chassis.paralleltracker * tyD * M_PI / 360);
    // changed for 2 inch tracking wheel
    // TRACKER WHEEL CONFIGURATION HERE
    chassis.ytarg2 = 0;
    drivePID.slewAmount = slew;
    straightlinegoal = dist;
    drivePID.setConstants(kp, drivePID.ki, kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    drivePID.maxOutput = maxSpeed;
    turnPID.maxOutput = maxSpeed;
    if (backward)
      backwards = true;

    movementtype = 11.1;

    if (breakdist > 0) {
      delay(30);
      while (true) {

        if (fabs(straighterror) < breakdist) {
          break;
        }
        delay(10);
      }
    }
    chassis.xtarg = chassis.x;
    chassis.ytarg = chassis.y;
    chassis.htarg = chassis.h;
  }

  // Boomerang: JAR-Template style drive-to-pose controller
  void boomerang(float x, float y, float target_heading, bool backward,
                 float drive_kp, float drive_kd, float heading_kp,
                 float heading_kd, float drive_max_voltage,
                 float heading_max_voltage, float lead, float min_voltage,
                 float settle_error, float setback, float slew_rate,
                 float timeout, float exit_dist) {

    drivePID.reset();
    turnPID.reset();

    drivePID.setConstants(drive_kp, drivePID.ki, drive_kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    drivePID.maxOutput = drive_max_voltage;
    turnPID.maxOutput = heading_max_voltage;
    drivePID.slewAmount = slew_rate;

    boomerang_target_x = x;
    boomerang_target_y = y;
    boomerang_target_heading = target_heading;
    boomerang_backwards = backward;
    boomerang_lead = lead;
    boomerang_setback = setback;
    boomerang_settle_error = settle_error;
    boomerang_drive_max_voltage = drive_max_voltage;
    boomerang_heading_max_voltage = heading_max_voltage;
    boomerang_min_voltage = min_voltage;
    boomerang_timeout = timeout;
    boomerang_start_time = Brain.timer(msec);

    boomerang_crossed_end_line = false;
    boomerang_prev_end_line_side = false;
    boomerang_end_line_initialized = false;
    boomerang_crossed_center_line = false;
    boomerang_prev_center_side = false;
    boomerang_center_initialized = false;
    boomerang_settled = false;
    boomerang_timed_out = false;
    boomerang_in_final_phase = false;
    boomerang_settle_count = 0;

    leftpower = 0;
    rightpower = 0;
    movementtype = 12;
    stopdrive(0);

    if (exit_dist > 0) {
      delay(10);
      while (true) {
        if (boomerang_settled)
          break;
        if (boomerang_timed_out)
          break;
        if (boomerang_crossed_end_line)
          break;
        float dx = boomerang_target_x - chassis.x;
        float dy = boomerang_target_y - chassis.y;
        if (sqrt(dx * dx + dy * dy) < exit_dist)
          break;
        delay(10);
      }
    }
  }

  void inceptionBoomerang(float x, float y, float target_heading, bool backward,
                          float drive_kp, float drive_kd,
                          float heading_kp, float heading_kd,
                          float drive_max_voltage, float heading_max_voltage,
                          float dLead, float gLead,
                          float min_voltage, float settle_error,
                          float slew_rate, float timeout,
                          float close_end_dist, float close_ghost_dist,
                          float parallel_dist, float exit_dist) {

    // Reset PIDs
    drivePID.reset();
    turnPID.reset();
    drivePID.setConstants(drive_kp, drivePID.ki, drive_kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    drivePID.maxOutput = drive_max_voltage;
    turnPID.maxOutput = heading_max_voltage;
    drivePID.slewAmount = slew_rate;

    // Set target pose
    ib_target_x = x;
    ib_target_y = y;
    ib_target_heading = target_heading;
    ib_backwards = backward;

    // Carrot geometry
    ib_dLead = dLead;
    ib_gLead = gLead;

    // Compute initial carrot position (behind endpoint along target heading)
    float target_angle_rad = TO_RAD(target_heading);
    ib_initial_carrot_x = x - sin(target_angle_rad) * dLead;
    ib_initial_carrot_y = y - cos(target_angle_rad) * dLead;
    ib_carrot_x = ib_initial_carrot_x;
    ib_carrot_y = ib_initial_carrot_y;

    // Initialize carrot retraction distances
    float dx = ib_carrot_x - chassis.x;
    float dy = ib_carrot_y - chassis.y;
    ib_init_carrot_dist = sqrt(dx * dx + dy * dy);
    ib_min_carrot_dist = ib_init_carrot_dist;

    // Phase thresholds
    ib_close_end_dist = close_end_dist;
    ib_close_ghost_dist = close_ghost_dist;
    ib_parallel_dist = parallel_dist;

    // Voltage limits
    ib_drive_max_voltage = drive_max_voltage;
    ib_heading_max_voltage = heading_max_voltage;
    ib_min_voltage = min_voltage;

    // Reset phase latches
    ib_close_end = false;
    ib_close_ghost = false;

    // Reset settlement
    ib_settle_error = settle_error;
    ib_settle_count = 0;
    ib_settled = false;

    // Reset perp line cross
    ib_crossed_end_line = false;
    ib_prev_end_line_side = false;
    ib_end_line_initialized = false;

    // Timeout
    ib_timeout = timeout;
    ib_start_time = Brain.timer(msec);
    ib_timed_out = false;

    // Activate
    leftpower = 0;
    rightpower = 0;
    movementtype = 14;
    stopdrive(0);

    // Blocking wait
    if (exit_dist > 0) {
      delay(10);
      while (true) {
        if (ib_settled) break;
        if (ib_timed_out) break;
        if (ib_crossed_end_line) break;

        float ddx = ib_target_x - chassis.x;
        float ddy = ib_target_y - chassis.y;
        if (sqrt(ddx * ddx + ddy * ddy) < exit_dist) break;

        delay(10);
      }
    }
  }

  // move.to() IS HERE (tuffest function oat)
  void to(float x, float y, float yspeed, float hspeed, float ykp, float hkp,
          float slew, float breakLength, int dir = 0) {
    chassis.xtarg = x;
    chassis.ytarg = y;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    turnDir = dir;
    leftpower = 0, rightpower = 0;
    hasExitedTurnOnly = false;
    movementtype = 4;
    stopdrive(0);
    if (breakLength > 0) {
      lWaitPerp(breakLength, false);
    }
  }

  void to(float x, float y, float yspeed, float hspeed, float ykp, float hkp,
          float slew, float breakLength, float turnmargin, float turntype, int dir = 0) {
    turnType = turntype;
    turnMargin = turnmargin;
    chassis.xtarg = x;
    chassis.ytarg = y;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    turnDir = dir;
    leftpower = 0, rightpower = 0;
    hasExitedTurnOnly = false;
    movementtype = 4;
    stopdrive(0);
    if (breakLength > 0) {
      lWaitPerp(breakLength, false);
    }
  }

  void bto(float x, float y, float yspeed, float hspeed, float ykp, float hkp,
           float slew, float breakLength, int dir = 0) {
    chassis.xtarg = x;
    chassis.ytarg = y;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    turnDir = dir;
    leftpower = 0, rightpower = 0;
    movementtype = 5;
    stopdrive(0);
    if (breakLength > 0) {
      lWaitPerp(breakLength, true);
    }
  }

  void bto(float x, float y, float yspeed, float hspeed, float ykp, float hkp,
           float slew, float breakLength, float turnmargin, float turntype, int dir = 0) {
    turnType = turntype;
    turnMargin = turnmargin;
    chassis.xtarg = x;
    chassis.ytarg = y;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    turnDir = dir;
    leftpower = 0, rightpower = 0;
    movementtype = 5;
    stopdrive(0);
    if (breakLength > 0) {
      lWaitPerp(breakLength, true);
    }
  }

  void PPSto(float x, float y, float yspeed, float hspeed, float ykp, float hkp,
             float slew, float breakLength, float relativeAng, float turnEnd,
             float T) {
    chassis.xtarg = x;
    chassis.ytarg = y;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    trackwidth = T;
    turndisablelength = turnEnd;
    turnMargin = relativeAng;
    leftpower = 0, rightpower = 0;
    movementtype = 6;
    stopdrive(0);
    if (breakLength > 0) {
      lWait(breakLength);
    }
  }

  void PPSbto(float x, float y, float yspeed, float hspeed, float ykp,
              float hkp, float slew, float breakLength, float relativeAng,
              float turnEnd, float T) {
    chassis.xtarg = x;
    chassis.ytarg = y;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    trackwidth = T;
    turndisablelength = turnEnd;
    turnMargin = relativeAng;
    leftpower = 0, rightpower = 0;
    movementtype = 7;
    stopdrive(0);
    if (breakLength > 0) {
      lWait(breakLength);
    }
  }

  void CurveTo(float x1, float y1, float x2, float y2, float yspeed,
               float hspeed, float ykp, float hkp, float slew, float lookDist,
               float breakLength) {
    int resolution = 50;
    getXCurve(chassis.x, x1, x2, resolution);
    getYCurve(chassis.y, y1, y2, resolution);
    int coordSet = 0;
    chassis.xtarg2 = x2;
    chassis.ytarg2 = y2;
    chassis.xtarg = x1;
    chassis.ytarg = y1;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    leftpower = 0, rightpower = 0;
    movementtype = 8;
    stopdrive(0);

    while (chassis.length(chassis.x, chassis.y, x2, y2) > breakLength) {
      chassis.xtarg =
          XCurve[coordSet]; // sets the current x target to the nearest
                            // coordinate greater than breaklength away
      chassis.ytarg =
          YCurve[coordSet]; // sets the current y target to the nearest
                            // coordinate greater than breaklength away

      if ((chassis.length(chassis.x, chassis.y, chassis.xtarg, chassis.ytarg)) <
          lookDist) { // if the point is less than the look ahead distance away
                      // go to next point

        if (coordSet <
            resolution - 2) { // dont go past the resolution or stuff breaks
          coordSet++;
        } else {
          chassis.xtarg = chassis.xtarg2;
          chassis.ytarg = chassis.ytarg2;
        }
      }
      delay(10);
    }
    chassis.xtarg = x2;
    chassis.ytarg = y2;
  }

  void arc(float x1, float y1, float x2, float y2, float speed,
           float breakLength, float arcCtKP, float arcHdgKP,
           float alignMargin, bool backward = false,
           float turnMaxSpeed = 12) {
    // compute arc through current position, midpoint (x1,y1), endpoint (x2,y2)
    computeArc(chassis.x, chassis.y, x1, y1, x2, y2);
    arc_backward = backward;

    // tangent heading at the start of the arc
    float theta = TO_DEG(atan2(chassis.x - arc_cx, chassis.y - arc_cy));
    float tangentHeading = chassis.wrapangle(theta + arc_direction * 90);
    if (backward) tangentHeading = chassis.wrapangle(tangentHeading + 180);

    // if heading is too far off the tangent, spot turn to align first
    if (alignMargin > 0 && fabs(chassis.wrapangle(tangentHeading - chassis.h)) > alignMargin) {
      chassis.htarg = tangentHeading;
      turnDir = 0; // clear stale turn direction from previous movements
      turnPID.reset();
      turnPID.scalePID = true;
      turnPID.headingMode = false;
      turnPID.maxOutput = turnMaxSpeed;
      leftpower = 0, rightpower = 0;
      movementtype = 1; //SPOT TURN
      stopdrive(0);
      delay(10);
      turnWaitConditions(alignMargin, smallE, bigTime, smallTime);
      turnDir = 0;
    }

    arc_speed = backward ? -speed : speed;
    arcPID.reset();
    arcPID.setConstants(arcCtKP, 0, arcHdgKP);
    arcPID.maxOutput = 12.7;
    arcPID.minOutput = 0;
    leftpower = 0, rightpower = 0;
    movementtype = 13;
    stopdrive(0);

    // perpendicular line at endpoint for fallback exit:
    // tangent at endpoint is perpendicular to radius (center -> endpoint)
    float endTheta = TO_DEG(atan2(x2 - arc_cx, y2 - arc_cy));
    float endTangentDeg = chassis.wrapangle(endTheta + arc_direction * 90);
    if (backward) endTangentDeg = chassis.wrapangle(endTangentDeg + 180);

    // unit vector along tangent at endpoint (VEX heading: 0°=+Y, CW positive)
    float tangentRad = endTangentDeg * M_PI / 180.0;
    float tx = sin(tangentRad);
    float ty = cos(tangentRad);

    while (chassis.length(chassis.x, chassis.y, x2, y2) > breakLength) {
      // perpendicular line cross: if robot passed the endpoint along the tangent, exit
      float dx = chassis.x - x2;
      float dy = chassis.y - y2;
      float dot = dx * tx + dy * ty;
      if (dot > 0) break;

      delay(10);
    }

    movementtype = 0;
    leftpower = 0;
    rightpower = 0;
    chassis.xtarg = x2;
    chassis.ytarg = y2;

  }

  void PPSCurveTo(float x1, float y1, float x2, float y2, float yspeed,
                  float hspeed, float ykp, float hkp, float slew,
                  float lookDist, float breakLength, float T) {
    movementtype = 8;
    trackwidth = T;
    int resolution = 50;
    getXCurve(chassis.x, x1, x2, resolution);
    getYCurve(chassis.y, y1, y2, resolution);
    int coordSet = 0;
    chassis.xtarg2 = x2;
    chassis.ytarg2 = y2;
    chassis.xtarg = x1;
    chassis.ytarg = y1;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    leftpower = 0, rightpower = 0;
    movementtype = 8;
    stopdrive(0);

    while (chassis.length(chassis.x, chassis.y, x2, y2) > breakLength) {
      chassis.xtarg =
          XCurve[coordSet]; // sets the current x target to the nearest
                            // coordinate greater than breaklength away
      chassis.ytarg =
          YCurve[coordSet]; // sets the current y target to the nearest
                            // coordinate greater than breaklength away

      if ((chassis.length(chassis.x, chassis.y, chassis.xtarg, chassis.ytarg)) <
          lookDist) { // if the point is less than the look ahead distance away
                      // go to next point

        if (coordSet <
            resolution - 2) { // dont go past the resolution or stuff breaks
          coordSet++;
        } else {
          chassis.xtarg = chassis.xtarg2;
          chassis.ytarg = chassis.ytarg2;
        }
      }
      delay(10);
    }
    chassis.xtarg = x2;
    chassis.ytarg = y2;
  }

  void PPbSCurveTo(float x1, float y1, float x2, float y2, float yspeed,
                   float hspeed, float ykp, float hkp, float slew,
                   float lookDist, float breakLength, float T) {
    trackwidth = T;
    int resolution = 50;
    getXCurve(chassis.x, x1, x2, resolution);
    getYCurve(chassis.y, y1, y2, resolution);
    int coordSet = 0;
    chassis.xtarg2 = x2;
    chassis.ytarg2 = y2;
    chassis.xtarg = x1;
    chassis.ytarg = y1;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    leftpower = 0, rightpower = 0;
    movementtype = 8.2;
    stopdrive(0);

    while (chassis.length(chassis.x, chassis.y, x2, y2) > breakLength) {
      chassis.xtarg =
          XCurve[coordSet]; // sets the current x target to the nearest
                            // coordinate greater than breaklength away
      chassis.ytarg =
          YCurve[coordSet]; // sets the current y target to the nearest
                            // coordinate greater than breaklength away

      if ((chassis.length(chassis.x, chassis.y, chassis.xtarg, chassis.ytarg)) <
          lookDist) { // if the point is less than the look ahead distance away
                      // go to next point

        if (coordSet <
            resolution - 2) { // dont go past the resolution or stuff breaks
          coordSet++;
        } else {
          chassis.xtarg = chassis.xtarg2;
          chassis.ytarg = chassis.ytarg2;
        }
      }
      delay(10);
    }
    chassis.xtarg = x2;
    chassis.ytarg = y2;
  }

  void BCurveTo(float x1, float y1, float x2, float y2, float yspeed,
                float hspeed, float ykp, float hkp, float slew, float lookDist,
                float breakLength) {
    int resolution = 50;
    getXCurve(chassis.x, x1, x2, resolution);
    getYCurve(chassis.y, y1, y2, resolution);
    int coordSet = 0;
    chassis.xtarg2 = x2;
    chassis.ytarg2 = y2;
    chassis.xtarg = x1;
    chassis.ytarg = y1;
    drivePID.reset();
    turnPID.reset();
    drivePID.maxOutput = yspeed;
    turnPID.maxOutput = hspeed;
    drivePID.slewAmount = slew;
    drivePID.setConstants(ykp, drivePID.ki, drivePID.kd);
    turnPID.scalePID = false;
    turnPID.headingMode = true;
    leftpower = 0, rightpower = 0;
    movementtype = 9;
    stopdrive(0);
    while (chassis.length(chassis.x, chassis.y, x2, y2) > breakLength) {
      chassis.xtarg =
          XCurve[coordSet]; // sets the current x target to the nearest
                            // coordinate greater than breaklength away
      chassis.ytarg =
          YCurve[coordSet]; // sets the current y target to the nearest
                            // coordinate greater than breaklength away

      if ((chassis.length(chassis.x, chassis.y, chassis.xtarg, chassis.ytarg)) <
          lookDist) { // if the point is less than the look ahead distance away
                      // go to next point

        if (coordSet <
            resolution - 2) { // dont go past the resolution or stuff breaks
          coordSet++;
        } else {
          chassis.xtarg = chassis.xtarg2;
          chassis.ytarg = chassis.ytarg2;
        }
      }
      delay(10);
    }
    chassis.xtarg = x2;
    chassis.ytarg = y2;
  }
};

extern movement move;