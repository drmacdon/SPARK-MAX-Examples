/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class Robot extends TimedRobot {
  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public int smartCurrentLimit = 20;
  public double positionConversionFactor;
  public double velocityConversionFactor;

  public boolean enableForwardLimit = true;
  public boolean enableBackwardLimit = true;
  public float forwardLimit = 0.1f;
  public float backwardLimit = 0.0f;

  @Override
  public void robotInit() {
    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    m_motor.setSmartCurrentLimit(smartCurrentLimit);
    positionConversionFactor = m_encoder.getPositionConversionFactor();
    velocityConversionFactor = m_encoder.getVelocityConversionFactor();

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    SmartDashboard.putNumber("Motor Smart Current Limit", smartCurrentLimit);
    SmartDashboard.putNumber("Position Conversion Factor", positionConversionFactor);
    SmartDashboard.putNumber("Velocity Conversion Factor", velocityConversionFactor);

    SmartDashboard.putBoolean("Enable Forward Limit", enableForwardLimit);
    SmartDashboard.putBoolean("Enable Backward Limit", enableBackwardLimit);
    SmartDashboard.putNumber("Forward Limit", forwardLimit);
    SmartDashboard.putNumber("Backward Limit", backwardLimit);

    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enableForwardLimit);
    m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enableBackwardLimit);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardLimit);
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, backwardLimit);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    SmartDashboard.putNumber("CAN ID", deviceID);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);

    SmartDashboard.putBoolean("RUN", false);
  }

  @Override
  public void teleopPeriodic() {
    boolean run = SmartDashboard.getBoolean("RUN", false);
    if (!run) {
      m_motor.stopMotor();
    } else {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      double maxV = SmartDashboard.getNumber("Max Velocity", 0);
      double minV = SmartDashboard.getNumber("Min Velocity", 0);
      double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
      double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

      int currentLimit = (int) SmartDashboard.getNumber("Motor Smart Current Limit", 20);
      double posConvFactor = SmartDashboard.getNumber("Position Conversion Factor", 1);
      double velConvFactor = SmartDashboard.getNumber("Velocity Conversion Factor", 1);
      boolean dashEnableForward = SmartDashboard.getBoolean("Enable Forward Limit", true);
      boolean dashEnableBackward = SmartDashboard.getBoolean("Enable Backward Limit", true);
      float dashForwardLimit = (float) SmartDashboard.getNumber("Forward Limit", 0.1f);
      float dashBackwardLimit = (float) SmartDashboard.getNumber("Backward Limit", 0.0f);

      if (currentLimit != smartCurrentLimit) {
        m_motor.setSmartCurrentLimit(currentLimit);
        smartCurrentLimit = currentLimit;
      }

      if (posConvFactor != positionConversionFactor) {
        m_encoder.setPositionConversionFactor(posConvFactor);
        positionConversionFactor = posConvFactor;
      }

      if (velConvFactor != velocityConversionFactor) {
        m_encoder.setVelocityConversionFactor(velConvFactor);
        velocityConversionFactor = velConvFactor;
      }

      if (dashEnableForward != enableForwardLimit) {
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, dashEnableForward);
        enableForwardLimit = dashEnableForward;
      }

      if (dashEnableBackward != enableBackwardLimit) {
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, dashEnableBackward);
        enableBackwardLimit = dashEnableBackward;
      }

      if (dashForwardLimit != forwardLimit) {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, dashForwardLimit);
        forwardLimit = dashForwardLimit;
      }

      if (dashBackwardLimit != backwardLimit) {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, dashBackwardLimit);
        backwardLimit = dashBackwardLimit;
      }

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if ((p != kP)) {
        m_pidController.setP(p);
        kP = p;
      }
      if ((i != kI)) {
        m_pidController.setI(i);
        kI = i;
      }
      if ((d != kD)) {
        m_pidController.setD(d);
        kD = d;
      }
      if ((iz != kIz)) {
        m_pidController.setIZone(iz);
        kIz = iz;
      }
      if ((ff != kFF)) {
        m_pidController.setFF(ff);
        kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        m_pidController.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;
      }
      if ((maxV != maxVel)) {
        m_pidController.setSmartMotionMaxVelocity(maxV, 0);
        maxVel = maxV;
      }
      if ((minV != minVel)) {
        m_pidController.setSmartMotionMinOutputVelocity(minV, 0);
        minVel = minV;
      }
      if ((maxA != maxAcc)) {
        m_pidController.setSmartMotionMaxAccel(maxA, 0);
        maxAcc = maxA;
      }
      if ((allE != allowedErr)) {
        m_pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
        allowedErr = allE;
      }

      double setPoint, processVariable;
      boolean mode = SmartDashboard.getBoolean("Mode", false);
      if (mode) {
        setPoint = SmartDashboard.getNumber("Set Velocity", 0);
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        processVariable = m_encoder.getVelocity();
      } else {
        setPoint = SmartDashboard.getNumber("Set Position", 0);
        /**
         * As with other PID modes, Smart Motion is set by calling the
         * setReference method on an existing pid object and setting
         * the control type to kSmartMotion
         */
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = m_encoder.getPosition();
      }

      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("Process Variable", processVariable);
      SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
    }
  }
}
