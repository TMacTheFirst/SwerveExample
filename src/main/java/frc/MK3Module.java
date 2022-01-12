// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.*;

/** Add your docs here. */
public class MK3Module {
    // JUNK FROM THE EXAMPLE
    private static final double kModuleMaxAngularVelocity = Math.PI; // 1/2 rotation per second
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  
    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    // ACTUAL HARDWARE ON MODULE
    WPI_TalonFX steeringMotor;
    WPI_TalonFX driveMotor;
    CANCoder encoder;

    public MK3Module(int steeringMotorId, int driveMotorId, int sensorId, double offset) {
        steeringMotor = new WPI_TalonFX(steeringMotorId);
        driveMotor = new WPI_TalonFX(driveMotorId);
        encoder = new CANCoder(sensorId);
        encoder.configMagnetOffset(offset);

        // Might need to do encoder setup here
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(steeringMotor.getSelectedSensorPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(steeringMotor.getSelectedSensorPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            m_drivePIDController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(steeringMotor.getSelectedSensorPosition(), state.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforward);
        steeringMotor.setVoltage(turnOutput + turnFeedforward);
    }

/** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0.0);
    steeringMotor.setSelectedSensorPosition(0.0);
  }
}
