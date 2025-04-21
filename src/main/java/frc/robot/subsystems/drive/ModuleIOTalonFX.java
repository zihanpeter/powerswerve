package frc.robot.subsystems.drive;

import static frc.lib.util.PhoenixUtil.tryUntilOk;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ModuleIOTalonFX implements ModuleIO{
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
    
    private final TalonFX driveTalon, turnTalon;
    private final CANcoder cancoder;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0);

    private final Queue<Double> timestampQueue, drivePositionQueue, turnPositionQueue;

    private final StatusSignal<Angle> drivePosition, turnAbsolutePosition, turnPosition;
    private final StatusSignal<AngularVelocity> driveVelocity, turnVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts, turnAppliedVolts;
    private final StatusSignal<Current> driveCurrent, turnCurrent;

    private final TalonFXConfiguration driveConfig, turnConfig;

    private BaseStatusSignal[] signals;

    private final Debouncer dirveConnectedDebounce = new Debouncer(0.5), turnConnectedDebounce = new Debouncer(0.5), turnEncoderConnectedDebounce = new Debouncer(0.5);
    
    public ModuleIOTalonFX(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus.getName());
        turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus.getName());
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus.getName());

        driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted =
            constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource =
            switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            case FusedCANdiPWM1 -> FeedbackSensorSourceValue.FusedCANdiPWM1;
            case FusedCANdiPWM2 -> FeedbackSensorSourceValue.FusedCANdiPWM2;
            case SyncCANdiPWM1 -> FeedbackSensorSourceValue.SyncCANdiPWM1;
            case SyncCANdiPWM2 -> FeedbackSensorSourceValue.SyncCANdiPWM2;
            case RemoteCANdiPWM1 -> FeedbackSensorSourceValue.RemoteCANdiPWM1;
            case RemoteCANdiPWM2 -> FeedbackSensorSourceValue.RemoteCANdiPWM2;
            case TalonFXS_PulseWidth -> FeedbackSensorSourceValue.RotorSensor;
            };
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration =
            turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted =
            constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection =
            constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

        tryUntilOk(
        10,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition));

        tryUntilOk(
            10,
            () ->
                BaseStatusSignal.setUpdateFrequencyForAll(
                    50.0,
                    driveVelocity,
                    driveAppliedVolts,
                    driveCurrent,
                    turnAbsolutePosition,
                    turnVelocity,
                    turnAppliedVolts,
                    turnCurrent));

        tryUntilOk(10, () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon));
        signals =
            new BaseStatusSignal[] {
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent,
            turnAbsolutePosition
            };
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
            case Voltage -> voltageRequest.withOutput(output);
            case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
            case Voltage -> voltageRequest.withOutput(output);
            case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            });
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        driveTalon.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
            case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
            case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
            });
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
            case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
            case TorqueCurrentFOC ->
                positionTorqueCurrentRequest.withPosition(rotation.getRotations());
            });
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveConfig.Slot0.kS = kS;
        driveConfig.Slot0.kV = kV;
        driveConfig.Slot0.kA = kA;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD, double kS, double kV, double kA) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kI = kI;
        turnConfig.Slot0.kD = kD;
        turnConfig.Slot0.kS = kS;
        turnConfig.Slot0.kV = kV;
        turnConfig.Slot0.kA = kA;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
    }
}