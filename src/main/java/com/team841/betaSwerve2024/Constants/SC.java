package com.team841.betaSwerve2024.Constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;

public class SC {

  public static class Shooter {
    private static AudioConfigs k_audio =
        new AudioConfigs()
            .withAllowMusicDurDisable(true)
            .withBeepOnBoot(true)
            .withBeepOnConfig(true);

    private static ClosedLoopRampsConfigs k_closedLoopRampConfig =
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.1)
            .withTorqueClosedLoopRampPeriod(0.1)
            .withVoltageClosedLoopRampPeriod(0.1);

    private static CurrentLimitsConfigs k_currentLimitsConfig =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60.0)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(60.0)
            .withSupplyTimeThreshold(0);

    private static CustomParamsConfigs k_customParamConfigs =
        new CustomParamsConfigs().withCustomParam0(841).withCustomParam1(841);

    private static MotionMagicConfigs k_shooterMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(200.0)
            .withMotionMagicCruiseVelocity(0.0)
            .withMotionMagicExpo_kA(0.0)
            .withMotionMagicExpo_kV(0.0)
            .withMotionMagicJerk(0.0);
    private static MotorOutputConfigs k_MotorOutputConfig =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    private static Slot0Configs k_slot0 =
        new Slot0Configs().withKP(0.6).withKI(0).withKD(0).withKV(0).withKS(0);

    public static TalonFXConfiguration k_BottomConfiguration =
        new TalonFXConfiguration()
            .withAudio(k_audio)
            .withClosedLoopRamps(k_closedLoopRampConfig)
            .withCurrentLimits(k_currentLimitsConfig)
            .withCustomParams(k_customParamConfigs)
            .withMotionMagic(k_shooterMotionMagicConfig)
            .withMotorOutput(k_MotorOutputConfig)
            .withSlot0(k_slot0);

    public static TalonFXConfiguration k_TopConfiguration =
        new TalonFXConfiguration()
            .withAudio(k_audio)
            .withClosedLoopRamps(k_closedLoopRampConfig)
            .withCurrentLimits(k_currentLimitsConfig)
            .withCustomParams(k_customParamConfigs)
            .withMotionMagic(k_shooterMotionMagicConfig)
            .withMotorOutput(k_MotorOutputConfig)
            .withSlot0(k_slot0);
  }

  public static class Indexer {

    private static AudioConfigs k_audio =
        new AudioConfigs()
            .withAllowMusicDurDisable(true)
            .withBeepOnBoot(true)
            .withBeepOnConfig(true);

    private static ClosedLoopRampsConfigs k_closedLoopRampConfig =
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.1)
            .withTorqueClosedLoopRampPeriod(0.1)
            .withVoltageClosedLoopRampPeriod(0.1);

    private static CurrentLimitsConfigs k_currentLimitsConfig =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60.0)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(60.0)
            .withSupplyTimeThreshold(0);

    private static CustomParamsConfigs k_customParamConfigs =
        new CustomParamsConfigs().withCustomParam0(841).withCustomParam1(841);

    private static MotionMagicConfigs k_IndexerMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(0.0)
            .withMotionMagicCruiseVelocity(0.0)
            .withMotionMagicExpo_kA(0.0)
            .withMotionMagicExpo_kV(0.12)
            .withMotionMagicJerk(0.0);
    private static MotorOutputConfigs k_IndexerMotorOutputConfig =
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

    // with note
    private static Slot0Configs k_slot0 =
        new Slot0Configs().withKP(60).withKI(0).withKD(0.1).withKV(0.12).withKS(0.5);

    // No Note
    private static Slot1Configs k_slot1 =
        new Slot1Configs().withKP(60).withKI(0).withKD(0.1).withKV(0.12).withKS(0.5);
    public static TalonFXConfiguration k_IndexerConfiguration =
        new TalonFXConfiguration()
            .withAudio(k_audio)
            .withClosedLoopRamps(k_closedLoopRampConfig)
            .withCurrentLimits(k_currentLimitsConfig)
            .withCustomParams(k_customParamConfigs)
            .withMotionMagic(k_IndexerMotionMagicConfig)
            .withMotorOutput(k_IndexerMotorOutputConfig)
            .withSlot0(k_slot0)
            .withSlot1(k_slot1);

    public static int k_IndexerSensorChannel = 1;
  }

  public static class Intake {
    public static final int currentLimit = 60; // in amps
  }

  public static class Arm {
    private static AudioConfigs k_audio =
        new AudioConfigs()
            .withAllowMusicDurDisable(true)
            .withBeepOnBoot(true)
            .withBeepOnConfig(true);

    private static ClosedLoopRampsConfigs k_closedLoopRampConfig =
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.05)
            .withTorqueClosedLoopRampPeriod(0.05)
            .withVoltageClosedLoopRampPeriod(0.1);

    private static CurrentLimitsConfigs k_currentLimitsConfig =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(10)
            .withSupplyTimeThreshold(0);

    private static CustomParamsConfigs k_customParamConfigs =
        new CustomParamsConfigs().withCustomParam0(841).withCustomParam1(841);

    private static MotionMagicConfigs k_ArmMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(200.0)
            .withMotionMagicCruiseVelocity(0.0)
            .withMotionMagicExpo_kA(0.0)
            .withMotionMagicExpo_kV(0.0)
            .withMotionMagicJerk(0.0);

    private static Slot0Configs k_slot0 =
        new Slot0Configs().withKP(0.6).withKI(0).withKD(0).withKV(0).withKS(0);

    public static TalonFXConfiguration k_ArmConfiguration =
        new TalonFXConfiguration()
            .withAudio(k_audio)
            .withClosedLoopRamps(k_closedLoopRampConfig)
            .withCurrentLimits(k_currentLimitsConfig)
            .withCustomParams(k_customParamConfigs)
            .withMotionMagic(k_ArmMotionMagicConfig)
            .withSlot0(k_slot0);
  }
}
