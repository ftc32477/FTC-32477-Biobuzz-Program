package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * 32477 Biobuzz 赛季初代车硬件类 (V0)
 * 采用 Pedro Pathing 的 Mecanum 底盘配置
 */
public class RobotHardwareV0 {

    // ========================================================================
    // 1. 发射与进料机构参数
    // ========================================================================
    public static final double INTAKE_POWER = 0.8;
    public static final boolean INTAKE_REVERSED = false;

    public static final double SHOOTER_POWER = 0.6;
    public static final boolean SHOOTER_R_REVERSED = true;

    public static final double FEED_SPEED = 0.50;
    public static final long FEED_TIME_MS = 1000L;
    public static final boolean FEED_SERVO_REVERSED = true;

    // ========================================================================
    // 2. 前部双舵机三档位参数
    // ========================================================================
    public static final double FRONT_RANGE_DEG = 180.0;
    public static final boolean UPPER_REVERSED = false;
    public static final boolean LOWER_REVERSED = true;

    public static final double DEG_DOWN = 45.0;   // 下档：45°
    public static final double DEG_MIDDLE = 70.0; // 中档：70°
    public static final double DEG_LIFT = 105.0;  // 上档：105°

    public enum ServoPositionMode {
        DOWN,   // 下档 (45°)
        MIDDLE, // 中档 (70°)
        LIFT    // 上档 (105°)
    }

    // ========================================================================
    // 3. 端口与配置名称
    // ========================================================================
    public static final int FEED_SERVO_PORT = 0;   // Control Hub
    public static final int UPPER_SERVO_PORT = 1;  // Control Hub
    public static final int LOWER_SERVO_PORT = 2;  // Control Hub

    private static final String[] SHOOTER_L_NAMES = {"shouterL", "shooterL", "ShouterL", "ShooterL"};
    private static final String[] SHOOTER_R_NAMES = {"shouterR", "shooterR", "ShouterR", "ShooterR"};
    private static final String[] INTAKE_NAMES = {"intake", "Intake"};

    // ========================================================================
    // 4. 硬件对象
    // ========================================================================
    // Pedro Pathing 封装的底盘 Follower
    public Follower follower;

    public DcMotor shooterL, shooterR, intake;
    public Servo feedServo, frontUpper, frontLower;

    private HardwareMap hwMap;

    // ========================================================================
    // 5. 初始化
    // ========================================================================
    public void init(HardwareMap ahwMap) {
        this.hwMap = ahwMap;

        // 通过 Pedro Pathing 的 Constants 初始化底盘
        follower = Constants.create(ahwMap);

        // 初始化机构电机
        shooterL = findMotorOrNull(SHOOTER_L_NAMES);
        shooterR = findMotorOrNull(SHOOTER_R_NAMES);
        intake = findMotorOrNull(INTAKE_NAMES);

        configureMechanismMotor(shooterL);
        configureMechanismMotor(shooterR);
        configureMechanismMotor(intake);

        // 初始化舵机
        feedServo = findServoByPort(FEED_SERVO_PORT);
        frontUpper = findServoByPort(UPPER_SERVO_PORT);
        frontLower = findServoByPort(LOWER_SERVO_PORT);

        // 初始化默认设为上档 (105°)
        setFrontServosMode(ServoPositionMode.LIFT);
        stopFeedServo();
    }

    // ========================================================================
    // 6. 运动与机构控制方法
    // ========================================================================

    /**
     * TeleOp 底盘控制：通过 Pedro Follower 的 manual 方法驱动底盘
     */
    public void driveMecanum(double axial, double lateral, double yaw) {
        if (follower != null) {
            // Pedro Pathing 的 TeleOp 手动控制方法
            follower.manual(axial, lateral, yaw);
            follower.update();
        }
    }

    public double setIntakeActive(boolean active) {
        if (intake == null) return 0.0;
        double power = active ? (INTAKE_REVERSED ? -INTAKE_POWER : INTAKE_POWER) : 0.0;
        intake.setPower(power);
        return power;
    }

    public double setIntakeReverse() {
        if (intake == null) return 0.0;
        double power = INTAKE_REVERSED ? INTAKE_POWER : -INTAKE_POWER;
        intake.setPower(power);
        return power;
    }

    public void setShooterActive(boolean active) {
        double power = active ? SHOOTER_POWER : 0.0;
        if (shooterL != null) {
            shooterL.setPower(power);
        }
        if (shooterR != null) {
            shooterR.setPower(SHOOTER_R_REVERSED ? -power : power);
        }
    }

    public boolean isShooterReady(boolean shooterOn) {
        return shooterOn && shooterL != null && shooterR != null;
    }

    public double runFeedServo(boolean sameAsIntake) {
        double dir = (FEED_SERVO_REVERSED ? -1.0 : 1.0) * (INTAKE_REVERSED ? -1.0 : 1.0);
        double pos = clamp01(0.5 + dir * (sameAsIntake ? FEED_SPEED : -FEED_SPEED));
        if (feedServo != null) {
            feedServo.setPosition(pos);
        }
        return pos;
    }

    public double stopFeedServo() {
        if (feedServo != null) {
            feedServo.setPosition(0.5);
        }
        return 0.5;
    }

    public void setFrontServosMode(ServoPositionMode mode) {
        double targetAngle;
        switch (mode) {
            case DOWN:
                targetAngle = DEG_DOWN;   // 下档: 45°
                break;
            case MIDDLE:
                targetAngle = DEG_MIDDLE; // 中档: 70°
                break;
            case LIFT:
            default:
                targetAngle = DEG_LIFT;   // 上档: 105°
                break;
        }

        if (frontLower != null) {
            frontLower.setPosition(angleToPos(targetAngle, FRONT_RANGE_DEG, LOWER_REVERSED));
        }
        if (frontUpper != null) {
            frontUpper.setPosition(angleToPos(targetAngle, FRONT_RANGE_DEG, UPPER_REVERSED));
        }
    }

    public void stopAll() {
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
        if (shooterL != null) shooterL.setPower(0);
        if (shooterR != null) shooterR.setPower(0);
        if (intake != null) intake.setPower(0);
        stopFeedServo();
    }
    // ========================================================================
    // 7. 底层工具方法
    // ========================================================================
    private void configureMechanismMotor(DcMotor motor) {
        if (motor != null) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }

    private DcMotor findMotorOrNull(String... wantedNames) {
        for (String wanted : wantedNames) {
            DcMotor motor = hwMap.tryGet(DcMotor.class, wanted);
            if (motor != null) return motor;
        }
        for (DcMotor motor : hwMap.getAll(DcMotor.class)) {
            for (String configuredName : hwMap.getNamesOf(motor)) {
                for (String wanted : wantedNames) {
                    if (configuredName.trim().equalsIgnoreCase(wanted)) {
                        return motor;
                    }
                }
            }
        }
        return null;
    }

    private Servo findServoByPort(int port) {
        for (Map.Entry<String, Servo> entry : hwMap.servo.entrySet()) {
            if (entry.getValue().getPortNumber() == port) {
                return entry.getValue();
            }
        }
        return null;
    }

    private static double angleToPos(double angleDeg, double rangeDeg, boolean reversed) {
        double p = angleDeg / rangeDeg;
        return clamp01(reversed ? 1.0 - p : p);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}