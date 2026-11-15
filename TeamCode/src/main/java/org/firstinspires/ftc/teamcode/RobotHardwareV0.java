package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * 32477 Biobuzz 赛季初代车硬件类 (V0)
 * 更新：
 * 1. 前部双舵机支持三档位独立控制（放下贴地、中置挡球、抬起）
 * 2. 纯 Power 发射驱动，连续舵机加速
 */
public class RobotHardwareV0 {

    // ========================================================================
    // 1. 底盘参数与实车方向修正
    // ========================================================================
    public static final boolean REAR_NAMES_SWAPPED = false;
    public static final boolean SWAP_LATERAL_YAW = true;
    public static final double AXIAL_SIGN = -1.0;
    public static final double LATERAL_SIGN = -1.0;
    public static final double YAW_SIGN = -1.0;

    public static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction LEFT_BACK_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction RIGHT_BACK_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior CHASSIS_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;

    // ========================================================================
    // 2. 发射与进料机构参数
    // ========================================================================
    public static final double INTAKE_POWER = 0.8;
    public static final boolean INTAKE_REVERSED = false;

    public static final double SHOOTER_POWER = 0.6;
    public static final boolean SHOOTER_R_REVERSED = true;

    public static final double FEED_SPEED = 0.40;
    public static final long FEED_TIME_MS = 1000L;
    public static final boolean FEED_SERVO_REVERSED = true;

    // ========================================================================
    // 3. 前部双舵机三档位参数（在此处微调角度）
    // ========================================================================
    public static final double FRONT_RANGE_DEG = 180.0;
    public static final boolean UPPER_REVERSED = false;
    public static final boolean LOWER_REVERSED = true;
    public static final double UPPER_OFFSET_DEG = 33.0;

    // 档位定义：下方舵机目标角度
    public static final double LOWER_DEG_DOWN = 12.0;   // 档位1：放下贴地
    public static final double LOWER_DEG_MIDDLE = 45.0; // 档位2：中置挡球
    public static final double LOWER_DEG_LIFT = 75.0;   // 档位3：抬起

    // 舵机档位枚举
    public enum ServoPositionMode {
        DOWN,   // 放下贴地
        MIDDLE, // 中置挡球
        LIFT    // 抬起
    }

    // ========================================================================
    // 4. 端口与配置名称
    // ========================================================================
    public static final int FEED_SERVO_PORT = 0;   // Control Hub
    public static final int UPPER_SERVO_PORT = 1;  // Control Hub
    public static final int LOWER_SERVO_PORT = 2;  // Control Hub

    private static final String[] SHOOTER_L_NAMES = {"shouterL", "shooterL", "ShouterL", "ShooterL"};
    private static final String[] SHOOTER_R_NAMES = {"shouterR", "shooterR", "ShouterR", "ShooterR"};
    private static final String[] INTAKE_NAMES = {"intake", "Intake"};

    // ========================================================================
    // 5. 硬件对象
    // ========================================================================
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor shooterL, shooterR, intake;
    public Servo feedServo, frontUpper, frontLower;

    public String frontLeftName, frontRightName, backLeftName, backRightName;

    private HardwareMap hwMap;

    // ========================================================================
    // 6. 初始化
    // ========================================================================
    public void init(HardwareMap ahwMap) {
        this.hwMap = ahwMap;

        frontLeft = findMotor("左前 (LF/端口0)", "lf", "LF");
        frontRight = findMotor("右前 (RF/端口1)", "rf", "RF");
        if (REAR_NAMES_SWAPPED) {
            backLeft = findMotor("左后 (端口3)", "rb", "RB");
            backRight = findMotor("右后 (端口2)", "lb", "LB");
        } else {
            backLeft = findMotor("左后 (LB/端口3)", "lb", "LB");
            backRight = findMotor("右后 (RB/端口2)", "rb", "RB");
        }

        frontLeftName = configNameOf(frontLeft);
        frontRightName = configNameOf(frontRight);
        backLeftName = configNameOf(backLeft);
        backRightName = configNameOf(backRight);

        configureChassisMotor(frontLeft, LEFT_FRONT_DIRECTION);
        configureChassisMotor(frontRight, RIGHT_FRONT_DIRECTION);
        configureChassisMotor(backLeft, LEFT_BACK_DIRECTION);
        configureChassisMotor(backRight, RIGHT_BACK_DIRECTION);

        shooterL = findMotorOrNull(SHOOTER_L_NAMES);
        shooterR = findMotorOrNull(SHOOTER_R_NAMES);
        intake = findMotorOrNull(INTAKE_NAMES);

        configureMechanismMotor(shooterL);
        configureMechanismMotor(shooterR);
        configureMechanismMotor(intake);

        feedServo = findServoByPort(FEED_SERVO_PORT);
        frontUpper = findServoByPort(UPPER_SERVO_PORT);
        frontLower = findServoByPort(LOWER_SERVO_PORT);

        setFrontServosMode(ServoPositionMode.DOWN);
        stopFeedServo();
    }

    // ========================================================================
    // 7. 运动与机构控制方法
    // ========================================================================

    public double[] driveMecanum(double rawAxial, double rawLateral, double rawYaw) {
        double axial = AXIAL_SIGN * rawAxial;
        double lateral = LATERAL_SIGN * rawLateral;
        double yaw = YAW_SIGN * rawYaw;

        if (SWAP_LATERAL_YAW) {
            double temp = lateral;
            lateral = yaw;
            yaw = temp;
        }

        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);

        return new double[]{lf, rf, lb, rb};
    }

    public double setIntakeActive(boolean active) {
        if (intake == null) return 0.0;
        double power = active ? (INTAKE_REVERSED ? -INTAKE_POWER : INTAKE_POWER) : 0.0;
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

    /** 设置前部双舵机三档位 */
    public void setFrontServosMode(ServoPositionMode mode) {
        double lowerAngle;
        switch (mode) {
            case MIDDLE:
                lowerAngle = LOWER_DEG_MIDDLE;
                break;
            case LIFT:
                lowerAngle = LOWER_DEG_LIFT;
                break;
            case DOWN:
            default:
                lowerAngle = LOWER_DEG_DOWN;
                break;
        }
        double upperAngle = lowerAngle - UPPER_OFFSET_DEG;
        if (frontLower != null) {
            frontLower.setPosition(angleToPos(lowerAngle, FRONT_RANGE_DEG, LOWER_REVERSED));
        }
        if (frontUpper != null) {
            frontUpper.setPosition(angleToPos(upperAngle, FRONT_RANGE_DEG, UPPER_REVERSED));
        }
    }

    public void stopAll() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
        if (shooterL != null) shooterL.setPower(0);
        if (shooterR != null) shooterR.setPower(0);
        if (intake != null) intake.setPower(0);
        stopFeedServo();
    }

    // ========================================================================
    // 8. 底层工具方法
    // ========================================================================
    private void configureChassisMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(CHASSIS_ZERO_POWER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    private void configureMechanismMotor(DcMotor motor) {
        if (motor != null) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }

    private DcMotor findMotor(String position, String... wantedNames) {
        DcMotor motor = findMotorOrNull(wantedNames);
        if (motor != null) {
            return motor;
        }
        throw new IllegalStateException(
                "找不到" + position + "电机：已尝试 " + Arrays.toString(wantedNames)
                        + "，但机器人配置里只有 " + hwMap.getAllNames(DcMotor.class)
                        + "。请到 Driver Station → Configure Robot 核对。");
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

    private String configNameOf(DcMotor motor) {
        List<String> names = new ArrayList<>(hwMap.getNamesOf(motor));
        Collections.sort(names);
        return names.isEmpty() ? "?" : names.get(0);
    }

    private static double angleToPos(double angleDeg, double rangeDeg, boolean reversed) {
        double p = angleDeg / rangeDeg;
        return clamp01(reversed ? 1.0 - p : p);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}