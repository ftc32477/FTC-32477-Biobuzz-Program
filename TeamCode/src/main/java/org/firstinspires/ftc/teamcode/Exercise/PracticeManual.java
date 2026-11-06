package org.firstinspires.ftc.teamcode.Exercise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * ============================================================================
 * 32477 手动 OpMode：底盘（沿用 Test_dipan，不改） + 发射 + 进料 + 前部两个舵机
 * ============================================================================
 *
 * 【硬件映射】
 *   底盘（Control Hub 端口）：左前 LF=0、右前 RF=1、左后 LB=3、右后 RB=2
 *   发射（Expansion Hub）：shouterL=0 口、shouterR=1 口（5203 1:1，两轮转向相反）
 *   进料（Expansion Hub）：intake=2 口（5203 5.2:1）
 *   舵机（Control Hub）：0 口 = 进料/发射舵机（连续旋转，0.5 停）
 *                        1 口 = 上方舵机（车头两个抬升舵机之一）
 *                        2 口 = 下方舵机（另一个）
 *
 * 【动作】
 *   启动后：下方舵机 75°，上方舵机 75-33 = 42°（上方比下方低 33°，且下方方向与上方相反）
 *   A 键：进料开/关。开的时候：intake 转、0 口舵机反方向转、两个前舵机抬到
 *         下方 40° / 上方 7°；再按一次 A 关闭并回到 75°/42°。
 *   B 键：发射轮开/关。两个 shouter 电机转向相反。
 *   右扳机：射球。只有"发射轮已开启 + 转速达标 + 进料已开"时才有效：
 *         0 口舵机先按与 intake 相同的方向转 FEED_TIME_MS（默认 1000ms），
 *         1 秒一到立刻改成与 intake 相反的方向继续转（进料关掉才停）。
 *   底盘：左摇杆平移、右摇杆旋转、按住 left_bumper 慢速——和 Test_dipan 完全一样。
 *
 * 【方向/角度需要现场微调时，只改下面这些常量】
 *   INTAKE_REVERSED        intake 转反了 → true
 *   SHOOTER_R_REVERSED     右发射轮与左轮同向 → false
 *   FEED_SERVO_REVERSED    0 口舵机"反向"其实转反了 → true
 *   LOWER_REVERSED         下方舵机与前舵机同向 → false
 *   FRONT_RANGE_DEG        前舵机实际行程：REV SRS 一般是 180；goBILDA 2000 系列是 300
 *   SHOOTER_MIN_VELOCITY   发射轮"达标"的 ticks/s，看遥测里的实际转速再改
 * ============================================================================
 */
@TeleOp(name = "Practice3: 手动 + 发射 + 进料", group = "Practice")
public class PracticeManual extends LinearOpMode {

    // ========================================================================
    // 1. 底盘参数（与 Test_dipan 完全一致，不要改）
    // ========================================================================
    private static final double DEADZONE = 0.05;
    /** false = 速度与摇杆幅度成线性正比（推一半=半速）；true = 平方曲线（低速更细腻） */
    private static final boolean SQUARE_INPUT = false;
    private static final double SLOW_MODE_SCALE = 0.35;
    private static final boolean REAR_NAMES_SWAPPED = false;

    // ---- 实车方向修正（2026-11-06 实测现象）----
    // 现象：左摇杆上→后退、左→顺时针、右→逆时针；右摇杆左→右移、右→左移。
    // 说明：这辆车的前进方向整体反了，并且"平移/旋转"两个通道在物理上是互换的。
    // 修正后：左摇杆上下=前后、左右=平移；右摇杆左右=旋转。
    // 如果以后重新接线，把这三个 SIGN 和 SWAP 调回去即可。
    private static final boolean SWAP_LATERAL_YAW = true;
    private static final double AXIAL_SIGN = -1.0;
    private static final double LATERAL_SIGN = -1.0;
    private static final double YAW_SIGN = -1.0;

    private static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction LEFT_BACK_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_BACK_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // ========================================================================
    // 2. 机构参数
    // ========================================================================
    /** 进料电机功率 */
    private static final double INTAKE_POWER = 0.8;
    /** intake 实际转向与"进料"相反时改 true */
    private static final boolean INTAKE_REVERSED = false;

    /** 发射轮功率 */
    private static final double SHOOTER_POWER = 0.9;
    /** 右发射轮是否与左发射轮反向（两轮对转） */
    private static final boolean SHOOTER_R_REVERSED = true;
    /** 发射轮转速达到多少 ticks/s 才允许射球（看遥测实测值再改） */
    private static final double SHOOTER_MIN_VELOCITY = 5000.0;
    /** 关掉这个就不检查转速（编码器没接时用） */
    private static final boolean REQUIRE_SHOOTER_SPEED = true;

    /**
     * 0 口舵机（连续旋转模式）的转速偏移。
     * goBILDA 25-3 连续模式的 PWM 范围是 1000~2000µs：
     *   0.25（1000µs）= 一个方向全速，0.5（1500µs）= 停，0.75（2000µs）= 另一个方向全速。
     * FTC 的 setPosition 0~1 默认对应 500~2500µs，所以这里用 0.25 而不是 0.5，
     * 避免发出 500/2500µs 这种超出连续模式范围、可能被舵机判为无效的脉宽。
     */
    private static final double FEED_SPEED = 0.25;
    /** 每次射球，0 口舵机同向转多久 */
    private static final long FEED_TIME_MS = 1000L;
    /** 0 口舵机的"反向/同向"如果整体相反，改这个（2026-11-06 实测反了，已置 true） */
    private static final boolean FEED_SERVO_REVERSED = true;

    // ========================================================================
    // 3. 前部两个舵机参数
    // ========================================================================
    /** 舵机行程（度）：REV SRS 默认 180；goBILDA 2000 系列 300 */
    private static final double FRONT_RANGE_DEG = 180.0;
    /** 上方舵机方向 */
    private static final boolean UPPER_REVERSED = false;
    /** 下方舵机方向（按要求与上方相反） */
    private static final boolean LOWER_REVERSED = true;
    /** 启动位置：下方 75°，上方 75-33 = 42° */
    private static final double START_LOWER_DEG = 75.0;
    /** 上方比下方低多少度 */
    private static final double UPPER_OFFSET_DEG = 33.0;
    /** 按 A 抬起后：下方 40°，上方 7° */
    private static final double LIFT_LOWER_DEG = 45.0;

    // ========================================================================
    // 4. 端口
    // ========================================================================
    private static final int FEED_SERVO_PORT = 0;   // Control Hub
    private static final int UPPER_SERVO_PORT = 1;  // Control Hub
    private static final int LOWER_SERVO_PORT = 2;  // Control Hub

    private static final String[] SHOOTER_L_NAMES = {"shouterL", "shooterL", "ShouterL", "ShooterL"};
    private static final String[] SHOOTER_R_NAMES = {"shouterR", "shooterR", "ShouterR", "ShooterR"};
    private static final String[] INTAKE_NAMES = {"intake", "Intake"};

    // ========================================================================
    // 5. 硬件对象
    // ========================================================================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterL, shooterR, intake;
    private Servo feedServo, frontUpper, frontLower;

    private String frontLeftName, frontRightName, backLeftName, backRightName;

    // ========================================================================
    // 6. 状态
    // ========================================================================
    private boolean intakeOn = false;
    private boolean shooterOn = false;
    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean triggerPrev = false;
    /** 0 口舵机同向转动截止时刻（毫秒） */
    private double feedUntil = 0.0;
    /** 计时器（LinearOpMode 没有 runtime 字段，自己建一个） */
    private final ElapsedTime timer = new ElapsedTime();

    // ========================================================================
    // 7. 主程序
    // ========================================================================
    @Override
    public void runOpMode() {

        // ---------------- Init：底盘 ----------------
        try {
            frontLeft = findMotor("左前 (LF/端口0)", "lf", "LF");
            frontRight = findMotor("右前 (RF/端口1)", "rf", "RF");
            if (REAR_NAMES_SWAPPED) {
                backLeft = findMotor("左后 (端口3)", "rb", "RB");
                backRight = findMotor("右后 (端口2)", "lb", "LB");
            } else {
                backLeft = findMotor("左后 (LB/端口3)", "lb", "LB");
                backRight = findMotor("右后 (RB/端口2)", "rb", "RB");
            }
        } catch (IllegalStateException e) {
            telemetry.addLine("底盘初始化失败：");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            while (opModeInInit()) {
                sleep(50);
            }
            return;
        }

        frontLeftName = configNameOf(frontLeft);
        frontRightName = configNameOf(frontRight);
        backLeftName = configNameOf(backLeft);
        backRightName = configNameOf(backRight);

        configureMotor(frontLeft, LEFT_FRONT_DIRECTION);
        configureMotor(frontRight, RIGHT_FRONT_DIRECTION);
        configureMotor(backLeft, LEFT_BACK_DIRECTION);
        configureMotor(backRight, RIGHT_BACK_DIRECTION);

        // ---------------- Init：发射 / 进料 / 舵机 ----------------
        shooterL = findMotorOrNull(SHOOTER_L_NAMES);
        shooterR = findMotorOrNull(SHOOTER_R_NAMES);
        intake = findMotorOrNull(INTAKE_NAMES);

        if (shooterL != null) {
            shooterL.setDirection(DcMotor.Direction.FORWARD);
            shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterL.setPower(0);
        }
        if (shooterR != null) {
            shooterR.setDirection(DcMotor.Direction.FORWARD);
            shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterR.setPower(0);
        }
        if (intake != null) {
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0);
        }

        feedServo = findServoByPort(FEED_SERVO_PORT);
        frontUpper = findServoByPort(UPPER_SERVO_PORT);
        frontLower = findServoByPort(LOWER_SERVO_PORT);

        // 启动位置：下方 75°，上方 42°
        applyFrontServos(false);
        if (feedServo != null) {
            feedServo.setPosition(0.5);   // 连续舵机：停
        }

        // ---------------- Init 自检 ----------------
        telemetry.addLine("==== 底盘（与 Test_dipan 相同）====");
        telemetry.addData("左前 LF", "%s  dir=%s", frontLeftName, frontLeft.getDirection());
        telemetry.addData("右前 RF", "%s  dir=%s", frontRightName, frontRight.getDirection());
        telemetry.addData("左后 LB", "%s  dir=%s", backLeftName, backLeft.getDirection());
        telemetry.addData("右后 RB", "%s  dir=%s", backRightName, backRight.getDirection());
        telemetry.addLine("==== 发射 / 进料 ====");
        telemetry.addData("shouterL", shooterL == null ? "未找到（检查配置名 shouterL）" : "OK");
        telemetry.addData("shouterR", shooterR == null ? "未找到（检查配置名 shouterR）" : "OK");
        telemetry.addData("intake", intake == null ? "未找到（检查配置名 intake）" : "OK");
        telemetry.addData("0 口进料舵机", feedServo == null ? "未找到（Control Hub 0 口）" : "OK");
        telemetry.addData("1 口上方舵机", frontUpper == null ? "未找到（Control Hub 1 口）" : "OK");
        telemetry.addData("2 口下方舵机", frontLower == null ? "未找到（Control Hub 2 口）" : "OK");
        telemetry.addData("前舵机行程假设", "%.0f°  (改 FRONT_RANGE_DEG)", FRONT_RANGE_DEG);
        telemetry.addLine("------------------------------------");
        telemetry.addLine("左摇杆=平移，右摇杆=旋转，left bumper=慢速");
        telemetry.addLine("A=进料开/关   B=发射轮开/关   右扳机=射球");
        telemetry.update();

        waitForStart();
        timer.reset();

        // ---------------- Loop ----------------
        while (opModeIsActive()) {

            // ---------- 底盘（原样照搬 Test_dipan） ----------
            double axial = AXIAL_SIGN * curve(-gamepad1.left_stick_y);
            double lateral = LATERAL_SIGN * curve(gamepad1.left_stick_x);
            double yaw = YAW_SIGN * curve(gamepad1.right_stick_x);
            if (SWAP_LATERAL_YAW) {
                double t = lateral;
                lateral = yaw;
                yaw = t;
            }

            boolean slowMode = gamepad1.left_bumper;
            if (slowMode) {
                axial *= SLOW_MODE_SCALE;
                lateral *= SLOW_MODE_SCALE;
                yaw *= SLOW_MODE_SCALE;
            }

            double lfPower = axial + lateral + yaw;   // 左前
            double rfPower = axial - lateral - yaw;   // 右前
            double lbPower = axial - lateral + yaw;   // 左后
            double rbPower = axial + lateral - yaw;   // 右后

            double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower)));
            if (max > 1.0) {
                lfPower /= max;
                rfPower /= max;
                lbPower /= max;
                rbPower /= max;
            }

            frontLeft.setPower(lfPower);
            frontRight.setPower(rfPower);
            backLeft.setPower(lbPower);
            backRight.setPower(rbPower);

            // ---------- A：进料开/关 ----------
            boolean aNow = gamepad1.a;
            if (aNow && !aPrev) {
                intakeOn = !intakeOn;
            }
            aPrev = aNow;

            // ---------- B：发射轮开/关 ----------
            boolean bNow = gamepad1.b;
            if (bNow && !bPrev) {
                shooterOn = !shooterOn;
            }
            bPrev = bNow;

            // ---------- 进料电机 ----------
            double intakePower = 0.0;
            if (intakeOn && intake != null) {
                intakePower = INTAKE_REVERSED ? -INTAKE_POWER : INTAKE_POWER;
                intake.setPower(intakePower);
            } else if (intake != null) {
                intake.setPower(0);
            }

            // ---------- 发射电机（两轮反向） ----------
            double shooterPower = shooterOn ? SHOOTER_POWER : 0.0;
            if (shooterL != null) {
                shooterL.setPower(shooterPower);
            }
            if (shooterR != null) {
                shooterR.setPower(SHOOTER_R_REVERSED ? -shooterPower : shooterPower);
            }

            double vL = velocityOf(shooterL);
            double vR = velocityOf(shooterR);
            boolean atSpeed = !REQUIRE_SHOOTER_SPEED
                    || (vL >= SHOOTER_MIN_VELOCITY && vR >= SHOOTER_MIN_VELOCITY);
            boolean shooterReady = shooterOn && atSpeed && shooterL != null && shooterR != null;

            // ---------- 右扳机：射球（按下瞬间触发一次） ----------
            boolean trigger = gamepad1.right_trigger > 0.5;
            if (trigger && !triggerPrev && shooterReady && intakeOn) {
                feedUntil = timer.milliseconds() + FEED_TIME_MS;
            }
            triggerPrev = trigger;

            // ---------- 0 口舵机 ----------
            double now = timer.milliseconds();
            boolean firing = now < feedUntil;
            double feedPos;
            if (firing) {
                // 扳机后的 1 秒：与 intake 同向，把球送进发射轮
                feedPos = feedPosition(true);
            } else if (intakeOn) {
                // 其余时间（包括 1 秒结束之后）：与 intake 反向
                feedPos = feedPosition(false);
            } else {
                feedPos = 0.5;   // 进料没开，舵机停
            }
            if (feedServo != null) {
                feedServo.setPosition(feedPos);
            }

            // ---------- 前部两个舵机 ----------
            applyFrontServos(intakeOn);

            // ---------- 遥测 ----------
            telemetry.addData("摇杆", "前进 %.2f | 平移 %.2f | 旋转 %.2f%s",
                    axial, lateral, yaw, slowMode ? " [慢速]" : "");
            telemetry.addData("底盘", "LF %.2f RF %.2f LB %.2f RB %.2f",
                    lfPower, rfPower, lbPower, rbPower);
            telemetry.addData("进料", "%s  功率 %.2f", intakeOn ? "开" : "关", intakePower);
            telemetry.addData("发射", "%s  L %.0f / R %.0f ticks/s",
                    shooterOn ? "开" : "关", vL, vR);
            telemetry.addData("发射轮状态", shooterOn
                    ? (atSpeed ? "转速达标，可射球" : "加速中…（未达标）")
                    : "未开启（按 B）");
            telemetry.addData("射球", firing
                    ? String.format("同向出球，还剩 %.0f ms", feedUntil - now)
                    : (intakeOn ? "待触发；舵机反向转" : "按右扳机（需 A 已开 + 发射轮达标）"));
            telemetry.addData("前舵机", "下 %.0f° / 上 %.0f°（行程 %.0f°）",
                    intakeOn ? LIFT_LOWER_DEG : START_LOWER_DEG,
                    (intakeOn ? LIFT_LOWER_DEG : START_LOWER_DEG) - UPPER_OFFSET_DEG,
                    FRONT_RANGE_DEG);
            telemetry.addData("0 口舵机", "%.2f%s", feedPos, feedServo == null ? "（未找到）" : "");
            telemetry.update();
        }

        // ---------------- 停止：全部断电 ----------------
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        if (shooterL != null) shooterL.setPower(0);
        if (shooterR != null) shooterR.setPower(0);
        if (intake != null) intake.setPower(0);
        if (feedServo != null) feedServo.setPosition(0.5);
    }

    // ========================================================================
    // 8. 机构辅助方法
    // ========================================================================

    /**
     * 0 口连续舵机的位置。
     * sameAsIntake=true  → 与进料电机同向；false → 反向。
     * FEED_SERVO_REVERSED / INTAKE_REVERSED 都会翻转这个映射。
     */
    private double feedPosition(boolean sameAsIntake) {
        double dir = (FEED_SERVO_REVERSED ? -1.0 : 1.0) * (INTAKE_REVERSED ? -1.0 : 1.0);
        double p = 0.5 + dir * (sameAsIntake ? FEED_SPEED : -FEED_SPEED);
        return clamp01(p);
    }

    /** 前部两个舵机：intakeOn 时 下方 40°/上方 7°，否则 下方 75°/上方 42° */
    private void applyFrontServos(boolean lifted) {
        double lowerAngle = lifted ? LIFT_LOWER_DEG : START_LOWER_DEG;
        double upperAngle = lowerAngle - UPPER_OFFSET_DEG;
        if (frontLower != null) {
            frontLower.setPosition(angleToPos(lowerAngle, FRONT_RANGE_DEG, LOWER_REVERSED));
        }
        if (frontUpper != null) {
            frontUpper.setPosition(angleToPos(upperAngle, FRONT_RANGE_DEG, UPPER_REVERSED));
        }
    }

    /** 角度(度) → 舵机位置(0~1)；reversed 时取反（1 - p） */
    private static double angleToPos(double angleDeg, double rangeDeg, boolean reversed) {
        double p = angleDeg / rangeDeg;
        return clamp01(reversed ? 1.0 - p : p);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    /** 发射轮转速（ticks/s）；DcMotor 接口没有 getVelocity，要用 DcMotorEx */
    private static double velocityOf(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) motor).getVelocity());
        }
        return 0.0;
    }

    // ========================================================================
    // 9. 取硬件 / 底盘工具（与 Test_dipan 相同的写法）
    // ========================================================================

    /** 按配置名找电机；找不到就抛异常并列出配置里所有电机名 */
    private DcMotor findMotor(String position, String... wantedNames) {
        for (String wanted : wantedNames) {
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, wanted);
            if (motor != null) {
                return motor;
            }
        }
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            for (String configuredName : hardwareMap.getNamesOf(motor)) {
                for (String wanted : wantedNames) {
                    if (configuredName.trim().equalsIgnoreCase(wanted)) {
                        return motor;
                    }
                }
            }
        }
        throw new IllegalStateException(
                "找不到" + position + "电机：已尝试 " + Arrays.toString(wantedNames)
                        + "，但机器人配置里只有 " + hardwareMap.getAllNames(DcMotor.class)
                        + "。请到 Driver Station → Configure Robot 核对（配置名区分大小写）。");
    }

    /** 机构电机：找不到返回 null，由遥测提示，不崩溃 */
    private DcMotor findMotorOrNull(String... wantedNames) {
        for (String wanted : wantedNames) {
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, wanted);
            if (motor != null) {
                return motor;
            }
        }
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            for (String configuredName : hardwareMap.getNamesOf(motor)) {
                for (String wanted : wantedNames) {
                    if (configuredName.trim().equalsIgnoreCase(wanted)) {
                        return motor;
                    }
                }
            }
        }
        return null;
    }

    /** 按端口找舵机（配置名未知，只能按端口） */
    private Servo findServoByPort(int port) {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            Servo servo = entry.getValue();
            if (servo.getPortNumber() == port) {
                return servo;
            }
        }
        return null;
    }

    private double curve(double raw) {
        double magnitude = Math.abs(raw);
        if (magnitude <= DEADZONE) {
            return 0.0;
        }
        double scaled = (magnitude - DEADZONE) / (1.0 - DEADZONE);
        scaled = Math.min(scaled, 1.0);
        if (SQUARE_INPUT) {
            scaled = scaled * scaled;
        }
        return Math.copySign(scaled, raw);
    }

    private void configureMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    private String configNameOf(DcMotor motor) {
        List<String> names = new ArrayList<>(hardwareMap.getNamesOf(motor));
        Collections.sort(names);
        return names.isEmpty() ? "?" : names.get(0);
    }
}
