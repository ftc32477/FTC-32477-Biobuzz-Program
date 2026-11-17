package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;
/**
 * 只包含 shooter（两个 5203 1:1）+ intake + 0 口送球舵机的独立测试程序。
 *
 * 特点：
 * - 两个发射轮各自一个 PIDF 速度环（K_P / K_I / K_D / K_F 在下面常量里改）。
 * - 目标转速用 RPM 设定，Dpad 上/下可现场加减，遥测实时显示目标 vs 实测。
 * - 转速门：两个轮都要在目标 ±RPM_TOLERANCE 以内、并稳定保持 SETTLE_MS，
 *   才允许按右扳机射球；否则扳机无效并在遥测里提示原因。
 * - 编码器：5203 1:1（5203-2402-0001）编码器在电机轴上，28 ticks/rev，
 *   1:1 输出轴也是 28 ticks/rev；速度用 getVelocity()（ticks/s）换算 RPM。
 *
 * 操作：
 *   LB        进料开/关
 *   A         按住 = intake 反转（吐球）
 *   RB        发射轮开/关
 *   右扳机     射球（必须：进料已开 + 转速门 READY）；一次送球 FEED_TIME_MS
 *   Dpad 上/下 目标转速 ±100 RPM
 */
@TeleOp(name = "Shooter: PIDF 转速门", group = "Test")
public class ShooterIntake extends LinearOpMode {
    // ===================== 硬件配置名 =====================
    private static final String[] SHOOTER_L_NAMES = {"shouterL", "shooterL", "ShouterL", "ShooterL"};
    private static final String[] SHOOTER_R_NAMES = {"shouterR", "shooterR", "ShouterR", "ShooterR"};
    private static final String[] INTAKE_NAMES = {"intake", "Intake"};
    private static final int FEED_SERVO_PORT = 0; // Control Hub 0 口
    // ===================== 转速与门限 =====================
    /**
     * 5203 1:1（5203-2402-0001）：编码器在电机轴上，28 ticks/rev，
     * 1:1 输出轴也是 28 ticks/rev；只有 19.2:1 才按 537.7 算。
     */
    private static final double TICKS_PER_REV = 28.0;
    /** 5203 1:1 空载 6000 RPM，留一点余量。 */
    private static final double MAX_RPM = 6000.0;
    private static final double DEFAULT_TARGET_RPM = 4500.0;
    private static final double TARGET_STEP_RPM = 100.0;
    /** 实测转速与目标相差多少 RPM 以内算“到速”。 */
    private static final double RPM_TOLERANCE = 100.0;
    /** 到速后还要稳定这么久才允许射球，防止刚冲上去就射。 */
    private static final long SETTLE_MS = 300L;
    // ===================== PIDF 系数 =====================
    // 误差单位是 ticks/s（目标 4500 RPM ≈ 2100 ticks/s），输出是 -1~1 的功率。
    // 调法：先只给 K_F（约 0.5/目标ticks），再慢慢加 K_P 到不抖，最后少量 K_I 消静差。
    private static final double K_P = 0.0006;
    private static final double K_I = 0.00015;
    private static final double K_D = 0.0;
    private static final double K_F = 0.00024;
    // ===================== 机构参数（沿用 RobotHardwareV0 已调好的值） =====================
    private static final double INTAKE_POWER = 0.8;
    private static final boolean INTAKE_REVERSED = false;
    /** 右发射轮与左轮反向：+1 同向，-1 反向。 */
    private static final int SHOOTER_R_SIGN = -1;
    private static final double FEED_SPEED = 0.50;
    private static final long FEED_TIME_MS = 1000L;
    private static final boolean FEED_SERVO_REVERSED = true;
    // ===================== 硬件对象 =====================
    private DcMotorEx shooterL;
    private DcMotorEx shooterR;
    private DcMotorEx intake;
    private Servo feedServo;
    private final VelocityPIDF pidL = new VelocityPIDF(K_P, K_I, K_D, K_F);
    private final VelocityPIDF pidR = new VelocityPIDF(K_P, K_I, K_D, K_F);
    // ===================== 运行状态 =====================
    private final ElapsedTime timer = new ElapsedTime();
    private double lastMs = 0.0;
    private double targetRpm = DEFAULT_TARGET_RPM;
    private boolean shooterOn = false;
    private boolean intakeOn = false;
    private double feedUntilMs = 0.0;
    private long readySinceMs = -1L;
    private String blockedReason = "";
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevTrigger = false;
    @Override
    public void runOpMode() {
        shooterL = findMotorEx(SHOOTER_L_NAMES);
        shooterR = findMotorEx(SHOOTER_R_NAMES);
        intake = findMotorEx(INTAKE_NAMES);
        feedServo = findServoByPort(FEED_SERVO_PORT);
        configure(shooterL);
        configure(shooterR);
        configure(intake);
        if (feedServo != null) {
            feedServo.setPosition(0.5);
        }
        telemetry.addLine("=== Shooter PIDF 转速门 ===");
        telemetry.addLine(shooterL != null ? "shooterL  OK  " + describe(shooterL) : "shooterL  ❌ 未找到");
        telemetry.addLine(shooterR != null ? "shooterR  OK  " + describe(shooterR) : "shooterR  ❌ 未找到");
        telemetry.addLine(intake != null ? "intake    OK  " + describe(intake) : "intake    ❌ 未找到");
        telemetry.addLine(feedServo != null ? "进料舵机  OK  端口 " + feedServo.getPortNumber()
                : "进料舵机  ❌ 端口 " + FEED_SERVO_PORT + " 未找到");
        telemetry.addLine();
        telemetry.addLine("目标转速 " + (int) targetRpm + " RPM");
        telemetry.addLine("LB 进料  A 反转  RB 发射轮  扳机 射球  Dpad 调转速");
        telemetry.update();
        waitForStart();
        timer.reset();
        lastMs = 0.0;
        readySinceMs = -1L;
        while (opModeIsActive()) {
            double now = timer.milliseconds();
            double dt = (now - lastMs) / 1000.0;
            lastMs = now;
            if (dt <= 0.0 || dt > 0.2) {
                dt = 0.02;
            }
            updateTargetRpm();
            updateToggles();
            // ---------- 两个发射轮的 PIDF ----------
            double targetTicks = rpmToTicks(targetRpm);
            double measuredL = velocityOf(shooterL);
            double measuredR = velocityOf(shooterR);
            double outL = 0.0;
            double outR = 0.0;
            if (shooterOn && shooterL != null && shooterR != null) {
                outL = pidL.update(targetTicks, measuredL, dt);
                outR = pidR.update(targetTicks, measuredR, dt);
                shooterL.setPower(outL);
                shooterR.setPower(SHOOTER_R_SIGN * outR);
            } else {
                if (shooterL != null) shooterL.setPower(0.0);
                if (shooterR != null) shooterR.setPower(0.0);
            }
            // ---------- intake ----------
            double intakePower = intakeOn ? INTAKE_POWER : 0.0;
            if (gamepad1.a) intakePower = -intakePower;          // 按住 A 反转
            if (INTAKE_REVERSED) intakePower = -intakePower;
            if (intake != null) intake.setPower(intakePower);
            // ---------- 转速门 ----------
            double rpmL = ticksToRpm(measuredL);
            double rpmR = ticksToRpm(measuredR);
            boolean bothAtSpeed = shooterOn
                    && shooterL != null && shooterR != null
                    && targetRpm > 0.0
                    && Math.abs(rpmL - targetRpm) <= RPM_TOLERANCE
                    && Math.abs(rpmR - targetRpm) <= RPM_TOLERANCE;
            if (bothAtSpeed) {
                if (readySinceMs < 0L) {
                    readySinceMs = (long) now;
                }
            } else {
                readySinceMs = -1L;
            }
            boolean ready = readySinceMs >= 0L && (now - readySinceMs) >= SETTLE_MS;
            // ---------- 射球 ----------
            boolean trigger = gamepad1.right_trigger > 0.5;
            if (trigger && !prevTrigger) {
                if (!shooterOn) {
                    blockedReason = "发射轮未开(RB)";
                } else if (!intakeOn) {
                    blockedReason = "进料未开(LB)";
                } else if (!ready) {
                    blockedReason = "转速未达标";
                } else {
                    feedUntilMs = now + FEED_TIME_MS;
                    blockedReason = "";
                }
            }
            prevTrigger = trigger;
            // ---------- 送球舵机 ----------
            boolean firing = now < feedUntilMs;
            if (feedServo != null) {
                if (firing) {
                    feedServo.setPosition(feedPosition(true));   // 与 intake 同向
                } else if (intakeOn) {
                    feedServo.setPosition(feedPosition(false));  // 进料时反向挡住球
                } else {
                    feedServo.setPosition(0.5);
                }
            }
            // ---------- 遥测 ----------
            telemetry.addLine("目标 " + (int) targetRpm + " RPM   (Dpad 上/下 ±" + (int) TARGET_STEP_RPM + ")");
            telemetry.addLine("实测 L " + String.format("%.1f", rpmL) + "  R " + String.format("%.1f", rpmR) + " RPM");
            telemetry.addLine("PIDF  L " + String.format("%.3f", outL) + "  R " + String.format("%.3f", outR));
            if (!shooterOn) {
                telemetry.addLine("转速门: 发射轮关闭 (RB)");
            } else if (ready) {
                telemetry.addLine("转速门: READY ✅");
            } else {
                telemetry.addLine(String.format("转速门: 等待...  ΔL %.1f  ΔR %.1f RPM",
                        Math.abs(rpmL - targetRpm), Math.abs(rpmR - targetRpm)));
            }
            telemetry.addLine(firing ? "送球: 正在送球 " + (int) (feedUntilMs - now) + " ms"
                    : (blockedReason.isEmpty() ? "送球: 待机" : "送球: " + blockedReason));
            telemetry.addLine("进料: " + (intakeOn ? "ON" : "OFF") + (gamepad1.a ? " (反转)" : "")
                    + "   发射轮: " + (shooterOn ? "ON" : "OFF"));
            telemetry.update();
        }
        // 退出时停机
        if (shooterL != null) shooterL.setPower(0.0);
        if (shooterR != null) shooterR.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
        if (feedServo != null) feedServo.setPosition(0.5);
    }
    // =====================================================================
    // 输入
    // =====================================================================
    private void updateTargetRpm() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !prevDpadUp) {
            targetRpm = clamp(targetRpm + TARGET_STEP_RPM, 0.0, MAX_RPM);
        }
        if (down && !prevDpadDown) {
            targetRpm = clamp(targetRpm - TARGET_STEP_RPM, 0.0, MAX_RPM);
        }
        prevDpadUp = up;
        prevDpadDown = down;
    }
    private void updateToggles() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLeftBumper) {
            intakeOn = !intakeOn;
        }
        prevLeftBumper = lb;

        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRightBumper) {
            shooterOn = !shooterOn;
            pidL.reset();
            pidR.reset();
            readySinceMs = -1L;
            blockedReason = "";
        }
        prevRightBumper = rb;
    }
    // =====================================================================
    // 硬件初始化
    // =====================================================================
    private void configure(DcMotorEx motor) {
        if (motor == null) return;
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private DcMotorEx findMotorEx(String... names) {
        for (String name : names) {
            DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
            if (motor != null) return motor;
        }
        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            for (String configured : hardwareMap.getNamesOf(motor)) {
                for (String name : names) {
                    if (configured.trim().equalsIgnoreCase(name)) return motor;
                }
            }
        }
        return null;
    }
    private Servo findServoByPort(int port) {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            Servo servo = entry.getValue();
            if (servo != null && servo.getPortNumber() == port) return servo;
        }
        return null;
    }
    private static String describe(DcMotorEx motor) {
        return "port " + motor.getPortNumber();
    }

    // =====================================================================
    // 换算
    // =====================================================================
    private static double rpmToTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
    private static double ticksToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }
    private static double velocityOf(DcMotorEx motor) {
        return motor == null ? 0.0 : Math.abs(motor.getVelocity());
    }
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    /** 送球舵机位置：same=true 与 intake 同向，false 反向。 */
    private static double feedPosition(boolean same) {
        double dir = (FEED_SERVO_REVERSED ? -1.0 : 1.0) * (INTAKE_REVERSED ? -1.0 : 1.0);
        double position = 0.5 + dir * (same ? FEED_SPEED : -FEED_SPEED);
        return clamp(position, 0.0, 1.0);
    }
    // =====================================================================
    // PIDF 速度环
    // =====================================================================
    private static final class VelocityPIDF {
        private static final double I_MAX_OUTPUT = 0.3;
        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;
        private double integral = 0.0;
        private double lastError = 0.0;
        VelocityPIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
        double update(double targetTicksPerSecond, double measuredTicksPerSecond, double dtSeconds) {
            double error = targetTicksPerSecond - measuredTicksPerSecond;
            integral += error * dtSeconds;
            if (kI != 0.0) {
                double limit = I_MAX_OUTPUT / Math.abs(kI);
                integral = clamp(integral, -limit, limit);
            }
            double derivative = dtSeconds > 1e-6 ? (error - lastError) / dtSeconds : 0.0;
            lastError = error;
            double output = kF * targetTicksPerSecond
                    + kP * error
                    + kI * integral
                    + kD * derivative;
            return clamp(output, -1.0, 1.0);
        }
        void reset() {
            integral = 0.0;
            lastError = 0.0;
        }
    }
}