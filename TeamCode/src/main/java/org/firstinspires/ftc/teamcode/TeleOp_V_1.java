package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * 32477 Biobuzz 手动程序 V1 = TeleOp_V_0 + ShooterIntake 的 PIDF 转速门。
 *
 * 与 V0 的区别：
 * - 发射轮不再固定 0.6 功率，而是两个 5203 1:1 各跑一个 PIDF 速度环；
 * - 转速门：两个轮都进入目标 ±RPM_TOLERANCE 并稳定 SETTLE_MS 后，右扳机才允许射球；
 * - X / B 键现场加减目标转速（每步 100 RPM），Y 键恢复默认目标。
 *
 * 键位：
 *   LB        进料开关（按住 A 时反转优先）
 *   A (按住)  强制进料反转吐球
 *   RB        发射轮开关
 *   右扳机     射球（需进料开 + 转速门 READY）
 *   Dpad 上/左右/下  前部双舵机三档（105° / 70° / 45°）
 *   X / B     目标转速 +100 / -100 RPM
 *   Y         目标转速恢复默认
 *
 * 依赖：RobotHardwareV0（Pedro 底盘 + 机构）。
 */
@TeleOp(name = "TeleOp_V_1", group = "Competition")
public class TeleOp_V_1 extends LinearOpMode {
    // ===================== 底盘 =====================
    private static final double DEADZONE = 0.05;
    private static final boolean SQUARE_INPUT = false;
    private static final double SLOW_MODE_SCALE = 0.35;
    // ===================== 转速与门限（5203 1:1） =====================
    /** 5203 1:1：编码器在电机轴上，28 ticks/rev。 */
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;
    private static final double DEFAULT_TARGET_RPM = 4500.0;
    private static final double TARGET_STEP_RPM = 100.0;
    private static final double RPM_TOLERANCE = 100.0;
    private static final long SETTLE_MS = 300L;
    // ===================== PIDF =====================
    // 误差单位 ticks/s（4500 RPM ≈ 2100 ticks/s），输出 -1~1 功率。
    private static final double K_P = 0.0006;
    private static final double K_I = 0.00015;
    private static final double K_D = 0.0;
    private static final double K_F = 0.00024;
    private final RobotHardwareV0 robot = new RobotHardwareV0();
    private final ElapsedTime timer = new ElapsedTime();
    private final VelocityPIDF pidL = new VelocityPIDF(K_P, K_I, K_D, K_F);
    private final VelocityPIDF pidR = new VelocityPIDF(K_P, K_I, K_D, K_F);
    // ===================== 状态 =====================
    private boolean intakeOn = false;
    private boolean shooterOn = false;
    private double targetRpm = DEFAULT_TARGET_RPM;
    private double feedUntil = 0.0;
    private long readySinceMs = -1L;
    private double lastMs = 0.0;
    private String blockedReason = "";
    private boolean lbPrev = false;
    private boolean rbPrev = false;
    private boolean triggerPrev = false;
    private boolean xPrev = false;
    private boolean bPrev = false;
    private boolean yPrev = false;
    private RobotHardwareV0.ServoPositionMode currentServoMode = RobotHardwareV0.ServoPositionMode.LIFT;
    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("硬件初始化失败：");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            while (opModeInInit()) {
                sleep(50);
            }
            return;
        }
        showInitTelemetry();
        waitForStart();
        timer.reset();
        readySinceMs = -1L;
        while (opModeIsActive()) {
            double now = timer.milliseconds();
            double dt = (now - lastMs) / 1000.0;
            lastMs = now;
            if (dt <= 0.0 || dt > 0.2) {
                dt = 0.02;
            }
            // ---------- 底盘 ----------
            double rawAxial = curve(-gamepad1.left_stick_y);
            double rawLateral = curve(-gamepad1.left_stick_x);
            double rawYaw = curve(-gamepad1.right_stick_x);
            boolean chassisSlow = gamepad1.left_trigger > 0.5;
            double slowScale = chassisSlow ? SLOW_MODE_SCALE : 1.0;
            robot.driveMecanum(rawAxial * slowScale, rawLateral * slowScale, rawYaw * slowScale);
            // ---------- 进料开关 ----------
            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lbPrev) {
                intakeOn = !intakeOn;
            }
            lbPrev = lbNow;
            // ---------- 发射轮开关 ----------
            boolean rbNow = gamepad1.right_bumper;
            if (rbNow && !rbPrev) {
                shooterOn = !shooterOn;
                pidL.reset();
                pidR.reset();
                readySinceMs = -1L;
                blockedReason = "";
            }
            rbPrev = rbNow;
            // ---------- 目标转速：X 加，B 减，Y 恢复默认 ----------
            boolean xNow = gamepad1.x;
            if (xNow && !xPrev) {
                targetRpm = clamp(targetRpm + TARGET_STEP_RPM, 0.0, MAX_RPM);
            }
            xPrev = xNow;
            boolean bNow = gamepad1.b;
            if (bNow && !bPrev) {
                targetRpm = clamp(targetRpm - TARGET_STEP_RPM, 0.0, MAX_RPM);
            }
            bPrev = bNow;
            boolean yNow = gamepad1.y;
            if (yNow && !yPrev) {
                targetRpm = DEFAULT_TARGET_RPM;
            }
            yPrev = yNow;
            // ---------- 前部舵机三档 ----------
            if (gamepad1.dpad_up) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.LIFT;
            } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.MIDDLE;
            } else if (gamepad1.dpad_down) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.DOWN;
            }
            robot.setFrontServosMode(currentServoMode);
            // ---------- 进料 ----------
            boolean intakeReverseHold = gamepad1.a;
            double intakePower;
            if (intakeReverseHold) {
                intakePower = robot.setIntakeReverse();
            } else {
                intakePower = robot.setIntakeActive(intakeOn);
            }
            // ---------- 发射轮 PIDF ----------
            double targetTicks = rpmToTicks(targetRpm);
            double measuredL = velocityOf(robot.shooterL);
            double measuredR = velocityOf(robot.shooterR);
            double outL = 0.0;
            double outR = 0.0;
            if (shooterOn && robot.shooterL != null && robot.shooterR != null) {
                outL = pidL.update(targetTicks, measuredL, dt);
                outR = pidR.update(targetTicks, measuredR, dt);
                robot.shooterL.setPower(outL);
                robot.shooterR.setPower(RobotHardwareV0.SHOOTER_R_REVERSED ? -outR : outR);
            } else {
                if (robot.shooterL != null) robot.shooterL.setPower(0.0);
                if (robot.shooterR != null) robot.shooterR.setPower(0.0);
            }
            // ---------- 转速门 ----------
            double rpmL = ticksToRpm(measuredL);
            double rpmR = ticksToRpm(measuredR);
            boolean bothAtSpeed = shooterOn
                    && robot.shooterL != null && robot.shooterR != null
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
            if (trigger && !triggerPrev) {
                if (!shooterOn) {
                    blockedReason = "发射轮未开(RB)";
                } else if (!intakeOn) {
                    blockedReason = "进料未开(LB)";
                } else if (intakeReverseHold) {
                    blockedReason = "正在反转吐球";
                } else if (!ready) {
                    blockedReason = "转速未达标";
                } else {
                    feedUntil = now + RobotHardwareV0.FEED_TIME_MS;
                    blockedReason = "";
                }
            }
            triggerPrev = trigger;
            // ---------- 送球舵机 ----------
            boolean firing = now < feedUntil;
            double feedPos;
            if (firing) {
                feedPos = robot.runFeedServo(true);
            } else if (intakeReverseHold || intakeOn) {
                feedPos = robot.runFeedServo(false);
            } else {
                feedPos = robot.stopFeedServo();
            }
            // ---------- 遥测 ----------
            telemetry.addData("摇杆", "前进 %.2f | 平移 %.2f | 旋转 %.2f", rawAxial, rawLateral, rawYaw);
            telemetry.addData("进料 (LB / 按住A反转)", "%s  功率 %.2f",
                    intakeReverseHold ? "【反转】" : (intakeOn ? "开" : "关"), intakePower);
            telemetry.addData("目标转速 (X+/B-/Y重置)", "%d RPM", (int) targetRpm);
            telemetry.addData("实测转速", "L %.0f | R %.0f RPM", rpmL, rpmR);
            telemetry.addData("PIDF 输出", "L %.3f | R %.3f", outL, outR);
            telemetry.addData("转速门", ready ? "READY ✅"
                    : (shooterOn ? String.format("等待 ΔL %.0f ΔR %.0f", Math.abs(rpmL - targetRpm), Math.abs(rpmR - targetRpm))
                    : "发射轮关闭"));
            telemetry.addData("发射轮 (RB)", "%s", shooterOn ? "开" : "关");
            telemetry.addData("前舵机档位 (Dpad)", "%s", currentServoMode);
            telemetry.addData("射球", firing ? String.format("送球中，还剩 %.0f ms", feedUntil - now)
                    : (blockedReason.isEmpty() ? "待机（按右扳机）" : blockedReason));
            telemetry.addData("0 口舵机", "%.2f", feedPos);
            telemetry.update();
        }

        robot.stopAll();
    }
    private void showInitTelemetry() {
        telemetry.addLine("==== TeleOp_V_1（手动 + PIDF 发射） ====");
        telemetry.addData("Follower", robot.follower != null ? "已就绪" : "初始化异常");
        telemetry.addData("机构", "intake: %s | shooterL: %s | shooterR: %s",
                robot.intake != null, robot.shooterL != null, robot.shooterR != null);
        telemetry.addData("舵机", "0: %s | 1: %s | 2: %s",
                robot.feedServo != null, robot.frontUpper != null, robot.frontLower != null);
        telemetry.addLine("------------------------------------");
        telemetry.addLine("LB 进料  A 反转  RB 发射轮  右扳机 射球  按住左扳机 慢速");
        telemetry.addLine("Dpad 上/左右/下 = 105/70/45 度");
        telemetry.addLine("X +100 RPM  B -100 RPM  Y 恢复默认 " + (int) DEFAULT_TARGET_RPM);
        telemetry.update();
    }
    // =====================================================================
    // 工具
    // =====================================================================
    private static double velocityOf(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) motor).getVelocity());
        }
        return 0.0;
    }
    private static double rpmToTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
    private static double ticksToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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
