package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 32477 Biobuzz 赛季手动程序 TeleOpV0_1
 * (Pedro 底盘 + 同轴 PIDF 发射机构 + 前部三档舵机)
 *
 * 调试完成后可另行改名为 TeleOp_V_1 / TeleOpV1_0
 *
 * 键位说明：
 * - LB (左肩键): 进料开关切换 + 配合摇杆作慢速模式
 * - A (按住): 强制进料反转吐球，松开恢复
 * - RB (右肩键): 发射轮 PIDF 闭环开关
 * - 右扳机 (Right Trigger): 射球（需进料开 + 发射轮达到目标转速门）
 * - Dpad 上 / 左右 / 下: 前部双舵机三档位 (上档105° / 中档70° / 下档45°)
 * - X / B: 目标转速动态微调 +100 / -100 RPM
 * - Y: 恢复默认目标转速 (4500 RPM)
 */
@TeleOp(name = "TeleOpV0_1", group = "Competition")
public class TeleOpV0_1 extends LinearOpMode {

    private static final double DEADZONE = 0.05;
    private static final boolean SQUARE_INPUT = false;
    private static final double SLOW_MODE_SCALE = 0.35;
    private static final double RPM_STEP = 100.0;

    private final RobotHardwareV0 robot = new RobotHardwareV0();
    private ShooterSubsystem shooter;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean intakeOn = false;
    private double feedUntil = 0.0;
    private String fireStatus = "待机（按右扳机）";

    // 按键边缘检测
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
            // 绑定 V0 的左右发射电机到同轴 PIDF 子系统
            shooter = new ShooterSubsystem(robot.shooterL, robot.shooterR, RobotHardwareV0.SHOOTER_R_REVERSED);
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

        while (opModeIsActive()) {
            // ---------- 1. 底盘控制（Pedro 驱动，已修正左右平移与旋转方向） ----------
            double rawAxial = curve(-gamepad1.left_stick_y);
            double rawLateral = curve(-gamepad1.left_stick_x);
            double rawYaw = curve(-gamepad1.right_stick_x);

            boolean chassisSlow = gamepad1.left_bumper;
            double slowScale = chassisSlow ? SLOW_MODE_SCALE : 1.0;
            robot.driveMecanum(rawAxial * slowScale, rawLateral * slowScale, rawYaw * slowScale);

            // ---------- 2. 进料与发射开关 ----------
            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lbPrev) {
                intakeOn = !intakeOn;
            }
            lbPrev = lbNow;

            boolean rbNow = gamepad1.right_bumper;
            if (rbNow && !rbPrev) {
                shooter.setEnabled(!shooter.isEnabled());
                if (!shooter.isEnabled()) {
                    fireStatus = "发射轮已关闭";
                }
            }
            rbPrev = rbNow;

            // ---------- 3. 现场微调目标转速 (X+ / B- / Y重置) ----------
            boolean xNow = gamepad1.x;
            if (xNow && !xPrev) shooter.adjustTargetRpm(RPM_STEP);
            xPrev = xNow;

            boolean bNow = gamepad1.b;
            if (bNow && !bPrev) shooter.adjustTargetRpm(-RPM_STEP);
            bPrev = bNow;

            boolean yNow = gamepad1.y;
            if (yNow && !yPrev) shooter.resetTargetRpm();
            yPrev = yNow;

            // ---------- 4. 前部双舵机三档位（十字键控制） ----------
            if (gamepad1.dpad_up) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.LIFT;       // 上档: 105°
            } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.MIDDLE;    // 中档: 70°
            } else if (gamepad1.dpad_down) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.DOWN;      // 下档: 45°
            }
            robot.setFrontServosMode(currentServoMode);

            // ---------- 5. 机构动作执行 ----------
            boolean intakeReverseHold = gamepad1.a;
            double intakePower = intakeReverseHold
                    ? robot.setIntakeReverse()
                    : robot.setIntakeActive(intakeOn);

            // 更新同轴发射轮 PIDF
            shooter.update();
            boolean readyToFire = shooter.isReadyToFire();

            // ---------- 6. 右扳机触发射球（带转速门保护） ----------
            double now = timer.milliseconds();
            boolean trigger = gamepad1.right_trigger > 0.5;
            if (trigger && !triggerPrev) {
                if (!shooter.isEnabled()) {
                    fireStatus = "拦截：发射轮未开 (按RB)";
                } else if (!intakeOn) {
                    fireStatus = "拦截：进料未开 (按LB)";
                } else if (intakeReverseHold) {
                    fireStatus = "拦截：正在反转吐球";
                } else if (!readyToFire) {
                    fireStatus = "拦截：转速未稳定达标";
                } else {
                    feedUntil = now + RobotHardwareV0.FEED_TIME_MS;
                }
            }
            triggerPrev = trigger;

            boolean firing = now < feedUntil;
            double feedPos;
            if (firing) {
                feedPos = robot.runFeedServo(true);
                fireStatus = String.format("正在出球 (剩余 %.0f ms)", feedUntil - now);
            } else if (intakeReverseHold || intakeOn) {
                feedPos = robot.runFeedServo(false);
                if (readyToFire && intakeOn && !intakeReverseHold) {
                    fireStatus = "就绪（按右扳机发射）";
                }
            } else {
                feedPos = robot.stopFeedServo();
            }

            // ---------- 7. 遥测反馈 ----------
            telemetry.addData("发射状态 (RB)", "%s | 目标: %.0f RPM (X+/B-/Y重置)",
                    shooter.isEnabled() ? "开启" : "关闭", shooter.getTargetRpm());
            telemetry.addData("发射实测转速", "平均实测: %.0f RPM | 统一输出功率: %.2f",
                    shooter.getCurrentRpm(), shooter.getOutputPower());
            telemetry.addData("转速门状态", readyToFire ? "READY (可击发)" : "蓄力/调整中...");
            telemetry.addData("射球提示", fireStatus);
            telemetry.addLine("------------------------------------");
            telemetry.addData("进料 (LB/按住A反转)", "%s (功率 %.2f) | 0口舵机: %.2f",
                    intakeReverseHold ? "反转吐球" : (intakeOn ? "开" : "关"), intakePower, feedPos);
            telemetry.addData("前舵机档位 (Dpad)", "%s", currentServoMode);
            telemetry.update();
        }

        robot.stopAll();
    }

    private void showInitTelemetry() {
        telemetry.addLine("==== TeleOpV0_1 (Pedro 底盘 + 同轴 PIDF 发射) ====");
        telemetry.addData("Follower", robot.follower != null ? "已就绪" : "初始化异常");
        telemetry.addData("机构硬件", "intake: %s | shooterL: %s | shooterR: %s",
                robot.intake != null, robot.shooterL != null, robot.shooterR != null);
        telemetry.addData("舵机硬件", "0口: %s | 1口: %s | 2口: %s",
                robot.feedServo != null, robot.frontUpper != null, robot.frontLower != null);
        telemetry.addLine("------------------------------------");
        telemetry.addLine("LB=进料开关/慢速 | 按住A=反转 | RB=发射轮 | 右扳机=射球");
        telemetry.addLine("X=+100 RPM | B=-100 RPM | Y=重置默认 4500 RPM");
        telemetry.addLine("十字键：上=105°(初始) | 左右=70° | 下=45°");
        telemetry.update();
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
}