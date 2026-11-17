package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 32477 Biobuzz 赛季初代车手动程序 (TeleOp_V_0)
 * 依赖硬件类：RobotHardwareV0 (已接入 Pedro Pathing 底盘)
 *
 * 键位说明：
 * - LB: 进料开关切换（松开 A 后恢复此状态）
 * - A (按住): 强制进料反转吐球，松开结束
 * - RB: 发射轮开关切换
 * - 右扳机: 射球（需进料开 + 发射轮开）
 * - Dpad 上 / 左右 / 下: 前部双舵机三档位切换 (上105° / 中70° / 下45°)
 */
@TeleOp(name = "TeleOp_V_0", group = "Competition")
public class TeleOp_V_0 extends LinearOpMode {

    private static final double DEADZONE = 0.05;
    private static final boolean SQUARE_INPUT = false;
    private static final double SLOW_MODE_SCALE = 0.35;

    private final RobotHardwareV0 robot = new RobotHardwareV0();
    private final ElapsedTime timer = new ElapsedTime();

    private boolean intakeOn = false;
    private boolean shooterOn = false;

    // 按键边缘检测变量
    private boolean lbPrev = false;
    private boolean rbPrev = false;
    private boolean triggerPrev = false;
    private double feedUntil = 0.0;

    // 当前舵机档位状态：初始为上档（LIFT，105°）
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

        while (opModeIsActive()) {
            // ---------- 底盘摇杆处理（交由 Pedro Follower 驱动） ----------
            double rawAxial = curve(-gamepad1.left_stick_y);
            double rawLateral = curve(-gamepad1.left_stick_x);
            double rawYaw = curve(-gamepad1.right_stick_x);

            boolean chassisSlow = gamepad1.left_bumper;
            double slowScale = chassisSlow ? SLOW_MODE_SCALE : 1.0;
            robot.driveMecanum(rawAxial * slowScale, rawLateral * slowScale, rawYaw * slowScale);

            // ---------- 键位：left_bumper 进料开/关切换 ----------
            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lbPrev) {
                intakeOn = !intakeOn;
            }
            lbPrev = lbNow;

            // ---------- 键位：right_bumper 发射轮开/关切换 ----------
            boolean rbNow = gamepad1.right_bumper;
            if (rbNow && !rbPrev) {
                shooterOn = !shooterOn;
            }
            rbPrev = rbNow;

            // ---------- 前部舵机三档位键位控制（十字键） ----------
            if (gamepad1.dpad_up) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.LIFT;       // 十字键上：105°
            } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.MIDDLE;    // 十字键左右：70°
            } else if (gamepad1.dpad_down) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.DOWN;      // 十字键下：45°
            }
            robot.setFrontServosMode(currentServoMode);

            // ---------- 进料执行（按住 A 键优先反转吐球，松开恢复 LB 状态） ----------
            boolean intakeReverseHold = gamepad1.a;
            double intakePower;
            if (intakeReverseHold) {
                intakePower = robot.setIntakeReverse();
            } else {
                intakePower = robot.setIntakeActive(intakeOn);
            }

            // ---------- 发射执行 ----------
            robot.setShooterActive(shooterOn);
            boolean shooterReady = robot.isShooterReady(shooterOn);

            // ---------- 右扳机射球 ----------
            boolean trigger = gamepad1.right_trigger > 0.5;
            if (trigger && !triggerPrev && shooterReady && intakeOn && !intakeReverseHold) {
                feedUntil = timer.milliseconds() + RobotHardwareV0.FEED_TIME_MS;
            }
            triggerPrev = trigger;

            double now = timer.milliseconds();
            boolean firing = now < feedUntil;
            double feedPos;
            if (firing) {
                feedPos = robot.runFeedServo(true);
            } else if (intakeReverseHold) {
                feedPos = robot.runFeedServo(false);
            } else if (intakeOn) {
                feedPos = robot.runFeedServo(false);
            } else {
                feedPos = robot.stopFeedServo();
            }

            // ---------- 遥测反馈 ----------
            telemetry.addData("摇杆输入", "前进 %.2f | 平移 %.2f | 旋转 %.2f", rawAxial, rawLateral, rawYaw);
            telemetry.addData("进料 (LB/按住A反转)", "%s  功率 %.2f",
                    intakeReverseHold ? "【反转吐球中】" : (intakeOn ? "开" : "关"), intakePower);
            telemetry.addData("发射 (RB)", "%s  功率 %.2f", shooterOn ? "开" : "关", shooterOn ? RobotHardwareV0.SHOOTER_POWER : 0.0);
            telemetry.addData("前舵机档位 (Dpad)", "当前模式: %s", currentServoMode);
            telemetry.addData("射球", firing ? String.format("同向出球，还剩 %.0f ms", feedUntil - now) : "就绪（按右扳机）");
            telemetry.addData("0 口舵机", "%.2f", feedPos);
            telemetry.update();
        }

        robot.stopAll();
    }

    private void showInitTelemetry() {
        telemetry.addLine("==== 底盘配置 (Pedro Pathing) ====");
        telemetry.addData("Follower 状态", robot.follower != null ? "已就绪 (LF, RF, LB, RB)" : "初始化异常");
        telemetry.addLine("==== 机构检查 ====");
        telemetry.addData("进料/发射", "intake: %s | shooterL/R: %s", robot.intake != null, robot.shooterL != null);
        telemetry.addData("舵机", "进料舵机(0): %s | 上方(1): %s | 下方(2): %s", robot.feedServo != null, robot.frontUpper != null, robot.frontLower != null);
        telemetry.addLine("------------------------------------");
        telemetry.addLine("LB = 进料开关   按住A = 反转吐球   RB = 发射轮开关   右扳机 = 射球");
        telemetry.addLine("十字键上(初始)=105° | 左右=70° | 下=45°");
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