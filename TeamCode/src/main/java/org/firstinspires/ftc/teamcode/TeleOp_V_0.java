package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 32477 Biobuzz 赛季初代车手动程序 (TeleOp_V_0)
 * 依赖硬件类：RobotHardwareV0
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

    // 当前舵机档位状态
    private RobotHardwareV0.ServoPositionMode currentServoMode = RobotHardwareV0.ServoPositionMode.DOWN;

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
        } catch (IllegalStateException e) {
            telemetry.addLine("底盘初始化失败：");
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
            // ---------- 底盘摇杆处理 ----------
            double rawAxial = curve(-gamepad1.left_stick_y);
            double rawLateral = curve(gamepad1.left_stick_x);
            double rawYaw = curve(gamepad1.right_stick_x);

            // 底盘慢速：按住左侧小扳机（left_bumper）时降低底盘灵敏度
            boolean chassisSlow = gamepad1.left_bumper;
            double slowScale = chassisSlow ? SLOW_MODE_SCALE : 1.0;
            double[] wheelPowers = robot.driveMecanum(rawAxial * slowScale, rawLateral * slowScale, rawYaw * slowScale);

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
                currentServoMode = RobotHardwareV0.ServoPositionMode.LIFT;       // 十字键上：抬起
            } else if (gamepad1.dpad_right || gamepad1.dpad_left) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.MIDDLE;    // 十字键左右：中置挡球
            } else if (gamepad1.dpad_down) {
                currentServoMode = RobotHardwareV0.ServoPositionMode.DOWN;      // 十字键下：放下贴地
            }
            robot.setFrontServosMode(currentServoMode);

            // ---------- 机构状态执行 ----------
            double intakePower = robot.setIntakeActive(intakeOn);
            robot.setShooterActive(shooterOn);

            boolean shooterReady = robot.isShooterReady(shooterOn);

            // ---------- 右扳机射球 ----------
            boolean trigger = gamepad1.right_trigger > 0.5;
            if (trigger && !triggerPrev && shooterReady && intakeOn) {
                feedUntil = timer.milliseconds() + RobotHardwareV0.FEED_TIME_MS;
            }
            triggerPrev = trigger;

            double now = timer.milliseconds();
            boolean firing = now < feedUntil;
            double feedPos;
            if (firing) {
                feedPos = robot.runFeedServo(true);
            } else if (intakeOn) {
                feedPos = robot.runFeedServo(false);
            } else {
                feedPos = robot.stopFeedServo();
            }

            // ---------- 遥测反馈 ----------
            telemetry.addData("摇杆", "前进 %.2f | 平移 %.2f | 旋转 %.2f", rawAxial, rawLateral, rawYaw);
            telemetry.addData("底盘", "LF %.2f RF %.2f LB %.2f RB %.2f", wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3]);
            telemetry.addData("进料 (LB)", "%s  功率 %.2f", intakeOn ? "开" : "关", intakePower);
            telemetry.addData("发射 (RB)", "%s (纯Power驱动)", shooterOn ? "开" : "关");
            telemetry.addData("前舵机档位 (Dpad)", "当前模式: %s", currentServoMode);
            telemetry.addData("射球", firing ? String.format("同向出球，还剩 %.0f ms", feedUntil - now) : "就绪（按右扳机）");
            telemetry.addData("0 口舵机", "%.2f", feedPos);
            telemetry.update();
        }

        robot.stopAll();
    }

    private void showInitTelemetry() {
        telemetry.addLine("==== 底盘硬件检查 ====");
        telemetry.addData("左前 LF", "%s", robot.frontLeftName);
        telemetry.addData("右前 RF", "%s", robot.frontRightName);
        telemetry.addData("左后 LB", "%s", robot.backLeftName);
        telemetry.addData("右后 RB", "%s", robot.backRightName);
        telemetry.addLine("==== 机构检查 ====");
        telemetry.addData("进料/发射", "intake: %s | shooterL/R: %s", robot.intake != null, robot.shooterL != null);
        telemetry.addData("舵机", "进料舵机(0): %s | 上方(1): %s | 下方(2): %s", robot.feedServo != null, robot.frontUpper != null, robot.frontLower != null);
        telemetry.addLine("------------------------------------");
        telemetry.addLine("LB = 进料开关   RB = 发射轮开关   右扳机 = 射球");
        telemetry.addLine("十字键上=抬起 | 十字键左右=中置挡球 | 十字键下=放下贴地");
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