package org.firstinspires.ftc.teamcode.Exercise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

/**
 * ============================================================================
 * 方向测试：intake 电机（Expansion Hub 2 口） vs Control Hub 0 口舵机
 * ============================================================================
 *
 * 目的：确认"0 口舵机的反向/同向"到底是哪个位置，再决定 PracticeManual 里的
 *       FEED_SERVO_REVERSED 用 false 还是 true。
 *
 * 操作：
 *   左摇杆上下   = 直接控制 intake 电机（上 = 正转/进料方向）
 *   右摇杆上下   = 直接控制 0 口舵机位置（上 = 1.0，下 = 0.0，中间 0.5 = 停）
 *   A（按住）    = 预设"反向"：intake 正转 0.6，舵机 0.5 - 0.4
 *   B（按住）    = 预设"同向"：intake 正转 0.6，舵机 0.5 + 0.4
 *   Y            = 翻转舵机方向映射（观察 A/B 两种预设的差别）
 *
 * 判断方法：
 *   按住 A，看舵机转向与 intake 辊子的转向是否相反；按住 B，看是否相同。
 *   如果 A 实际是同向、B 实际是反向，就在 PracticeManual 里把
 *   FEED_SERVO_REVERSED 改成 true（或把这里的 Y 打开后再确认一次）。
 * ============================================================================
 */
@TeleOp(name = "Test: intake 舵机方向", group = "Test")
public class Test_FeedServo extends LinearOpMode {
    private static final int FEED_SERVO_PORT = 0;      // Control Hub
    /**
     * 连续模式下 1000~2000µs 对应 0.25~0.75（0.5 停）。
     * FTC 的 setPosition 0~1 默认对应 500~2500µs，所以全速用 0.25/0.75，不要用 0/1。
     */
    private static final double FEED_SPEED = 0.25;     // 预设测试速度
    private static final String[] INTAKE_NAMES = {"intake", "Intake"};
    private DcMotor intake;
    private Servo feedServo;
    private boolean reversed = false;
    private boolean yPrev = false;
    @Override
    public void runOpMode() {
        intake = findMotorOrNull(INTAKE_NAMES);
        feedServo = findServoByPort(FEED_SERVO_PORT);
        telemetry.addData("intake", intake == null ? "未找到（配置名 intake）" : "OK");
        telemetry.addData("0 口舵机", feedServo == null ? "未找到" : "OK");
        if (feedServo != null) {
            feedServo.setPosition(0.5);
        }
        telemetry.addLine("左摇杆=intake，右摇杆=舵机，A=反向预设，B=同向预设，Y=翻转映射");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // Y：翻转舵机方向映射
            boolean yNow = gamepad1.y;
            if (yNow && !yPrev) {
                reversed = !reversed;
            }
            yPrev = yNow;
            double dir = reversed ? -1.0 : 1.0;
            // 左摇杆：intake 功率（推上 = 正转）
            double intakePower = -gamepad1.left_stick_y;
            if (Math.abs(intakePower) < 0.05) {
                intakePower = 0.0;
            }
            // 右摇杆：舵机位置（推上 = 1.0）
            double servoPos = (1.0 - gamepad1.right_stick_y) / 2.0;
            // A/B 预设覆盖
            String preset = "手动";
            if (gamepad1.a) {
                intakePower = 0.6;
                servoPos = 0.5 - FEED_SPEED * dir;   // 反向
                preset = "A 反向";
            } else if (gamepad1.b) {
                intakePower = 0.6;
                servoPos = 0.5 + FEED_SPEED * dir;   // 同向
                preset = "B 同向";
            }
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            if (intake != null) {
                intake.setPower(intakePower);
            }
            if (feedServo != null) {
                feedServo.setPosition(servoPos);
            }
            double velocity = velocityOf(intake);
            telemetry.addData("预设", preset);
            telemetry.addData("intake 功率", "%.2f  (%.0f ticks/s)", intakePower, velocity);
            telemetry.addData("0 口舵机", "%.2f%s", servoPos, reversed ? "  [映射已翻转]" : "");
            telemetry.addData("提示", "看 A/B 时舵机与 intake 辊子的转向关系");
            telemetry.update();
        }
        if (intake != null) {
            intake.setPower(0);
        }
        if (feedServo != null) {
            feedServo.setPosition(0.5);
        }
    }
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
    private Servo findServoByPort(int port) {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            Servo servo = entry.getValue();
            if (servo.getPortNumber() == port) {
                return servo;
            }
        }
        return null;
    }
    /** DcMotor 接口没有 getVelocity，要用 DcMotorEx */
    private static double velocityOf(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) motor).getVelocity());
        }
        return 0.0;
    }
}
