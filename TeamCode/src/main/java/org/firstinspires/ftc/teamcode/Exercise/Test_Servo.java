package org.firstinspires.ftc.teamcode.Exercise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * ============================================================================
 * 舵机旋转测试：上方舵机（端口 1）+ 下方舵机（端口 2）
 * ============================================================================
 *
 * 【这个程序做什么】
 *   1. 直接按“端口号”找舵机，不依赖机器人配置里的名字：
 *        上方 = 端口 1，下方 = 端口 2。
 *      INIT 遥测会显示找到的设备名和端口，先核对一眼。
 *   2. INIT 阶段两个舵机都放到 0.5。
 *   3. START 后自动来回扫：上方 0 → 1 → 0，下方 1 → 0 → 1，
 *      两个正好反相，方便一眼看出“哪个在动、往哪边动”。
 *   4. 按 Y 切换手动：左摇杆上下 = 上方舵机，右摇杆上下 = 下方舵机；
 *      A = 0.0，B = 0.5，X = 1.0（两个一起）。
 *
 * 【注意】
 *   · 位置 0.0 ~ 1.0 对应舵机行程两端。普通 180° 舵机：0.5 在中点；
 *     连续旋转舵机：0.5 = 停，0.0 / 1.0 = 两个方向的全速。
 *   · 遥测里的“≈ 度”按 180° 舵机估算，仅供参考；SRS 的实际行程
 *     取决于它被配置成多少度。
 *   · 如果两个舵机分别接在 Control Hub 和 Expansion Hub 上，
 *     两个 Hub 都可能各有“端口 1/2”，这时遥测会提示重复，需要改成按名字找。
 *   · REV SRS 堵转电流较大，如果一动就抖/复位，检查是否该用 Servo Power Module 单独供电。
 */
@TeleOp(name = "Test: 舵机旋转 (上1下2)", group = "Test")
public class Test_Servo extends LinearOpMode {
    /** 上方舵机的端口号 */
    private static final int TOP_PORT = 1;
    /** 下方舵机的端口号 */
    private static final int BOTTOM_PORT = 2;
    // 端口找不到时的兜底名字（一般用不到，按端口找更可靠）
    private static final String TOP_NAME = "servo1";
    private static final String BOTTOM_NAME = "servo2";
    // 某个舵机转向相反时改成 true
    private static final boolean TOP_REVERSED = false;
    private static final boolean BOTTOM_REVERSED = false;
    /** 自动扫描一个来回的时间（秒） */
    private static final double SWEEP_PERIOD_S = 3.0;
    /** 手动模式摇杆满推时每秒改变的位置量 */
    private static final double MANUAL_SPEED_PER_S = 0.6;
    private static final double STICK_DEADBAND = 0.15;
    private Servo servoTop;
    private Servo servoBottom;
    private boolean autoMode = true;
    private boolean yPressedLast = false;
    private double posTop = 0.5;
    private double posBottom = 0.5;
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        List<String> configured = listServoNames();
        // 优先按端口找：上=1，下=2
        servoTop = findByPort(TOP_PORT);
        servoBottom = findByPort(BOTTOM_PORT);
        // 端口没找到才退回按名字/配置顺序找
        if (servoTop == null) {
            servoTop = findServo(TOP_NAME);
        }
        if (servoBottom == null) {
            servoBottom = findServo(BOTTOM_NAME);
        }
        if (servoTop == null && configured.size() >= 1) {
            servoTop = findServo(configured.get(0));
        }
        if (servoBottom == null) {
            for (String name : configured) {
                Servo candidate = findServo(name);
                if (candidate != null && candidate != servoTop) {
                    servoBottom = candidate;
                    break;
                }
            }
        }
        if (servoTop != null) {
            servoTop.setDirection(TOP_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servoTop.setPosition(posTop);
        }
        if (servoBottom != null) {
            servoBottom.setDirection(BOTTOM_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servoBottom.setPosition(posBottom);
        }
        telemetry.addData("配置里的舵机", configured.isEmpty() ? "（一个都没找到）" : configured);
        telemetry.addData("上方舵机（端口 " + TOP_PORT + "）", describe(servoTop, TOP_REVERSED));
        telemetry.addData("下方舵机（端口 " + BOTTOM_PORT + "）", describe(servoBottom, BOTTOM_REVERSED));
        telemetry.addLine();
        telemetry.addLine("START 后自动 0→1→0 扫描；按 Y 切换手动");
        telemetry.addLine("手动：左摇杆Y=上方，右摇杆Y=下方；A=0，B=0.5，X=1");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lastTime = 0.0;
        while (opModeIsActive()) {
            double now = runtime.seconds();
            double dt = Math.min(now - lastTime, 0.05);
            lastTime = now;
            // Y 只在按下的瞬间切换一次
            boolean y = gamepad1.y;
            if (y && !yPressedLast) {
                autoMode = !autoMode;
            }
            yPressedLast = y;
            if (autoMode) {
                double phase = 2.0 * Math.PI * now / SWEEP_PERIOD_S;
                posTop = 0.5 - 0.5 * Math.cos(phase);     // 0 → 1 → 0
                posBottom = 0.5 + 0.5 * Math.cos(phase);  // 1 → 0 → 1
            } else {
                if (gamepad1.a) {
                    posTop = 0.0;
                    posBottom = 0.0;
                } else if (gamepad1.b) {
                    posTop = 0.5;
                    posBottom = 0.5;
                } else if (gamepad1.x) {
                    posTop = 1.0;
                    posBottom = 1.0;
                } else {
                    double stickTop = -gamepad1.left_stick_y;
                    double stickBottom = -gamepad1.right_stick_y;
                    if (Math.abs(stickTop) > STICK_DEADBAND) {
                        posTop = clamp01(posTop + stickTop * MANUAL_SPEED_PER_S * dt);
                    }
                    if (Math.abs(stickBottom) > STICK_DEADBAND) {
                        posBottom = clamp01(posBottom + stickBottom * MANUAL_SPEED_PER_S * dt);
                    }
                }
            }
            if (servoTop != null) {
                servoTop.setPosition(posTop);
            }
            if (servoBottom != null) {
                servoBottom.setPosition(posBottom);
            }
            telemetry.addData("模式", autoMode ? "自动扫描（Y 切手动）" : "手动（Y 切自动）");
            telemetry.addData("上方（端口 " + TOP_PORT + "）", describe(servoTop, TOP_REVERSED));
            telemetry.addData("  位置", "%.3f  ≈ %.0f°", posTop, posTop * 180.0);
            telemetry.addData("下方（端口 " + BOTTOM_PORT + "）", describe(servoBottom, BOTTOM_REVERSED));
            telemetry.addData("  位置", "%.3f  ≈ %.0f°", posBottom, posBottom * 180.0);
            telemetry.addData("摇杆", "左 %.2f / 右 %.2f", -gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.update();
        }
    }
    private List<String> listServoNames() {
        List<String> names = new ArrayList<>();
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            names.add(entry.getKey());
        }
        return names;
    }
    /** 按端口号找舵机；同一端口有多个时取第一个并提示 */
    private Servo findByPort(int port) {
        Servo found = null;
        int count = 0;
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            Servo servo = entry.getValue();
            if (servo.getPortNumber() == port) {
                count++;
                if (found == null) {
                    found = servo;
                }
            }
        }
        if (count > 1) {
            telemetry.addLine("注意：端口 " + port + " 找到 " + count + " 个舵机（可能跨了两个 Hub），已取第一个");
        }
        return found;
    }
    private Servo findServo(String name) {
        try {
            return hardwareMap.servo.get(name);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }
    private static String describe(Servo servo, boolean reversed) {
        if (servo == null) {
            return "未找到（检查端口/配置）";
        }
        return servo.getDeviceName() + " @端口" + servo.getPortNumber() + (reversed ? " [REVERSE]" : "");
    }
    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
