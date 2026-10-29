package 程序组代码练习;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * ============================================================================
 * 底盘自检工具 —— 核对"配置名字 / 电机端口 / 物理位置 / 转向"是否一致
 * ============================================================================
 *
 * 用途：麦轮底盘最容易错的不是公式，而是"哪个名字的电机到底装在哪、往哪边转"。
 *       这个程序把答案直接做成了 3 个测试，不用改代码、不用猜。
 *
 * 把车架空（轮子离地）或放在空旷地上，按下面的顺序按一遍：
 *
 *   A 键 → 四轮全部"前进"功率
 *          预期：车整体向前走
 *          如果：车整体后退 → 说明左右两个方向符号整体反了，
 *                请把 Test_dipan.java 里 4 个 *_DIRECTION 常量全部对调
 *                （REVERSE ↔ FORWARD）
 *
 *   B 键 → 麦轮"右平移"标准图案（左前+/右前-/左后-/右后+）
 *          预期：车整体向右平移
 *          如果：车原地打转 → 说明"左后/右后"两个名字与物理位置对应反了，
 *                请把 Test_dipan.java 里 REAR_NAMES_SWAPPED 改成 true
 *          如果：车整体向左平移 → 方向符号整体反了（同 A 键的处理）
 *
 *   X 键 → 停止所有轮子，回到单轮手动模式
 *
 *   ← / → 键 → 切换"选中的轮子"（左前 → 右前 → 左后 → 右后 循环）
 *   ↑ / ↓ 键（按住）→ 只让选中的那个轮子正转 / 反转
 *          预期：转的必须是你以为的那个物理轮子
 *          这一步能确认"名字 ↔ 端口 ↔ 物理位置"三者的对应关系，
 *          遥测里会显示每个名字实际读到的电机端口号。
 *
 * 遥控器映射与 Test_dipan.java 完全一致（左侧 REVERSE、右侧 FORWARD），
 * 所以这里测出来的结论可以直接搬到 Test_dipan.java。
 * ============================================================================
 */
@TeleOp(name = "Test Wheel Check (轮子自检)", group = "Linear OpMode")
public class Test_WheelCheck extends LinearOpMode {

    // ===== 与 Test_dipan.java 保持一致的 4 个设置 =====
    private static final boolean REAR_NAMES_SWAPPED = false;
    private static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction LEFT_BACK_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_BACK_DIRECTION = DcMotor.Direction.FORWARD;

    /** 自检用的功率（别给太大，轮子离地时全速转很吓人） */
    private static final double TEST_POWER = 0.3;

    /** 顺序：0=左前 1=右前 2=左后 3=右后 */
    private static final String[] PHYSICAL_LABEL = {"左前 LF", "右前 RF", "左后 LB", "右后 RB"};

    private final DcMotor[] motors = new DcMotor[4];

    /** 实际匹配到的"配置名"，只用于遥测显示（不是 getDeviceName()，那返回的是设备型号） */
    private final String[] configuredName = new String[4];

    private int selected = 0;
    /** 0 = 单轮手动（方向键控制）, 1 = 全轮前进, 2 = 右平移测试 */
    private int mode = 0;

    @Override
    public void runOpMode() {

        // ---------------- Init ----------------
        try {
            motors[0] = findMotor(hardwareMap, PHYSICAL_LABEL[0] + " / 端口0", "lf", "LF");
            motors[1] = findMotor(hardwareMap, PHYSICAL_LABEL[1] + " / 端口1", "rf", "RF");
            if (REAR_NAMES_SWAPPED) {
                motors[2] = findMotor(hardwareMap, "左后 / 端口3", "rb", "RB");
                motors[3] = findMotor(hardwareMap, "右后 / 端口2", "lb", "LB");
            } else {
                motors[2] = findMotor(hardwareMap, "左后 LB / 端口3", "lb", "LB");
                motors[3] = findMotor(hardwareMap, "右后 RB / 端口2", "rb", "RB");
            }
        } catch (IllegalStateException e) {
            telemetry.addLine("底盘自检初始化失败：");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            while (opModeInInit()) {
                sleep(50);
            }
            return;
        }

        DcMotor.Direction[] directions = {
                LEFT_FRONT_DIRECTION, RIGHT_FRONT_DIRECTION, LEFT_BACK_DIRECTION, RIGHT_BACK_DIRECTION
        };
        for (int i = 0; i < 4; i++) {
            motors[i].setDirection(directions[i]);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setPower(0);
            configuredName[i] = configNameOf(motors[i]);
        }

        telemetry.addLine("==== 底盘自检：请核对下面 4 行的 名字 / 端口 / 方向 ====");
        for (int i = 0; i < 4; i++) {
            telemetry.addData(PHYSICAL_LABEL[i],
                    "名字=%s  端口=%s  方向=%s",
                    configuredName[i], portLabel(motors[i]), motors[i].getDirection());
        }
        telemetry.addLine("A=全轮前进  B=右平移测试  X=停止/单轮模式");
        telemetry.addLine("←/→ 选轮子，↑/↓ 按住让选中轮子转");
        telemetry.update();

        waitForStart();

        // ---------------- Loop ----------------
        while (opModeIsActive()) {

            // 1) 选择轮子
            if (gamepad1.dpadLeftWasPressed()) {
                selected = (selected + 3) % 4;
            }
            if (gamepad1.dpadRightWasPressed()) {
                selected = (selected + 1) % 4;
            }

            // 2) 切换测试模式
            if (gamepad1.aWasPressed()) {
                mode = 1;
            }
            if (gamepad1.bWasPressed()) {
                mode = 2;
            }
            if (gamepad1.xWasPressed()) {
                mode = 0;
            }

            // 3) 计算四个轮子的功率
            double[] power = new double[4];
            if (mode == 1) {
                // 全轮前进
                for (int i = 0; i < 4; i++) {
                    power[i] = TEST_POWER;
                }
            } else if (mode == 2) {
                // 麦轮右平移标准图案：左前 + / 右前 - / 左后 - / 右后 +
                power[0] = TEST_POWER;
                power[1] = -TEST_POWER;
                power[2] = -TEST_POWER;
                power[3] = TEST_POWER;
            } else {
                // 单轮手动
                if (gamepad1.dpad_up) {
                    power[selected] = TEST_POWER;
                } else if (gamepad1.dpad_down) {
                    power[selected] = -TEST_POWER;
                }
            }

            for (int i = 0; i < 4; i++) {
                motors[i].setPower(power[i]);
            }

            // 4) 遥测
            if (mode == 1) {
                telemetry.addLine("【当前测试】A：全轮前进 → 预期车整体向前");
                telemetry.addLine("  若车后退 = 方向符号整体反了（对调 4 个 *_DIRECTION）");
            } else if (mode == 2) {
                telemetry.addLine("【当前测试】B：右平移 → 预期车整体向右平移");
                telemetry.addLine("  若原地打转 = 左后/右后名字对反了（改 REAR_NAMES_SWAPPED = true）");
                telemetry.addLine("  若向左平移 = 方向符号整体反了（对调 4 个 *_DIRECTION）");
            } else {
                telemetry.addLine("【当前测试】单轮手动：←/→ 选轮子，↑/↓ 按住让它转");
                telemetry.addLine("  预期：转的必须是你以为的那个物理轮子");
            }
            telemetry.addLine("-----------------------------------------------");
            for (int i = 0; i < 4; i++) {
                telemetry.addData((i == selected && mode == 0 ? "▶ " : "   ") + PHYSICAL_LABEL[i],
                        "名字=%-4s 端口=%-2s 功率=%+.2f",
                        configuredName[i], portLabel(motors[i]), power[i]);
            }
            telemetry.addLine("A=全轮前进  B=右平移  X=停止并回单轮模式");
            telemetry.update();
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0);
        }
    }

    /**
     * 读取电机在它所在 Hub 上的端口号，用于核对"名字 ↔ 端口"。
     * 端口号只暴露在 SDK 内部实现类上，所以用反射调用；读不到就返回 "?"，不影响运行。
     */
    private static String portLabel(DcMotor motor) {
        try {
            // 端口号只暴露在 SDK 内部实现类上，用反射调用可以让本文件在
            // Android Studio 与 OnBotJava 两种环境下都能编译；读不到就显示 "?"。
            Object port = motor.getClass().getMethod("getPortNumber").invoke(motor);
            return String.valueOf(port);
        } catch (Throwable ignored) {
            return "?";
        }
    }

    /**
     * 与 Test_dipan.java 相同的查找逻辑：必须用 hardwareMap 按"配置名"取设备。
     * 不能用 HardwareDevice.getDeviceName()（那返回的是设备型号，不是配置名）。
     * 找不到时用 getAllNames() 把配置里真实的电机名全部列出来。
     */
    private static DcMotor findMotor(HardwareMap hardwareMap, String position, String... wantedNames) {
        for (String wanted : wantedNames) {
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, wanted);
            if (motor != null) {
                return motor;
            }
        }
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            for (String configured : hardwareMap.getNamesOf(motor)) {
                for (String wanted : wantedNames) {
                    if (configured.trim().equalsIgnoreCase(wanted)) {
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

    /** 取电机在机器人配置里的名字，仅用于遥测显示 */
    private String configNameOf(DcMotor motor) {
        List<String> names = new ArrayList<>(hardwareMap.getNamesOf(motor));
        Collections.sort(names);
        return names.isEmpty() ? "?" : names.get(0);
    }
}
