package org.firstinspires.ftc.teamcode.Exercise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * ============================================================================
 * 底盘诊断：不用手柄也能测电机 + 同时监视手柄信号
 * ============================================================================
 *
 * 【为什么需要这个程序】
 *   当"车不听手柄"时，问题可能在三处完全不同的地方：
 *     (1) 手柄 / Driver Station 侧  —— 机器人根本没收到输入
 *     (2) 电机 / 接线 / 配置侧      —— 收到了输入但轮子不转
 *     (3) 你选的 OpMode 根本不是这份代码（比如 Hub 上跑着旧版本编译结果）
 *   这个程序把 (1) 和 (2) 分开：它的电机测试是**定时自动跑**的，不需要按任何键，
 *   所以即使手柄完全没信号，也能验证电机通路；同时屏幕实时显示手柄原始值来验证 (1)。
 *
 * 【用法】
 *   1. 把车架空（轮子离地）！本程序会自动让轮子转。
 *   2. Driver Station 里选 "Test Motor Auto" 并 START。
 *   3. 看遥测 + 听声音，对照下面 6 个阶段：
 *        阶段1  只转 左前 LF
 *        阶段2  只转 右前 RF
 *        阶段3  只转 左后 LB
 *        阶段4  只转 右后 RB
 *        阶段5  四个轮子同时前进（整车应向前）
 *        阶段6  右平移图案（整车应右移；若原地打转 = 后轮名字对应反了）
 *     每个阶段之间有 1.5 秒停顿，方便你确认"是不是只有那一个轮子在转"。
 *   4. 过程中随手动一下左摇杆：如果自动测试立刻中止并显示"检测到手柄输入"，
 *      说明手柄信号是通的（这是本程序故意留的安全中止）。
 *
 * 【怎么读结论】
 *   · 阶段1~6 轮子都按预期转  → 电机、配置名、端口、方向 全部正常
 *                              ⇒ 问题在 Driver Station / 手柄 侧
 *   · 轮子完全不转，但"手柄原始值"推摇杆时会变 → 问题在电机输出侧
 *                              （看下面表格里的 名字/端口/方向 是否和实车一致）
 *   · 轮子不转，摇杆数值也一直是 0.00 → 问题在 Driver Station / 手柄 侧
 *   · 连遥测都不出现 / OpMode 立刻结束 → 你跑的不是这份代码（Hub 上是旧编译结果）
 * ============================================================================
 */
@TeleOp(name = "Test Motor Auto (轮子会自动转)", group = "Linear OpMode")
public class Test_MotorAuto extends LinearOpMode {

    // ===== 与 Test_dipan.java 保持一致的设置 =====
    private static final boolean REAR_NAMES_SWAPPED = false;
    private static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction LEFT_BACK_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_BACK_DIRECTION = DcMotor.Direction.FORWARD;

    /** 自动测试功率：故意给得很小，架空时轮子转得慢、看得清 */
    private static final double TEST_POWER = 0.2;

    /** 超过这个值就认为"检测到手柄输入"，自动测试立刻中止 */
    private static final double INPUT_THRESHOLD = 0.3;

    private static final String[] LABEL = {"左前 LF", "右前 RF", "左后 LB", "右后 RB"};

    private final DcMotor[] motors = new DcMotor[4];
    private final String[] names = new String[4];
    private final String[] ports = new String[4];

    private boolean motorsReady = false;
    private String initError = null;

    private final ElapsedTime timer = new ElapsedTime();

    /** 记录"整个运行期间是否收到过任何手柄输入"，用于最后给结论 */
    private boolean everSawGamepadInput = false;
    private boolean interruptedByGamepad = false;

    @Override
    public void runOpMode() {

        // ---------------- Init：取电机（失败也不中止，方便仍然监视手柄）----------------
        try {
            motors[0] = findMotor(hardwareMap, LABEL[0] + " / 端口0", "lf", "LF");
            motors[1] = findMotor(hardwareMap, LABEL[1] + " / 端口1", "rf", "RF");
            if (REAR_NAMES_SWAPPED) {
                motors[2] = findMotor(hardwareMap, "左后 / 端口3", "rb", "RB");
                motors[3] = findMotor(hardwareMap, "右后 / 端口2", "lb", "LB");
            } else {
                motors[2] = findMotor(hardwareMap, "左后 LB / 端口3", "lb", "LB");
                motors[3] = findMotor(hardwareMap, "右后 RB / 端口2", "rb", "RB");
            }
            DcMotor.Direction[] directions = {
                    LEFT_FRONT_DIRECTION, RIGHT_FRONT_DIRECTION, LEFT_BACK_DIRECTION, RIGHT_BACK_DIRECTION
            };
            for (int i = 0; i < 4; i++) {
                motors[i].setDirection(directions[i]);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setPower(0);
                names[i] = configNameOf(motors[i]);
                ports[i] = portLabel(motors[i]);
            }
            motorsReady = true;
        } catch (IllegalStateException e) {
            // 电机没找齐也继续跑：至少让你看到手柄信号有没有进来
            initError = e.getMessage();
        }

        telemetry.addLine("==== 底盘 / 手柄 诊断 ====");
        if (motorsReady) {
            telemetry.addLine("电机会自动逐个转动，请把车架空！");
            for (int i = 0; i < 4; i++) {
                telemetry.addData(LABEL[i], "名字=%s 端口=%s 方向=%s",
                        names[i], ports[i], motors[i].getDirection());
            }
        } else {
            telemetry.addLine("电机没找齐（下面这句话说明原因），但仍会继续监视手柄：");
            telemetry.addLine(initError);
        }
        telemetry.update();

        waitForStart();
        timer.reset();

        // ---------------- Loop ----------------
        while (opModeIsActive()) {

            // 1) 读手柄原始值（不做任何死区处理，就是要看原始数据）
            double rawY = gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double rawRX = gamepad1.right_stick_x;
            double rawLY = gamepad1.left_stick_y;
            double rawRY = gamepad1.right_stick_y;

            boolean anyInput = Math.abs(rawY) > INPUT_THRESHOLD
                    || Math.abs(rawX) > INPUT_THRESHOLD
                    || Math.abs(rawRX) > INPUT_THRESHOLD
                    || Math.abs(rawLY) > INPUT_THRESHOLD
                    || Math.abs(rawRY) > INPUT_THRESHOLD
                    || gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y
                    || gamepad1.left_bumper || gamepad1.right_bumper
                    || gamepad1.dpad_up || gamepad1.dpad_down
                    || gamepad1.dpad_left || gamepad1.dpad_right
                    || gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1
                    || gamepad1.start || gamepad1.back;

            if (anyInput) {
                everSawGamepadInput = true;
                // 安全中止：手柄一有输入就停下自动测试，避免失控
                if (!interruptedByGamepad && timer.seconds() < 14.0) {
                    interruptedByGamepad = true;
                }
            }

            // 2) 自动测试时序（完全不需要手柄参与）
            double t = timer.seconds();
            double[] power = new double[4];
            String stage;

            if (interruptedByGamepad) {
                stage = "已被手柄输入中止（说明手柄信号是通的！）";
            } else if (t < 1.0) {
                power[0] = TEST_POWER;
                stage = "阶段 1/6：只转 左前 LF";
            } else if (t < 2.5) {
                stage = "阶段 1 结束，停顿中…";
            } else if (t < 3.5) {
                power[1] = TEST_POWER;
                stage = "阶段 2/6：只转 右前 RF";
            } else if (t < 5.0) {
                stage = "阶段 2 结束，停顿中…";
            } else if (t < 6.0) {
                power[2] = TEST_POWER;
                stage = "阶段 3/6：只转 左后 LB";
            } else if (t < 7.5) {
                stage = "阶段 3 结束，停顿中…";
            } else if (t < 8.5) {
                power[3] = TEST_POWER;
                stage = "阶段 4/6：只转 右后 RB";
            } else if (t < 10.0) {
                stage = "阶段 4 结束，停顿中…";
            } else if (t < 11.0) {
                for (int i = 0; i < 4; i++) {
                    power[i] = TEST_POWER;
                }
                stage = "阶段 5/6：四轮同时前进（整车应向前）";
            } else if (t < 12.5) {
                stage = "阶段 5 结束，停顿中…";
            } else if (t < 13.5) {
                // 麦轮右平移图案：左前 + / 右前 - / 左后 - / 右后 +
                power[0] = TEST_POWER;
                power[1] = -TEST_POWER;
                power[2] = -TEST_POWER;
                power[3] = TEST_POWER;
                stage = "阶段 6/6：右平移图案（整车应右移；若原地打转=后轮名字反了）";
            } else {
                stage = "自动测试已结束（全部停止）";
            }

            // 3) 输出
            if (motorsReady) {
                for (int i = 0; i < 4; i++) {
                    motors[i].setPower(power[i]);
                }
            }

            // 4) 遥测
            telemetry.addLine("==== " + stage + " ====");
            telemetry.addData("时间", "%.1f s", t);
            telemetry.addLine("---- 手柄原始值（推摇杆看这里变不变）----");
            telemetry.addData("左摇杆", "x=%.3f  y=%.3f  (推上 y=-1)", rawX, rawY);
            telemetry.addData("右摇杆", "x=%.3f  y=%.3f", rawRX, rawRY);
            telemetry.addData("按键", "A=%b B=%b X=%b Y=%b LB=%b RB=%b dpad=%b/%b/%b/%b",
                    gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y,
                    gamepad1.left_bumper, gamepad1.right_bumper,
                    gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
            telemetry.addData("扳机", "LT=%.2f  RT=%.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("★ 运行期间是否收到过手柄输入", everSawGamepadInput ? "是（手柄/DS 没问题）" : "否（问题在 Driver Station 或手柄）");
            telemetry.addLine("---- 电机 ----");
            if (motorsReady) {
                for (int i = 0; i < 4; i++) {
                    telemetry.addData(LABEL[i], "名字=%-4s 端口=%-2s 方向=%-7s 功率=%+.2f",
                            names[i], ports[i], motors[i].getDirection(), power[i]);
                }
            } else {
                telemetry.addLine(initError);
            }
            telemetry.update();
        }

        if (motorsReady) {
            for (int i = 0; i < 4; i++) {
                motors[i].setPower(0);
            }
        }
    }

    /**
     * 读取电机在它所在 Hub 上的端口号。端口号只在 SDK 内部实现类上公开，
     * 因此用反射调用；读不到就返回 "?"，不影响运行。
     */
    private static String portLabel(DcMotor motor) {
        try {
            Object port = motor.getClass().getMethod("getPortNumber").invoke(motor);
            return String.valueOf(port);
        } catch (Throwable ignored) {
            return "?";
        }
    }

    /**
     * 按"物理位置"找电机。必须用 hardwareMap 的"配置名"来取：
     * tryGet / getNamesOf / getAllNames 都是按配置名工作的；
     * 不能用 HardwareDevice.getDeviceName()（SDK 文档明确说它返回的是设备型号名，不是配置名）。
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

    /** 取电机在机器人配置里的名字，仅用于显示 */
    private String configNameOf(DcMotor motor) {
        List<String> list = new ArrayList<>(hardwareMap.getNamesOf(motor));
        Collections.sort(list);
        return list.isEmpty() ? "?" : list.get(0);
    }
}
