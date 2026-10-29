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
 * 32477 底盘手柄控制 —— 麦轮 X 型布局（车体坐标系 Robot Centric）
 * ========================================  ====================================
 *
 * 【功能】
 *   左摇杆 上/下/左/右 → 车做同方向运动（上=前进 下=后退 左=左平移 右=右平移）
 *   右摇杆 左/右        → 原地旋转（右 = 顺时针 / 右转）
 *   按住 left_bumper    → 慢速模式（功率 × 0.35），方便精确走位
 *
 * ----------------------------------------------------------------------------
 * 【硬件映射】物理位置 → 配置名称 → 电机端口
 * ----------------------------------------------------------------------------
 *   物理位置        配置名称        电机端口        方向设置
 *   ---------------------------------------------------------------
 *   左前 LF         "lf" / "LF"     0              REVERSE
 *   右前 RF         "rf" / "RF"     1              FORWARD
 *   左后 LB         "lb" / "LB"     3              REVERSE
 *   右后 RB         "rb" / "RB"     2              FORWARD
 *
 * 【为什么名字写两遍（"lf" 和 "LF"）】
 *   FTC SDK 的硬件名是"区分大小写"的（HardwareMap 源码里明确写着 case sensitive，
 *   底层就是 HashMap + trim），所以配置里写 "lf" 时用 "LF" 去取会直接抛
 *   IllegalArgumentException 让 OpMode 崩在 Init。本项目 RobotHardwareV3.java 和
 *   pedroPathing/Constants.java 用的都是小写 "lf/rf/lb/rb"，而旧版 Test_dipan 用的是
 *   大写 "LF/RF/LB/RB"——两者必有一个是错的。本程序对两种写法都做匹配，彻底避免这个问题；
 *   如果两种都没找到，会把配置里"所有电机的真实名字"打印到错误信息里，一眼看出问题。
 *
 * 【关于后轮名字】
 *   旧代码里把"左后"取名为 RB、"右后"取名为 LB（名字与物理位置相反）。本程序按物理位置
 *   定义变量（frontLeft / frontRight / backLeft / backRight），名字只作为"取硬件"的钥匙：
 *      REAR_NAMES_SWAPPED = false（默认）：左后 = "lb"（端口 3），右后 = "rb"（端口 2）
 *      REAR_NAMES_SWAPPED = true         ：左后 = "rb"（端口 3），右后 = "lb"（端口 2）
 *   默认值与本项目 RobotHardwareV3.java、Constants.java 保持一致。
 *   拿不准就用同目录下的 Test_WheelCheck 逐个轮子转一下，2 分钟就能确认。
 *
 * 【方向为什么是"左 REVERSE、右 FORWARD"】
 *   麦轮左右两侧是镜像安装的，同样的正电压会让左右轮反向转，所以必须把一侧在软件里翻过来。
 *   本项目另外两个文件（RobotHardwareV3、Constants）都用"左侧 REVERSE / 右侧 FORWARD"，
 *   本程序沿用同一约定，保证手动与 Pedro 自动一致。
 *   如果实测某个轮子转反了，改下面 4 个 *_DIRECTION 常量即可，不用动公式。
 *
 * ----------------------------------------------------------------------------
 * 【麦轮 X 型运动学】标准公式（也是 FTC 官方 BasicOmniOpMode 的写法）
 * ----------------------------------------------------------------------------
 *   axial   = -left_stick_y   摇杆推上 = +1（SDK 里推上本来是负值，所以要取反）
 *   lateral =  left_stick_x   摇杆推右 = +1
 *   yaw     = right_stick_x   摇杆推右 = 右转
 *
 *   左前 = axial + lateral + yaw
 *   右前 = axial - lateral - yaw
 *   左后 = axial - lateral + yaw
 *   右后 = axial + lateral - yaw
 *
 * 【自检方法（把车架空，看轮子转向）】
 *   只推"左摇杆向前" → 四个轮子都向前转
 *   只推"左摇杆向右" → 左前 + 右后 向前转，右前 + 左后 向后转
 *   只推"右摇杆向右" → 左侧两轮向前、右侧两轮向后（车头向右转）
 * ============================================================================
 */
@TeleOp(name = "Test Dipan (Robot Centric)", group = "Linear OpMode")
public class Test_dipan extends LinearOpMode {

    // ========================================================================
    // 1. 可调参数
    // ========================================================================

    /** 摇杆死区：小于这个值的输入当作 0，避免手柄回中不准导致车自己爬 */
    private static final double DEADZONE = 0.05;

    /** true = 平方曲线（小推杆更细腻、大推杆到满功率），新手/走位推荐开着 */
    private static final boolean SQUARE_INPUT = true;

    /** 慢速模式倍率（按住 left_bumper 生效） */
    private static final double SLOW_MODE_SCALE = 0.35;

    /**
     * 后轮"名字 ↔ 物理位置"是否互换，见文件头说明。
     * false（默认）：左后 = "lb"（端口 3），右后 = "rb"（端口 2）
     * true         ：左后 = "rb"（端口 3），右后 = "lb"（端口 2）
     */
    private static final boolean REAR_NAMES_SWAPPED = false;

    // 电机方向：麦轮镜像安装 → 左侧整体反转、右侧保持正向
    private static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction LEFT_BACK_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_BACK_DIRECTION = DcMotor.Direction.FORWARD;

    /** 零功率行为：BRAKE = 松开摇杆立刻刹住（竞争赛推荐），FLOAT = 滑行 */
    private static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // ========================================================================
    // 2. 硬件对象（变量名 = 物理位置，而不是配置里的名字）
    // ========================================================================

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    /** 实际匹配到的"配置名"，只用于遥测显示（不是 getDeviceName()，那返回的是设备型号） */
    private String frontLeftName;
    private String frontRightName;
    private String backLeftName;
    private String backRightName;

    // ========================================================================
    // 3. OpMode 主体
    // ========================================================================

    @Override
    public void runOpMode() {

        // ---------------- Init：取硬件 + 设方向 ----------------
        // 取硬件包在 try 里：万一配置名对不上，把错误直接显示在 Driver Station 遥测上，
        // 而不是让 OpMode 崩掉、只留一行看不懂的日志。
        try {
            frontLeft = findMotor(hardwareMap, "左前 (LF/端口0)", "lf", "LF");
            frontRight = findMotor(hardwareMap, "右前 (RF/端口1)", "rf", "RF");
            if (REAR_NAMES_SWAPPED) {
                backLeft = findMotor(hardwareMap, "左后 (端口3)", "rb", "RB");
                backRight = findMotor(hardwareMap, "右后 (端口2)", "lb", "LB");
            } else {
                backLeft = findMotor(hardwareMap, "左后 (LB/端口3)", "lb", "LB");
                backRight = findMotor(hardwareMap, "右后 (RB/端口2)", "rb", "RB");
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

        telemetry.addLine("==== 底盘初始化完成（请核对下面 4 行名字）====");
        telemetry.addData("左前 LF", "%s  dir=%s", frontLeftName, frontLeft.getDirection());
        telemetry.addData("右前 RF", "%s  dir=%s", frontRightName, frontRight.getDirection());
        telemetry.addData("左后 LB", "%s  dir=%s", backLeftName, backLeft.getDirection());
        telemetry.addData("右后 RB", "%s  dir=%s", backRightName, backRight.getDirection());
        telemetry.addLine("--------------------------------------------");
        telemetry.addLine("左摇杆 = 平移（上=前进/左=左移）");
        telemetry.addLine("右摇杆 = 原地旋转");
        telemetry.addLine("按住 left bumper = 慢速模式");
        telemetry.update();

        waitForStart();

        // ---------------- Loop：读手柄 → 解算 → 输出 ----------------
        while (opModeIsActive()) {

            // 1) 采集左手柄（推上 = +1 前进，推右 = +1 右移）
            double axial = curve(-gamepad1.left_stick_y);
            double lateral = curve(gamepad1.left_stick_x);

            // 2) 右摇杆原地旋转（推右 = 右转）
            double yaw = curve(gamepad1.right_stick_x);

            // 3) 慢速模式
            boolean slowMode = gamepad1.left_bumper;
            if (slowMode) {
                axial *= SLOW_MODE_SCALE;
                lateral *= SLOW_MODE_SCALE;
                yaw *= SLOW_MODE_SCALE;
            }

            // 4) 麦轮 X 型混合：按"物理位置"套标准公式
            double lfPower = axial + lateral + yaw;   // 左前
            double rfPower = axial - lateral - yaw;   // 右前
            double lbPower = axial - lateral + yaw;   // 左后
            double rbPower = axial + lateral - yaw;   // 右后

            // 5) 归一化：保证"方向不变、比例不变"，只把最大值压到 1.0 以内
            double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower)));
            if (max > 1.0) {
                lfPower /= max;
                rfPower /= max;
                lbPower /= max;
                rbPower /= max;
            }

            // 6) 输出到电机
            frontLeft.setPower(lfPower);
            frontRight.setPower(rfPower);
            backLeft.setPower(lbPower);
            backRight.setPower(rbPower);

            // 7) 遥测
            telemetry.addData("摇杆输入", "前进 %.2f | 平移 %.2f | 旋转 %.2f %s",
                    axial, lateral, yaw, slowMode ? "[慢速]" : "");
            telemetry.addData("左前 " + frontLeftName, "%.2f", lfPower);
            telemetry.addData("右前 " + frontRightName, "%.2f", rfPower);
            telemetry.addData("左后 " + backLeftName, "%.2f", lbPower);
            telemetry.addData("右后 " + backRightName, "%.2f", rbPower);
            telemetry.update();
        }

        // 退出时把四个轮子停掉（保险，正常停止时 SDK 也会停）
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // ========================================================================
    // 4. 工具方法
    // ========================================================================

    /**
     * 死区 + 曲线处理：
     * 先把死区外的输入线性重映射到 0~1（避免刚出死区就"跳"到 0.05），
     * 再可选地做平方，让小幅推杆更容易精确控制。
     */
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

    /** 设置方向 + 零功率行为 + 运行模式（RUN_WITHOUT_ENCODER：直接给电压，手动控制最可预测） */
    private void configureMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    /**
     * 按"物理位置"找电机。
     *
     * 【重要】取硬件必须用"配置名"：hardwareMap.tryGet / getNamesOf / getAllNames 都是按配置名工作的。
     * 绝对不能用 HardwareDevice.getDeviceName() —— SDK 文档里明确写了它返回的是"设备型号名"
     * （例如 LynxMotor），"与用户在机器人配置里起的名字没有任何关系"。用它会取错/取不到。
     */
    private static DcMotor findMotor(HardwareMap hardwareMap, String position, String... wantedNames) {
        // 1) 先用配置名精确取（大小写敏感，最直接）
        for (String wanted : wantedNames) {
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, wanted);
            if (motor != null) {
                return motor;
            }
        }
        // 2) 再做一次大小写不敏感匹配，容忍配置里写成 lf / LF / Lf
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            for (String configuredName : hardwareMap.getNamesOf(motor)) {
                for (String wanted : wantedNames) {
                    if (configuredName.trim().equalsIgnoreCase(wanted)) {
                        return motor;
                    }
                }
            }
        }
        // 3) 都找不到：把配置里真实的电机名字列出来，一眼看出差在哪
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
