package org.firstinspires.ftc.teamcode.Exercise;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * ============================================================================
 * 练习用底盘封装：4 个麦轮电机 + goBILDA Pinpoint 里程计 + 走直线到目标点
 * ============================================================================
 *
 * 【坐标系约定】（来自 Pinpoint 驱动源码的 javadoc，不是我们自定义的）
 *     +X = 车头正前方        （前进时 X 增大）
 *     +Y = 车的左侧          （左移时 Y 增大）
 *     航向 heading = 逆时针为正，单位弧度
 *   所以"角度"的含义是：0° = 前，90° = 左，180° = 后，270° = 右。
 *
 *   如果开 OpMode 时调用了 resetPose()，那么"当前车头方向"就是 0°、
 *   当前位置就是 (0, 0)，之后所有坐标都相对这个起点。
 *
 * 【麦轮公式】（X 型布局，与 Test_dipan 一致）
 *     左前 = axial + lateral + yaw
 *     右前 = axial - lateral - yaw
 *     左后 = axial - lateral + yaw
 *     右后 = axial + lateral - yaw
 *   其中 axial = 前进（前为正）、lateral = 横移（右为正）、yaw = 自转（右为正）。
 *
 * 【走直线的控制思路】（教学重点，别只抄公式，理解为什么）
 *   1. 里程计每圈刷新一次，得到当前 (x, y, heading)；
 *   2. 算出"还差多少"到目标点：(errX, errY)，以及剩余距离；
 *   3. 剩余距离越远给越大功率，快到目标时线性减速，避免冲过头；
 *   4. 把"场坐标下的速度"用航向角旋转成"车体坐标下的速度"（车头不一定朝前）；
 *   5. 同时用一个 P 控制器把航向拉回起始航向，防止走歪（横移时特别容易歪）。
 * ============================================================================
 */
public class PracticeChassis {

    // =========================================================================
    // 1. 需要按实车确认的配置
    // =========================================================================

    /** 后轮"名字 ↔ 物理位置"是否互换：false = 左后叫 lb(端口3)、右后叫 rb(端口2) */
    public static final boolean REAR_NAMES_SWAPPED = false;

    /** 电机方向：麦轮左右镜像安装，左侧整体反转（与 RobotHardwareV3 一致） */
    public static final DcMotor.Direction DIR_LEFT_FRONT = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIR_RIGHT_FRONT = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIR_LEFT_BACK = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIR_RIGHT_BACK = DcMotor.Direction.FORWARD;

    /** Pinpoint 在 Driver Station 机器人配置里的名字 */
    public static final String PINPOINT_NAME = "odo";

    /**
     * Pinpoint 两个测距轮（pod）相对"车体旋转中心"的偏移，单位 mm。
     * 定义（SDK 驱动 javadoc 原文）：
     *    POD_X_OFFSET_MM：X（前进方向）那个 pod，相对车心的左右位置 —— 在车心左边为正，右边为负
     *    POD_Y_OFFSET_MM：Y（横移方向）那个 pod，相对车心的前后位置 —— 在车心前方为正，后方为负
     * 这两个值必须自己拿尺子量。填错的最大后果是"一边走一边转"时定位漂移；
     * 纯直线运动受影响很小，所以练习阶段可以先不动，但对比赛自动很重要。
     */
    public static final double POD_X_OFFSET_MM = -74.0;
    public static final double POD_Y_OFFSET_MM = -136.0;

    /**
     * 编码器方向约定（SDK javadoc）：
     *    X 编码器应在"车往前走"时读数增大
     *    Y 编码器应在"车往左移"时读数增大
     * 若自检时方向相反，把对应的改成 EncoderDirection.REVERSED。
     */
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // =========================================================================
    // 2. 运动控制参数（都能调，调之前先看注释）
    // =========================================================================

    /** 认为"到位"的距离误差（mm）。越小越准，但太小会在目标点附近来回抖 */
    public static final double POSITION_TOLERANCE_MM = 15.0;

    /** 进入这个距离（mm）之后开始线性减速 */
    public static final double SLOWDOWN_DISTANCE_MM = 250.0;

    /** 最大输出功率（0~1）。练手先别给满，0.5 大约够走 1 米 */
    public static final double MAX_POWER = 0.5;

    /** 减速到最小也保留这个比例的功率，否则摩擦力会让车在终点前停住 */
    public static final double MIN_POWER_FRACTION = 0.15;

    /** 航向保持的 P 增益与输出上限 */
    public static final double HEADING_KP = 0.6;
    public static final double HEADING_MAX_OUTPUT = 0.25;

    /** 航向误差死区（弧度）：小于它就不修正，避免原地小幅抖动 */
    public static final double HEADING_DEADBAND_RAD = Math.toRadians(1.0);

    /** 单次移动超时（毫秒）：超时就停下并报错，避免卡在循环里顶着墙推 */
    public static final long MOVE_TIMEOUT_MS = 6000;

    // =========================================================================
    // 3. 内部状态
    // =========================================================================

    private final DcMotor[] motors = new DcMotor[4];
    private final String[] motorNames = new String[4];
    private final String[] motorPorts = new String[4];

    private GoBildaPinpointDriver pinpoint;

    private double poseXmm;
    private double poseYmm;
    private double poseHeadingRad;

    private boolean moving = false;
    private double targetXmm;
    private double targetYmm;
    private double targetHeadingRad;
    private double startXmm;
    private double startYmm;
    private double moveDistanceMm;
    private String moveResult = "";
    private final ElapsedTime moveTimer = new ElapsedTime();

    /** 最近一次移动的实际结果，用于遥测显示"实际走了多少、差多少" */
    private double lastDisplacementMm;
    private double lastErrorMm;

    // =========================================================================
    // 4. 初始化
    // =========================================================================

    /**
     * 取硬件并配置。失败时抛 IllegalStateException，消息里直接写清原因和可选值。
     */
    public void init(HardwareMap hardwareMap) {
        // ---- 4 个底盘电机（按物理位置取，名字大小写都兼容）----
        motors[0] = findMotor(hardwareMap, "左前 LF / 端口0", "lf", "LF");
        motors[1] = findMotor(hardwareMap, "右前 RF / 端口1", "rf", "RF");
        if (REAR_NAMES_SWAPPED) {
            motors[2] = findMotor(hardwareMap, "左后 / 端口3", "rb", "RB");
            motors[3] = findMotor(hardwareMap, "右后 / 端口2", "lb", "LB");
        } else {
            motors[2] = findMotor(hardwareMap, "左后 LB / 端口3", "lb", "LB");
            motors[3] = findMotor(hardwareMap, "右后 RB / 端口2", "rb", "RB");
        }
        DcMotor.Direction[] directions = {
                DIR_LEFT_FRONT, DIR_RIGHT_FRONT, DIR_LEFT_BACK, DIR_RIGHT_BACK
        };
        for (int i = 0; i < 4; i++) {
            motors[i].setDirection(directions[i]);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setPower(0);
            motorNames[i] = configNameOf(hardwareMap, motors[i]);
            motorPorts[i] = portLabel(motors[i]);
        }

        // ---- Pinpoint 里程计 ----
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        } catch (RuntimeException e) {
            // hardwareMap.get 找不到设备时抛的是 IllegalArgumentException，
            // 这里统一换成带排查信息的 IllegalStateException，让 OpMode 能把原因显示在遥测上。
            throw new IllegalStateException(
                    "找不到 Pinpoint 里程计：" + e.getMessage()
                            + "。（要求：机器人配置里有名为 \"" + PINPOINT_NAME
                            + "\" 的设备，且类型选的是 GoBildaPinpointDriver）"
                            + " 当前配置里的 Pinpoint：" + pinpointNames(hardwareMap), e);
        }
        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);
        pinpoint.resetPosAndIMU();   // 位置与航向归零（要求车静止）
        update();                    // 读一次，避免后续读到旧数据
    }

    /**
     * 把当前位置和航向清零：让"现在车头朝的方向"成为 0°，"现在的位置"成为 (0,0)。
     * 这样操作手不用关心车停在哪，看着手柄走就行。
     */
    public void resetPose() {
        ensurePinpoint();
        pinpoint.setPosX(0, DistanceUnit.MM);
        pinpoint.setPosY(0, DistanceUnit.MM);
        pinpoint.setHeading(0, AngleUnit.RADIANS);
        pinpoint.update();
        update();
    }

    // =========================================================================
    // 5. 里程计读取
    // =========================================================================

    /** 每圈必须调用：把 Pinpoint 的最新数据读进缓存 */
    public void update() {
        ensurePinpoint();
        pinpoint.update();
        poseXmm = pinpoint.getPosX(DistanceUnit.MM);
        poseYmm = pinpoint.getPosY(DistanceUnit.MM);
        poseHeadingRad = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public double getXmm() {
        return poseXmm;
    }

    public double getYmm() {
        return poseYmm;
    }

    public double getHeadingRad() {
        return poseHeadingRad;
    }

    public double getHeadingDeg() {
        return Math.toDegrees(poseHeadingRad);
    }

    // =========================================================================
    // 6. 直接驱动（车体坐标系）
    // =========================================================================

    /**
     * 按车体坐标系驱动。
     *
     * @param axial        前进为正（车头方向）
     * @param lateralRight 右移为正
     * @param yawRight     顺时针（右转）为正
     */
    public void driveRobotFrame(double axial, double lateralRight, double yawRight) {
        double lf = axial + lateralRight + yawRight;
        double rf = axial - lateralRight - yawRight;
        double lb = axial - lateralRight + yawRight;
        double rb = axial + lateralRight - yawRight;

        // 归一化：只等比缩小，不改变运动方向
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        motors[0].setPower(lf);
        motors[1].setPower(rf);
        motors[2].setPower(lb);
        motors[3].setPower(rb);
    }

    /** 四个轮子全部停转 */
    public void stop() {
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
        moving = false;
    }

    // =========================================================================
    // 7. 走 1 米的核心：走到"起点的某个方向 + 某个距离"的目标点
    // =========================================================================

    /**
     * 从当前位置出发，沿 directionDeg 方向走 distanceMm 毫米。
     *
     * @param directionDeg 0=前，90=左，180=后，270=右（逆时针为正）
     * @param distanceMm   走多远（毫米）
     */
    public void startMove(double directionDeg, double distanceMm) {
        ensurePinpoint();
        update();

        double rad = Math.toRadians(directionDeg);
        startXmm = poseXmm;
        startYmm = poseYmm;
        targetXmm = poseXmm + Math.cos(rad) * distanceMm;
        targetYmm = poseYmm + Math.sin(rad) * distanceMm;
        targetHeadingRad = poseHeadingRad;    // 保持出发时的车头方向
        moveDistanceMm = distanceMm;
        moveResult = "正在移动…";
        lastDisplacementMm = 0;
        lastErrorMm = 0;
        moveTimer.reset();
        moving = true;
    }

    /**
     * 正在移动时必须每圈调用：计算并输出这一圈的四个轮子功率。
     * 没在移动时它什么也不做。
     */
    public void updateMove() {
        if (!moving) {
            return;
        }

        // 1) 还差多少
        double errX = targetXmm - poseXmm;
        double errY = targetYmm - poseYmm;
        double remaining = Math.hypot(errX, errY);

        // 2) 到达 / 超时判定
        if (remaining <= POSITION_TOLERANCE_MM) {
            finishMove("到位（剩余误差 " + String.format("%.1f", remaining) + " mm）");
            return;
        }
        if (moveTimer.milliseconds() > MOVE_TIMEOUT_MS) {
            finishMove("超时停止（剩余 " + String.format("%.0f", remaining) + " mm）");
            return;
        }

        // 3) 功率大小：远时最大，近时线性减速（但保留下限，否则摩擦会让它提前停）
        double magnitude = MAX_POWER;
        if (remaining < SLOWDOWN_DISTANCE_MM) {
            magnitude = MAX_POWER * Math.max(MIN_POWER_FRACTION, remaining / SLOWDOWN_DISTANCE_MM);
        }

        // 4) 场坐标下的速度方向（指向目标点的单位向量）
        double vxField = errX / remaining * magnitude;        // +X = 前
        double vyLeftField = errY / remaining * magnitude;    // +Y = 左

        // 5) 场坐标 → 车体坐标（按当前航向旋转 -heading）
        double cos = Math.cos(poseHeadingRad);
        double sin = Math.sin(poseHeadingRad);
        double axial = cos * vxField + sin * vyLeftField;
        double lateralRight = sin * vxField - cos * vyLeftField;

        // 6) 航向保持：航向是逆时针为正，而 yawRight 是顺时针为正，所以取负号
        double headingError = normalizeAngle(targetHeadingRad - poseHeadingRad);
        double yawRight = 0.0;
        if (Math.abs(headingError) > HEADING_DEADBAND_RAD) {
            yawRight = clamp(-HEADING_KP * headingError, HEADING_MAX_OUTPUT);
        }

        // 7) 输出
        driveRobotFrame(axial, lateralRight, yawRight);
    }

    private void finishMove(String reason) {
        // 实际位移：从起点到现在走过的直线距离（理想值 = moveDistanceMm）
        lastDisplacementMm = Math.hypot(poseXmm - startXmm, poseYmm - startYmm);
        // 终点误差：现在离目标点还差多少
        lastErrorMm = Math.hypot(targetXmm - poseXmm, targetYmm - poseYmm);
        stop();
        moving = false;
        moveResult = reason;
    }

    /** 立刻中止当前移动（紧急停止 / 操作手打断） */
    public void abortMove(String reason) {
        if (!moving) {
            return;
        }
        lastDisplacementMm = Math.hypot(poseXmm - startXmm, poseYmm - startYmm);
        lastErrorMm = Math.hypot(targetXmm - poseXmm, targetYmm - poseYmm);
        stop();
        moving = false;
        moveResult = "已中止：" + reason;
    }

    public boolean isMoving() {
        return moving;
    }

    public String getMoveResult() {
        return moveResult;
    }

    /** 剩余距离（mm） */
    public double getRemainingMm() {
        return Math.hypot(targetXmm - poseXmm, targetYmm - poseYmm);
    }

    /** 最近一次移动实际走过的距离（mm） */
    public double getLastDisplacementMm() {
        return lastDisplacementMm;
    }

    /** 最近一次移动结束时离目标点还差多少（mm） */
    public double getLastErrorMm() {
        return lastErrorMm;
    }

    // =========================================================================
    // 8. 遥测用的信息
    // =========================================================================

    public String describeMotors() {
        StringBuilder sb = new StringBuilder();
        String[] label = {"左前", "右前", "左后", "右后"};
        for (int i = 0; i < 4; i++) {
            if (i > 0) {
                sb.append("  ");
            }
            sb.append(label[i]).append("=").append(motorNames[i]).append("(端口").append(motorPorts[i]).append(")");
        }
        return sb.toString();
    }

    public boolean isReady() {
        return motors[0] != null && motors[3] != null && pinpoint != null;
    }

    // =========================================================================
    // 9. 工具方法
    // =========================================================================

    private void ensurePinpoint() {
        if (pinpoint == null) {
            throw new IllegalStateException("Pinpoint 里程计没有初始化（检查机器人配置里是否叫 \"" + PINPOINT_NAME + "\"）");
        }
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    /** 把角度归一化到 (-π, π] */
    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians <= -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    /**
     * 按物理位置找电机。必须用 hardwareMap 的"配置名"：
     * tryGet / getNamesOf / getAllNames 都是按配置名工作的；
     * 不能用 getDeviceName()（SDK 文档说它返回设备型号名，不是配置名）。
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

    private static String configNameOf(HardwareMap hardwareMap, DcMotor motor) {
        List<String> names = new ArrayList<>(hardwareMap.getNamesOf(motor));
        Collections.sort(names);
        return names.isEmpty() ? "?" : names.get(0);
    }

    /** 列出配置里所有 Pinpoint 的配置名，用于报错提示 */
    private static String pinpointNames(HardwareMap hardwareMap) {
        try {
            return hardwareMap.getAllNames(GoBildaPinpointDriver.class).toString();
        } catch (Throwable t) {
            return "（读取失败）";
        }
    }

    /** 读电机在其所在 Hub 上的端口号（只用于显示，靠反射调用，读不到返回 "?"） */
    private static String portLabel(DcMotor motor) {
        try {
            Object port = motor.getClass().getMethod("getPortNumber").invoke(motor);
            return String.valueOf(port);
        } catch (Throwable ignored) {
            return "?";
        }
    }
}
