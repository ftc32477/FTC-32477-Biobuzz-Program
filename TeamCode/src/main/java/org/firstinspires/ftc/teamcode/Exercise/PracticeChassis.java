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
 * 练习用底盘封装：4 个麦轮电机 +「Pinpoint 的两个测距轮原始编码器」里程计
 * ============================================================================
 *
 * 【本版本的位置来源（重点）】
 *   位置 (X, Y) 只来自接在 Pinpoint 上的两个测距轮（X pod / Y pod）的“原始计数”：
 *       pinpoint.getEncoderX()  → X（前进方向）测距轮的累计 ticks
 *       pinpoint.getEncoderY()  → Y（横移方向）测距轮的累计 ticks
 *   代码里完全不使用 pinpoint.getPosX() / getPosY()，也就是说
 *   Pinpoint 内部融合出来的位置我们一概不读，X/Y 由我们自己按 ticks 积分。
 *
 *   航向（heading）仍然用 Pinpoint 的 IMU：
 *   两个测距轮只能测出“沿 X 走了多少、沿 Y 走了多少”，
 *   在数学上无法同时解出“车转了多少度”（2 个测量值解 3 个未知数），
 *   所以航向必须靠陀螺仪。这不属于“位置信息”。
 *
 * 【坐标系约定】
 *     +X = 车头正前方     （前进时 X 增大）
 *     +Y = 车的左侧       （左移时 Y 增大）
 *     heading = 逆时针为正，单位弧度
 *   所以角度含义是：0° = 前，90° = 左，180° = 后，270° = 右。
 *   按 START 时会 resetPose()：当前位置 = (0,0)，当前车头 = 0°。
 *
 * 【麦轮公式】（与实车验证过的 Test_dipan 完全一致，不要换成教科书公式）
 *     左前 LF = -axial - lateralRight + yawRight
 *     右前 RF = -axial + lateralRight - yawRight
 *     左后 LB = +axial - lateralRight - yawRight
 *     右后 RB = +axial + lateralRight + yawRight
 *   这台车的电机接线/安装方向比较特殊（Test_dipan 里 left_stick_y 不取反、
 *   left_stick_x 取反、后轮整体取反），上面这组符号就是照着能跑的那版抄下来的。
 *   如果以后换了电机线或底盘，先让 Test_dipan 手动跑对，再回来同步这 4 行。
 *
 * 【从两个测距轮算位置：为什么要补偿偏移】
 *   测距轮不在车体旋转中心，所以车一转，轮子也会跟着“多走一段”：
 *     X pod 装在中心左边 y_pod 处 → 它读到的是      dX_pod = dX - dθ * y_pod
 *     Y pod 装在中心前方 x_pod 处 → 它读到的是      dY_pod = dY + dθ * x_pod
 *   反解回来：
 *     dX = dX_pod + dθ * X_POD_OFFSET_MM
 *     dY = dY_pod - dθ * Y_POD_OFFSET_MM
 *   再把车体位移按航向旋转到场地坐标（用这一圈的中点航向更准）：
 *     dX_field = cos(h)*dX - sin(h)*dY
 *     dY_field = sin(h)*dX + cos(h)*dY
 *   逐圈累加，就得到 1 米、2 米……的位置。
 *
 * 【需要按实车确认的配置】
 *   1) PINPOINT_NAME：机器人配置里 Pinpoint 的名字（默认 "odo"）
 *   2) X_POD_OFFSET_MM / Y_POD_OFFSET_MM：
 *        X pod 相对车心的“左右”偏移（在左边为正）——默认 -74（在右边 74mm）
 *        Y pod 相对车心的“前后”偏移（在前方为正）——默认 -136（在后方 136mm）
 *      拿尺子量一下；这两个值填错只会让“边转边走”时位置漂，直线影响很小。
 *   3) POD_TICKS_PER_MM：goBILDA 4-Bar Pod = 19.89436789；Swingarm Pod = 13.26291192
 *   4) X_POD_SIGN / Y_POD_SIGN：编码器计数方向。正常 +1。
 *      自检：向前推车，X 轮行程应为正；向左推车，Y 轮行程应为正。反了就把对应项改成 -1。
 *   5) DISTANCE_CALIBRATION：实际走 1000mm 只走了 950mm，就填 1000/950。
 *
 * 【走直线的控制思路】
 *   1. 每圈读两个测距轮 ticks + IMU 航向，积分出 (x, y)；
 *   2. 算“还差多少”到目标点；
 *   3. 剩余越远功率越大，快到目标线性减速；
 *   4. 把场坐标速度按航向旋转成车体坐标；
 *   5. 用 P 控制器把航向拉回出发时的方向；
 *   6. 进入 10mm 误差或 6 秒超时就停。
 * ============================================================================
 */
public class PracticeChassis {

    // =========================================================================
    // 1. 需要按实车确认的配置
    // =========================================================================

    /** 后轮"名字 ↔ 物理位置"是否互换：false = 左后叫 lb(端口3)、右后叫 rb(端口2) */
    public static final boolean REAR_NAMES_SWAPPED = false;

    /**
     * 电机方向。
     * 左侧整体反转是麦轮镜像安装的标准做法；
     * 右前改成 REVERSE：按当前混控（推上 axial=-1）右前拿到的功率是负的，
     * 原来的 FORWARD 会让右前和左前转向相反（实车表现为右前"反着转"）。
     * 后轮的 90° 锥齿轮换向由 lb/rb 公式里的整体负号补偿，后轮方向常量先不要动。
     */
    public static final DcMotor.Direction DIR_LEFT_FRONT = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIR_RIGHT_FRONT = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIR_LEFT_BACK = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIR_RIGHT_BACK = DcMotor.Direction.FORWARD;

    /** Pinpoint 在 Driver Station 机器人配置里的名字 */
    public static final String PINPOINT_NAME = "odo";

    /** 两个测距轮相对车心的偏移（mm），定义见文件头 */
    public static final double X_POD_OFFSET_MM = -74.0;
    public static final double Y_POD_OFFSET_MM = -136.0;

    /** 测距轮每毫米的编码器计数：4-Bar Pod = 19.89436789，Swingarm Pod = 13.26291192 */
    public static final double POD_TICKS_PER_MM = 19.89436789;

    /** 距离校准系数：实际距离 / 编码器算出的距离 */
    public static final double DISTANCE_CALIBRATION = 1.0;

    /** 每个测距轮的计数方向，正常 +1；自检反了就改 -1 */
    public static final double X_POD_SIGN = -1.0;
    public static final double Y_POD_SIGN = +1.0;

    /**
     * 测距轮编码器方向（只影响 Pinpoint 内部，不影响我们读到的原始 ticks）。
     * 与队伍之前的配置保持一致：X 正转，Y 反转。
     */
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /** 一个 tick 对应多少毫米（含校准系数） */
    private static final double MM_PER_TICK = 1.0 / POD_TICKS_PER_MM * DISTANCE_CALIBRATION;

    // =========================================================================
    // 2. 运动控制参数（都能调，调之前先看注释）
    // =========================================================================

    /** 认为"到位"的距离误差（mm）。测距轮很准，10mm 足够 */
    public static final double POSITION_TOLERANCE_MM = 10.0;

    /** 进入这个距离（mm）之后开始线性减速 */
    public static final double SLOWDOWN_DISTANCE_MM = 250.0;

    /** 最大输出功率（0~1）。0.5 左右走 1 米比较稳 */
    public static final double MAX_POWER = 0.55;

    /** 减速到最小也保留这个比例的功率，否则静摩擦会让车在终点前停住 */
    public static final double MIN_POWER_FRACTION = 0.18;

    /** 航向保持的 P 增益与输出上限 */
    public static final double HEADING_KP = 1.0;
    public static final double HEADING_MAX_OUTPUT = 0.3;

    /** 航向误差死区（弧度）：小于它就不修正，避免原地小幅抖动 */
    public static final double HEADING_DEADBAND_RAD = Math.toRadians(0.8);

    /** 单次移动超时（毫秒）：超时就停下并报错，避免卡在循环里顶着墙推 */
    public static final long MOVE_TIMEOUT_MS = 6000;

    // =========================================================================
    // 3. 内部状态
    // =========================================================================

    private final DcMotor[] motors = new DcMotor[4];
    private final String[] motorNames = new String[4];
    private final String[] motorPorts = new String[4];

    private GoBildaPinpointDriver pinpoint;

    /** 上一次读到的两个测距轮原始 tick */
    private int lastXTicks;
    private int lastYTicks;

    /** 自 resetPose 以来两个测距轮各自“净走”的距离（mm，已按方向修正） */
    private double podXDistanceMm;
    private double podYDistanceMm;

    /** 设备航向与我们位姿航向之间的固定差值（resetPose 时记录） */
    private double headingOffsetRad;

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

        // ---- Pinpoint：只当“测距轮读卡器 + 陀螺仪”用 ----
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        } catch (RuntimeException e) {
            throw new IllegalStateException(
                    "找不到 Pinpoint：" + e.getMessage()
                            + "。（要求：机器人配置里有名为 \"" + PINPOINT_NAME
                            + "\" 的设备，且类型选的是 GoBildaPinpointDriver）"
                            + " 当前配置里的 Pinpoint：" + pinpointNames(hardwareMap), e);
        }
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);
        pinpoint.resetPosAndIMU();   // 校准 IMU（要求车静止）
        pinpoint.update();

        // 建立编码器基准并把位姿清零
        resetPose();
    }

    /**
     * 把当前位置和航向清零：让"现在车头朝的方向"成为 0°，"现在的位置"成为 (0,0)。
     * 只记录基准，不要求电机停转，随时可以调用。
     */
    public void resetPose() {
        ensurePinpoint();
        pinpoint.update();

        headingOffsetRad = pinpoint.getHeading(AngleUnit.RADIANS);
        lastXTicks = pinpoint.getEncoderX();
        lastYTicks = pinpoint.getEncoderY();
        podXDistanceMm = 0;
        podYDistanceMm = 0;

        poseXmm = 0;
        poseYmm = 0;
        poseHeadingRad = 0;
    }

    // =========================================================================
    // 5. 里程计读取（两个测距轮的原始 ticks）
    // =========================================================================

    /**
     * 每圈必须调用：读两个测距轮的原始计数和 IMU 航向，积分成 (x, y, heading)。
     * 它不发送任何功率，只更新位姿。
     */
    public void update() {
        ensurePinpoint();
        pinpoint.update();

        // ---- 1) 两个测距轮这一圈各走了多少毫米 ----
        int xTicks = pinpoint.getEncoderX();
        int yTicks = pinpoint.getEncoderY();
        double dXPod = (xTicks - lastXTicks) * MM_PER_TICK * X_POD_SIGN;
        double dYPod = (yTicks - lastYTicks) * MM_PER_TICK * Y_POD_SIGN;
        lastXTicks = xTicks;
        lastYTicks = yTicks;
        podXDistanceMm += dXPod;
        podYDistanceMm += dYPod;

        // ---- 2) 航向（IMU）：转成相对 resetPose 的角度 ----
        double deviceHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        double newHeading = normalizeAngle(deviceHeading - headingOffsetRad);
        double dHeading = normalizeAngle(newHeading - poseHeadingRad);

        // ---- 3) 补偿测距轮不在旋转中心带来的额外行程 ----
        double dX = dXPod + dHeading * X_POD_OFFSET_MM;
        double dY = dYPod - dHeading * Y_POD_OFFSET_MM;

        // ---- 4) 车体坐标 → 场地坐标（用中点航向，走弧线更准）----
        double midHeading = normalizeAngle(newHeading - dHeading / 2.0);
        poseXmm += Math.cos(midHeading) * dX - Math.sin(midHeading) * dY;
        poseYmm += Math.sin(midHeading) * dX + Math.cos(midHeading) * dY;
        poseHeadingRad = newHeading;
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

    /** 自 resetPose 以来 X 测距轮净走的距离（mm） */
    public double getPodXDistanceMm() {
        return podXDistanceMm;
    }

    /** 自 resetPose 以来 Y 测距轮净走的距离（mm） */
    public double getPodYDistanceMm() {
        return podYDistanceMm;
    }

    // =========================================================================
    // 6. 直接驱动（车体坐标系）
    // =========================================================================

    /**
     * 按车体坐标系驱动。
     *
     * 这里的 4 个式子与实车验证过的 Test_dipan 完全一致：
     *   Test_dipan 里   axial_t = left_stick_y（推上 = -1）
     *                  lateral_t = -left_stick_x（推右 = -1）
     * 而本函数约定   axial = 前进为正、lateralRight = 右移为正、yawRight = 右转为正，
     * 把 axial_t = -axial、lateral_t = -lateralRight 代进 Test_dipan 的混控，
     * 就得到下面这组符号。别改成教科书上的标准麦轮公式，否则实车方向会错。
     *
     * @param axial        前进为正（车头方向）
     * @param lateralRight 右移为正
     * @param yawRight     顺时针（右转）为正
     */
    public void driveRobotFrame(double axial, double lateralRight, double yawRight) {
        double lf = -axial - lateralRight + yawRight;
        double rf = -axial + lateralRight - yawRight;
        double lb = +axial - lateralRight - yawRight;
        double rb = +axial + lateralRight + yawRight;

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

    /** 两个测距轮自 resetPose 以来走的距离，用于检查编码器方向 */
    public String describePods() {
        return String.format("X轮=%+.0f mm   Y轮=%+.0f mm", podXDistanceMm, podYDistanceMm);
    }

    public boolean isReady() {
        return motors[0] != null && motors[3] != null && pinpoint != null;
    }

    // =========================================================================
    // 9. 工具方法
    // =========================================================================

    private void ensurePinpoint() {
        if (pinpoint == null) {
            throw new IllegalStateException(
                    "Pinpoint 没有初始化（检查机器人配置里是否叫 \"" + PINPOINT_NAME + "\"）");
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
