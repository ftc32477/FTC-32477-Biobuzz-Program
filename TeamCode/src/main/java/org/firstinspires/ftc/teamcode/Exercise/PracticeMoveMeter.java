package org.firstinspires.ftc.teamcode.Exercise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ============================================================================
 * 练习 1：左摇杆指方向，车自动走 1 米（位置只来自 Pinpoint 两个测距轮）
 * ============================================================================
 *
 * 【用法】
 *   1. Driver Station 里选 "Practice1: 摇杆方向走 1 米（测距轮）"，按 INIT。
 *      INIT 阶段可以用手推车，检查测距轮方向对不对（见下面"自检"）。
 *   2. 按 START。程序会把"当前位置"清零、"当前车头方向"记为 0°，
 *      所以之后所有动作都相对于它现在停着的姿势。
 *   3. 推左摇杆（走的过程中手可以松开，车会自己收敛到目标点）：
 *        上 →   0°  → 沿车头正前方走 1000 mm
 *        左 →  90°  → 向左走 1000 mm
 *        下 → 180°  → 向后走 1000 mm
 *        右 → 270°  → 向右走 1000 mm
 *      斜推时取"占主导的那个轴"。
 *      摇杆必须回到中间，才能触发下一次（防止一直推着连发）。
 *   4. 按键：B = 紧急停止；Y = 把坐标和航向重新清零。
 *
 * 【位置从哪来】
 *   X/Y 位置只用 Pinpoint 上两个测距轮的原始编码器计数
 *   （getEncoderX / getEncoderY），自己积分算出来；
 *   不读 Pinpoint 内部融合的 getPosX / getPosY。
 *   航向用 Pinpoint 的 IMU —— 两个测距轮在数学上测不出转向，
 *   这一项不是"位置信息"。
 *
 * 【自检：INIT 阶段 30 秒做完】
 *   用手把车向前推约 1 米 → 屏幕 X 应变成 +1000 左右，X 轮行程为正；
 *   把车向左推         → Y 应变大，Y 轮行程为正；
 *   把车逆时针转 90°   → 航向应变成 +90 左右。
 *   如果推车时某个轮子行程是负的，去 PracticeChassis 把
 *   X_POD_SIGN / Y_POD_SIGN 对应项改成 -1。
 *
 * 【走不准怎么校准】
 *   命令走 1000 mm，用尺子量实际走了多少（例如 950 mm），
 *   把 PracticeChassis.DISTANCE_CALIBRATION 改成 1000 / 950 ≈ 1.053。
 *
 * 【为什么能走准】
 *   不是"按时间给功率"，而是每圈读测距轮、算还差多少、越近给越小功率，
 *   进了 10 mm 误差就停。测距轮不受麦轮打滑影响，比用底盘电机编码器准得多。
 * ============================================================================
 */
@TeleOp(name = "Practice1: 摇杆方向走 1 米（测距轮）", group = "练习")
public class PracticeMoveMeter extends LinearOpMode {

    /** 每次走多远（毫米）。改成 500 就是走半米 */
    private static final double MOVE_DISTANCE_MM = 1000.0;

    /** 左摇杆超过这个量才算"下了一条指令" */
    private static final double STICK_TRIGGER = 0.7;

    /** 左摇杆低于这个量才算"已回到中间"，才解锁下一条指令 */
    private static final double STICK_NEUTRAL = 0.3;

    private static final String[] DIRECTION_NAME = {"前", "左", "后", "右"};

    private PracticeChassis chassis;

    /** true = 摇杆已回中，允许接收下一条指令 */
    private boolean armed = true;

    /** 上一次走的方向（度），仅用于遥测显示 */
    private double lastDirectionDeg = 0;

    @Override
    public void runOpMode() {

        // ---------------- Init：取硬件 ----------------
        chassis = new PracticeChassis();
        try {
            chassis.init(hardwareMap);
        } catch (IllegalStateException e) {
            telemetry.addLine("底盘初始化失败：");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            while (opModeInInit()) {
                sleep(50);
            }
            return;
        }

        // ---------------- 等待 START：推车自检测距轮 ----------------
        while (opModeInInit()) {
            chassis.update();
            telemetry.addLine("==== 测距轮自检（用手推车试试）====");
            telemetry.addLine("向前推 1 米 → X 应变成 +1000 左右");
            telemetry.addLine("向左推        → Y 应变大");
            telemetry.addLine("逆时针转 90°  → 航向应变 +90 左右");
            telemetry.addData("现在 X (mm)", "%.1f", chassis.getXmm());
            telemetry.addData("现在 Y (mm)", "%.1f", chassis.getYmm());
            telemetry.addData("航向 (度)", "%.1f  (逆时针为正)", chassis.getHeadingDeg());
            telemetry.addLine(chassis.describePods());
            telemetry.addLine("-----------------------------------------------");
            telemetry.addLine("按 START 开始：把当前位置清零、当前车头记为 0°");
            telemetry.addLine(chassis.describeMotors());
            telemetry.update();
            sleep(30);
        }

        waitForStart();

        // 起跑瞬间清零：以后所有坐标都相对"现在的位置和朝向"
        chassis.resetPose();
        armed = true;

        // ---------------- 主循环 ----------------
        while (opModeIsActive()) {

            // 1) 先刷新测距轮里程计（每圈必须做）
            chassis.update();

            // 2) Y 键：重新清零坐标
            if (gamepad1.yWasPressed()) {
                chassis.abortMove("操作手重新清零");
                chassis.resetPose();
            }

            // 3) B 键：紧急停止
            if (gamepad1.b) {
                chassis.abortMove("按了 B 键");
            }

            // 4) 读左摇杆（推上 = +1，推右 = +1）
            double stickForward = -gamepad1.left_stick_y;
            double stickRight = gamepad1.left_stick_x;
            double stickMagnitude = Math.hypot(stickForward, stickRight);

            // 5) 不在移动中才接受新指令
            if (!chassis.isMoving()) {
                if (stickMagnitude < STICK_NEUTRAL) {
                    armed = true;          // 摇杆回中，解锁下一条指令
                }
                if (armed && stickMagnitude > STICK_TRIGGER) {
                    double directionDeg = stickToDirection(stickForward, stickRight);
                    lastDirectionDeg = directionDeg;
                    chassis.startMove(directionDeg, MOVE_DISTANCE_MM);
                    armed = false;         // 必须松回中间才能再下一条
                }
            } else {
                // 6) 移动中：让底盘算这一圈的四个轮子功率
                chassis.updateMove();
            }

            // 7) 遥测
            telemetry.addLine("==== 左摇杆 → 走 " + (int) MOVE_DISTANCE_MM + " mm（测距轮） ====");
            telemetry.addLine("上=0°(前)  左=90°  下=180°(后)  右=270°");
            telemetry.addLine("B=急停   Y=坐标清零");
            telemetry.addLine("-----------------------------------------------");
            telemetry.addData("摇杆", "前后=%+.2f 左右=%+.2f 幅度=%.2f",
                    stickForward, stickRight, stickMagnitude);
            telemetry.addData("状态", chassis.isMoving() ? "移动中…" : (armed ? "待命（可推摇杆）" : "请把摇杆松回中间"));

            if (chassis.isMoving()) {
                telemetry.addData("目标方向", "%.0f°（%s）", lastDirectionDeg,
                        DIRECTION_NAME[directionIndex(lastDirectionDeg)]);
                telemetry.addData("剩余距离", "%.0f mm", chassis.getRemainingMm());
            } else {
                telemetry.addData("上一次结果", chassis.getMoveResult());
                telemetry.addData("上次实际位移", "%.0f mm（目标 %.0f mm）",
                        chassis.getLastDisplacementMm(), MOVE_DISTANCE_MM);
                telemetry.addData("上次终点误差", "%.1f mm", chassis.getLastErrorMm());
            }

            telemetry.addLine("-----------------------------------------------");
            telemetry.addData("坐标", "X=%.1f mm  Y=%.1f mm", chassis.getXmm(), chassis.getYmm());
            telemetry.addData("航向", "%.1f 度", chassis.getHeadingDeg());
            telemetry.addData("测距轮", chassis.describePods());
            telemetry.addData("电机", chassis.describeMotors());
            telemetry.update();
        }

        chassis.stop();
    }

    /**
     * 把摇杆读数映射成 4 个离散方向之一：取占主导的那个轴。
     * 上 → 0°，左 → 90°，下 → 180°，右 → 270°（逆时针为正）。
     */
    private double stickToDirection(double forward, double right) {
        boolean verticalDominant = Math.abs(forward) >= Math.abs(right);
        if (verticalDominant) {
            return forward > 0 ? 0 : 180;
        }
        // 往左（right < 0）是 90°，往右是 270°
        return right > 0 ? 270 : 90;
    }

    /** 0/90/180/270 → 0/1/2/3，用于显示"前/左/后/右" */
    private int directionIndex(double deg) {
        int index = (int) Math.round(deg / 90.0) % 4;
        if (index < 0) {
            index += 4;
        }
        return index;
    }
}
