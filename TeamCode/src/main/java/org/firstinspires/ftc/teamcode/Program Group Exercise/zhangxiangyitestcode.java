package 程序组代码练习;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "张祥义测试底盘", group = "Linear OpMode")
public class zhangxiangyitestcode extends LinearOpMode {

    // 声明四个底盘电机
    private DcMotor leftFrontDrive = null;   // Port 0: LF
    private DcMotor rightFrontDrive = null;  // Port 1: RF
    private DcMotor leftBackDrive = null;    // Port 3: LB
    private DcMotor rightBackDrive = null;   // Port 2: RB

    @Override
    public void runOpMode() {

        // 1. 从硬件映射 (HardwareMap) 中获取电机对象，名称需与 Driver Hub 配置保持一致
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "RB");

        // 2. 设置电机旋转方向（右侧电机反转）
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. 设置电机停止时的刹车模式
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        // 等待 Driver Station 点击 Start 按钮
        waitForStart();

        // 主控制循环
        while (opModeIsActive()) {

            // 4. 读取手柄 1 的左摇杆数据
            // 注意：FTC 手柄 Y 轴向上推返回 -1.0，因此前面加负号取反
            double drive = -gamepad1.left_stick_y; // 前后运动：上推为正 (+), 下拉为负 (-)
            double strafe = gamepad1.left_stick_x; // 左右平移：右推为正 (+), 左推为负 (-)

            /* 麦克纳姆轮全向移动动力学计算 */
            double leftFrontPower  = drive + strafe;
            double rightFrontPower = drive - strafe;
            double leftBackPower   = drive - strafe;
            double rightBackPower  = drive + strafe;

            // 5. 归一化处理，防止摇杆斜推时数值超过 1.0 导致动作变形
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));

            if (maxPower > 1.0) {
                leftFrontPower  /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower   /= maxPower;
                rightBackPower  /= maxPower;
            }

            // 6. 给电机输出功率
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // 7. 在 Driver Hub 屏幕上实时显示数据
            telemetry.addData("Drive (Y)", "%5.2f", drive);
            telemetry.addData("Strafe (X)", "%5.2f", strafe);
            telemetry.addData("Motors Power", "LF (%.2f), RF (%.2f), LB (%.2f), RB (%.2f)",
                    leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}