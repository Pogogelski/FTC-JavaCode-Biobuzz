package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class Biobuzz2026 extends LinearOpMode {

    // ============ MECANUM ============
    // O robô estava indo para a esquerda quando mandava ir para a direita.
    // true = inverte o movimento lateral (strafe). Se voltar a inverter, troque para false.
    static final boolean INVERTER_STRAFE = true;

    // Zona morta do analógico: valores menores que isso viram 0.
    // Evita o robô "andar sozinho" quando o analógico não volta exatamente ao centro.
    static final double ZONA_MORTA = 0.08;

    // Sentido de giro (olhando de frente para o eixo do motor):
    // potência POSITIVA = anti-horário | potência NEGATIVA = horário
    static final double SHOOTER_ANTIHORARIO =  -0.6; // RB - 60% - gira para a esquerda
    static final double SHOOTER_HORARIO     = 1.0; // LB - 100% - gira para a direita
    static final double INTAKE_ANTIHORARIO  =  1.0; // D-pad cima - 100%
    static final double INTAKE_HORARIO      = -1.0; // D-pad baixo - 100%

    // ============ SERVO (PORTÃO) ============
    // Ponto 0 = portão ABERTO (para cima)
    // SERVO_ANGULO graus no sentido horário (para a direita) = portão FECHADO
    // Capacidade máxima do servo: REV Smart Robot Servo = 270 graus de fábrica
    static final double SERVO_CURSO_TOTAL = 270.0;
    // Limite de segurança: o servo NUNCA passa de 1/4 de volta
    static final double SERVO_LIMITE      = 90.0;
    // Quanto o servo gira para FECHAR o portão (no máximo SERVO_LIMITE)
    static final double SERVO_ANGULO      = 50.0;
    static final double SERVO_POS_ZERO    = 0.0;                         // portão ABERTO
    static final double SERVO_POS_88      = SERVO_ANGULO / SERVO_LIMITE; // portão FECHADO

    @Override
    public void runOpMode() throws InterruptedException {
        // ============ MECANUM - CONTROL HUB (gamepad1) ============
        // As strings entre aspas precisam bater com a configuração do Driver Hub.
        DcMotor frontLeft  = hardwareMap.dcMotor.get("Mecanum 1"); // porta 0 - frente esquerda
        DcMotor frontRight = hardwareMap.dcMotor.get("Mecanum 2"); // porta 1 - frente direita
        DcMotor backLeft   = hardwareMap.dcMotor.get("Mecanum 3"); // porta 2 - trás esquerda
        DcMotor backRight  = hardwareMap.dcMotor.get("Mecanum 4"); // porta 3 - trás direita

        // Inverte o lado direito.
        // Se o robô andar para trás quando mandar ir para frente,
        // inverta o lado esquerdo em vez do direito.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ============ MECANISMOS - EXPANSION HUB (gamepad2) ============
        DcMotor shooter = hardwareMap.dcMotor.get("Shooter"); // porta 0 do Expansion Hub
        DcMotor intake  = hardwareMap.dcMotor.get("Intake");  // porta 1 do Expansion Hub

        // Mantidos em FORWARD para o sentido seguir as constantes do topo.
        // Se algum girar ao contrário no teste, troque só aquele para REVERSE.
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter desacelera sozinho; intake trava ao soltar
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo do portão na porta 0 de servos (nome precisa bater com o Driver Hub)
        Servo servo = hardwareMap.servo.get("Servo");
        // Ao apertar A, o portão deve girar para a DIREITA (horário).
        // Se girar para a esquerda, troque para REVERSE e refaça a montagem do ponto 0.
        servo.setDirection(Servo.Direction.FORWARD);
        // Trava o servo entre 0 e 90 graus: qualquer posição pedida fica dentro desse intervalo
        servo.scaleRange(0.0, SERVO_LIMITE / SERVO_CURSO_TOTAL);

        // Portão já fecha no INIT, segurando as bolas antes do PLAY
        servo.setPosition(SERVO_POS_88);

        // ============ IMU ============
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Ajuste conforme a montagem do hub no robô
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        // Ao apertar PLAY, a direção atual do robô vira a "frente do campo"
        imu.resetYaw();

        // Ao apertar PLAY, o portão começa FECHADO
        servo.setPosition(SERVO_POS_88);

        // true = field-centric (frente do CAMPO) | false = robot-centric (frente do ROBÔ)
        // Aperte X no gamepad 1 para alternar durante o jogo.
        boolean campoCentrico = true;
        boolean xAnterior = false;

        while (opModeIsActive()) {
            // ============ MOVIMENTAÇÃO - GAMEPAD 1 ============
            double y = zonaMorta(-gamepad1.left_stick_y); // eixo Y do analógico vem invertido
            double x = zonaMorta(gamepad1.left_stick_x);
            double rx = zonaMorta(gamepad1.right_stick_x);

            // X alterna o modo (só uma vez por toque, não fica piscando segurando)
            if (gamepad1.x && !xAnterior) {
                campoCentrico = !campoCentrico;
            }
            xAnterior = gamepad1.x;

            // Botão Y zera a direção "frente do campo"
            // (deixe o robô apontado para frente antes de apertar)
            if (gamepad1.y) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // No robot-centric o ângulo é ignorado (frente = frente do robô)
            double anguloUsado = campoCentrico ? botHeading : 0;

            // Rotaciona o comando do joystick contra o ângulo do robô
            double rotX = x * Math.cos(-anguloUsado) - y * Math.sin(-anguloUsado);
            double rotY = x * Math.sin(-anguloUsado) + y * Math.cos(-anguloUsado);

            // Corrige o strafe invertido DEPOIS da rotação,
            // para o field-centric continuar certo com o robô virado
            if (INVERTER_STRAFE) {
                rotX = -rotX;
            }

            rotX = rotX * 1.1;  // compensa o strafe imperfeito das mecanum

            // Mantém todas as potências entre -1 e 1 sem perder a proporção
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // ============ SHOOTER - GAMEPAD 2 ============
            // Segurando RB: anti-horário 60% | Segurando LB: horário 100% | Solto: parado
            double shooterPower = 0;
            if (gamepad2.right_bumper) {
                shooterPower = SHOOTER_ANTIHORARIO;
            } else if (gamepad2.left_bumper) {
                shooterPower = SHOOTER_HORARIO;
            }
            shooter.setPower(shooterPower);

            // ============ INTAKE - GAMEPAD 2 ============
            // D-pad cima: anti-horário 100% | D-pad baixo: horário 100% | Solto: parado
            double intakePower = 0;
            if (gamepad2.dpad_up) {
                intakePower = INTAKE_ANTIHORARIO;
            } else if (gamepad2.dpad_down) {
                intakePower = INTAKE_HORARIO;
            }
            intake.setPower(intakePower);

            // ============ SERVO (PORTÃO) - GAMEPAD 2 ============
            // A: fecha o portão | B: abre o portão (ponto 0)
            if (gamepad2.a) {
                servo.setPosition(SERVO_POS_88);
            } else if (gamepad2.b) {
                servo.setPosition(SERVO_POS_ZERO);
            }

            // ============ TELEMETRIA ============
            telemetry.addData("Modo (X alterna)", campoCentrico ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Heading (graus)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addData("Strafe invertido", INVERTER_STRAFE);
            telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("Shooter", "%.2f", shooterPower);
            telemetry.addData("Intake", "%.2f", intakePower);
            telemetry.addData("Portão", servo.getPosition() > SERVO_POS_88 / 2 ? "FECHADO" : "ABERTO");
            telemetry.update();
        }
    }

    // Zera valores pequenos do analógico
    private double zonaMorta(double valor) {
        return Math.abs(valor) < ZONA_MORTA ? 0 : valor;
    }
}