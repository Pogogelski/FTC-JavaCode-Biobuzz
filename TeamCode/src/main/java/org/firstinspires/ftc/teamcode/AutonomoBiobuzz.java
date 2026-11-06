package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// preselectTeleOp: quando o autônomo acabar, o Driver Hub já deixa o TeleOp selecionado
@Autonomous(name = "Autonomo Biobuzz", preselectTeleOp = "Biobuzz2026")
public class AutonomoBiobuzz extends LinearOpMode {

    // ============ MESMOS AJUSTES DO TELEOP ============
    static final boolean INVERTER_STRAFE = true;

    // ============ SHOOTER E PORTÃO NO AUTÔNOMO ============
    static final double SHOOTER_AUTONOMO = -0.65;  // fica ligado o autônomo inteiro
    static final double TEMPO_AUTONOMO   = 27.0;  // segundos

    static final double TEMPO_ACELERAR   = 5.0;   // espera do shooter ligar até a 1ª bola
    static final double TEMPO_DESLIGAR   = 4.0;   // shooter desliga isso depois da última bola
    static final int    QUANTIDADE_BOLAS = 3;     // quantas vezes o portão abre e fecha
    static final double INTERVALO_BOLAS  = 3.5;   // segundos entre cada abertura
    // Quanto tempo o portão fica aberto: o mínimo para o servo chegar lá e voltar.
    // Se NENHUMA bola cair, aumente (ex.: 0.20). Se caírem DUAS, diminua (ex.: 0.10).
    static final double TEMPO_PORTAO_ABERTO = 0.20;

    static final double SERVO_CURSO_TOTAL = 270.0;
    static final double SERVO_LIMITE      = 90.0;
    static final double SERVO_ANGULO      = 50.0;
    static final double PORTAO_ABERTO     = 0.0;
    static final double PORTAO_FECHADO    = SERVO_ANGULO / SERVO_LIMITE;

    // ============ AJUSTES DO AUTÔNOMO ============
    // Força da correção para andar reto (se o robô "balançar", diminua)
    static final double KP_RETO = 0.03;
    // Força do giro (se passar do ângulo, diminua | se ficar lento, aumente)
    static final double KP_GIRO = 0.02;
    static final double GIRO_POTENCIA_MIN = 0.15; // abaixo disso o robô não consegue girar
    static final double GIRO_POTENCIA_MAX = 0.6;
    static final double GIRO_TOLERANCIA   = 2.0;  // graus de erro aceitos

    DcMotor frontLeft, frontRight, backLeft, backRight, shooter, intake;
    Servo servo;
    IMU imu;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime tempoTotal = new ElapsedTime(); // conta o tempo desde o PLAY
    int bolasSoltas = 0;

    // Direção que o robô deve manter (0 = como estava quando apertou PLAY)
    double alvoHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ============ HARDWARE (mesmos nomes do TeleOp) ============
        frontLeft  = hardwareMap.dcMotor.get("Mecanum 1");
        frontRight = hardwareMap.dcMotor.get("Mecanum 2");
        backLeft   = hardwareMap.dcMotor.get("Mecanum 3");
        backRight  = hardwareMap.dcMotor.get("Mecanum 4");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // No autônomo as rodas TRAVAM ao parar, para o robô não deslizar
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.dcMotor.get("Shooter");
        intake  = hardwareMap.dcMotor.get("Intake");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get("Servo");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.scaleRange(0.0, SERVO_LIMITE / SERVO_CURSO_TOTAL);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Portão já fecha no INIT, segurando as bolas antes do PLAY
        servo.setPosition(PORTAO_FECHADO);

        telemetry.addLine("Pronto. Alinhe o robo e aperte PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetYaw();
        alvoHeading = 0;
        tempoTotal.reset(); // começa a contar os 25 segundos

        // =====================================================
        // ============ SEQUÊNCIA DO AUTÔNOMO ==================
        // Comandos disponíveis:
        // andar(frente, lado, segundos) | girar(graus) | esperar(segundos)
        // soltarUmaBola() | esperarAteFim()
        // =====================================================

        // Portão FECHADO segurando as bolas, shooter ligado o tempo todo
        servo.setPosition(PORTAO_FECHADO);
        shooter.setPower(SHOOTER_AUTONOMO);

        // 5 s para o rotor do shooter pegar velocidade
        esperar(TEMPO_ACELERAR);

        for (int i = 1; i <= QUANTIDADE_BOLAS && opModeIsActive(); i++) {
            if (i > 1) {
                esperar(INTERVALO_BOLAS); // 3 s entre uma bola e outra
            }
            soltarUmaBola();
            bolasSoltas = i;
        }

        // Shooter gira mais 4 s depois da última bola e desliga
        esperar(TEMPO_DESLIGAR);
        shooter.setPower(0);

        // ============ FIM DA SEQUÊNCIA ======================

        pararTudo();
    }

    // ============ FUNÇÕES ============

    // Anda por tempo mantendo a direção com o IMU (corrige o robô "puxando" para um lado)
    void andar(double frente, double lado, double segundos) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < segundos) {
            double erro = normalizar(alvoHeading - headingGraus());
            double correcao = -erro * KP_RETO;
            mover(frente, lado, correcao);
            mostrar("Andando");
        }
        mover(0, 0, 0);
    }

    // Gira até o ângulo pedido usando o IMU (+ esquerda, - direita)
    void girar(double graus) {
        alvoHeading = normalizar(alvoHeading + graus);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 3.0) { // limite de 3 s por segurança
            double erro = normalizar(alvoHeading - headingGraus());
            if (Math.abs(erro) < GIRO_TOLERANCIA) break;

            double potencia = Math.abs(erro) * KP_GIRO;
            potencia = Math.max(GIRO_POTENCIA_MIN, Math.min(GIRO_POTENCIA_MAX, potencia));
            // erro positivo = precisa girar para a esquerda = rx negativo
            mover(0, 0, erro > 0 ? -potencia : potencia);
            mostrar("Girando");
        }
        mover(0, 0, 0);
    }

    // Abre o portão o mais rápido possível e fecha de novo: deixa passar só uma bola
    void soltarUmaBola() {
        servo.setPosition(PORTAO_ABERTO);
        esperar(TEMPO_PORTAO_ABERTO);
        servo.setPosition(PORTAO_FECHADO);
    }

    // Segura o shooter ligado até completar o tempo do autônomo
    void esperarAteFim() {
        while (opModeIsActive() && tempoTotal.seconds() < TEMPO_AUTONOMO) {
            mostrar("Shooter ligado ate o fim");
        }
    }

    void esperar(double segundos) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < segundos) {
            mostrar("Esperando");
        }
    }

    // Mesma conta de mecanum do TeleOp (robot-centric)
    void mover(double y, double x, double rx) {
        if (INVERTER_STRAFE) x = -x;
        x = x * 1.1;
        double denominador = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominador);
        backLeft.setPower((y - x + rx) / denominador);
        frontRight.setPower((y - x - rx) / denominador);
        backRight.setPower((y + x - rx) / denominador);
    }

    void pararTudo() {
        mover(0, 0, 0);
        shooter.setPower(0);
        intake.setPower(0);
    }

    double headingGraus() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Deixa o ângulo entre -180 e 180 (evita girar a volta inteira à toa)
    double normalizar(double graus) {
        while (graus > 180)  graus -= 360;
        while (graus < -180) graus += 360;
        return graus;
    }

    void mostrar(String etapa) {
        telemetry.addData("Etapa", etapa);
        telemetry.addData("Tempo", "%.1f / %.0f s", tempoTotal.seconds(), TEMPO_AUTONOMO);
        telemetry.addData("Bolas soltas", "%d / %d", bolasSoltas, QUANTIDADE_BOLAS);
        telemetry.addData("Heading", "%.1f", headingGraus());
        telemetry.addData("Alvo", "%.1f", alvoHeading);
        telemetry.update();
    }
}