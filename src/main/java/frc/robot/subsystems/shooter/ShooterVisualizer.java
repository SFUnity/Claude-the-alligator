package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ShooterVisualizer {
    private LoggedMechanism2d shooterMech;
    private LoggedMechanismRoot2d shooterBase;
    private LoggedMechanismLigament2d turret;
    private LoggedMechanismLigament2d hood;
    private LoggedMechanismLigament2d shooterVisualizer;
    private double xCoord = 0; //TODO update
    private double yCoord = 0;
    private double zCoord = 0;

    public ShooterVisualizer() {

        shooterMech = new LoggedMechanism2d(0, 0);
        shooterBase = shooterMech.getRoot("Shooter Base", xCoord, yCoord);
        turret = new LoggedMechanismLigament2d("Turret", 0, 0);
        hood = new LoggedMechanismLigament2d("Hood", 0, 0);
        shooterVisualizer = new LoggedMechanismLigament2d("Shooter Visualizer", 0, 0);

        shooterBase.append(turret);
        turret.append(hood);
        hood.append(shooterVisualizer);

    }

    public void update(double turretAngle, double hoodAngle, double flywheelSpeed) {
        turret.setAngle(turretAngle);
        hood.setAngle(hoodAngle);
        shooterVisualizer.setLength(flywheelSpeed);

        Logger.recordOutput("Shooter/Mechanism2D", shooterMech);

        Pose3d ShooterPose = new Pose3d(
            xCoord + Math.cos(turretAngle)*Math.cos(hoodAngle)*flywheelSpeed,
            yCoord + Math.sin(turretAngle)*Math.cos(hoodAngle)*flywheelSpeed,
            zCoord + Math.sin(hoodAngle)*flywheelSpeed,
            new Rotation3d()
        );

        Logger.recordOutput("Shooter/Mechanism3D", ShooterPose);
    }
}
