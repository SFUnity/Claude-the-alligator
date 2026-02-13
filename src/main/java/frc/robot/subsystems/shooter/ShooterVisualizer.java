package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ShooterVisualizer {
  private final LoggedMechanism2d hoodMechanism;
  private final LoggedMechanismRoot2d hoodRoot;
  private final LoggedMechanismLigament2d hood;

  private final LoggedMechanism2d turretMechanism;
  private final LoggedMechanismRoot2d turretRoot;
  private final LoggedMechanismLigament2d turret;

  private final String key;

  private final double turretRadius = Units.inchesToMeters(4);
  private final double hoodHeight = Units.inchesToMeters(8);

  private final LoggedTunableNumber xOffset = new LoggedTunableNumber("Shooter/XOffset", 0);
  private final LoggedTunableNumber yOffset = new LoggedTunableNumber("Shooter/YOffset", 0);
  private final LoggedTunableNumber zOffset = new LoggedTunableNumber("Shooter/ZOffset", 36);

  public ShooterVisualizer(String key, Color color) {
    this.key = key;

    hoodMechanism = new LoggedMechanism2d(1, 1, new Color8Bit(Color.kBlack));
    hoodRoot = hoodMechanism.getRoot("Hood Root", 0, 0);
    hood = new LoggedMechanismLigament2d("Hood", hoodHeight, 0, 8, new Color8Bit(color));

    turretMechanism = new LoggedMechanism2d(1, 1, new Color8Bit(Color.kBlack));
    turretRoot = turretMechanism.getRoot("Turret Root", 0.5, 0.5);
    turret = new LoggedMechanismLigament2d("Turret", turretRadius, 0, 4, new Color8Bit(color));

    turretRoot.append(turret);
    hoodRoot.append(hood);
  }

  public void update(double turretAngle, double hoodAngle, PoseManager poseManager) {
    hood.setAngle(hoodAngle);
    Logger.recordOutput("Subsystems/Shooter/Hood/Mechanism2D/" + key, hoodMechanism);

    turret.setAngle(turretAngle);
    Logger.recordOutput("Subsystems/Shooter/Turret/Mechanism2D/" + key, turretMechanism);

    Pose2d robotPose = poseManager.getPose();

    Pose3d shooterPose =
        new Pose3d(
            robotPose.getX() + Units.inchesToMeters(xOffset.get()),
            robotPose.getY() + Units.inchesToMeters(yOffset.get()),
            Units.inchesToMeters(zOffset.get()),
            new Rotation3d(
                0.0, Units.degreesToRadians(hoodAngle), Units.degreesToRadians(turretAngle)));
    Logger.recordOutput("Subsystems/Shooter/Mechanism3D/" + key, shooterPose);
  }
}
