package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.LoggedTunableNumber;
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

  private final LoggedTunableNumber turretXOffset =
      new LoggedTunableNumber("Shooter/turretXOffset", 0);
  private final LoggedTunableNumber turretYOffset =
      new LoggedTunableNumber("Shooter/turretYOffset", 0);
  private final LoggedTunableNumber turretZOffset =
      new LoggedTunableNumber("Shooter/turretZOffset", 0);
  private final LoggedTunableNumber hoodXOffset = new LoggedTunableNumber("Shooter/hoodXOffset", 0);
  private final LoggedTunableNumber hoodYOffset = new LoggedTunableNumber("Shooter/hoodYOffset", 0);
  private final LoggedTunableNumber hoodZOffset = new LoggedTunableNumber("Shooter/hoodZOffset", 0);

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

  public void update(double turretAngle, double hoodAngle) {
    hood.setAngle(hoodAngle);
    Logger.recordOutput("Subsystems/Shooter/Hood/Mechanism2D/" + key, hoodMechanism);

    turret.setAngle(turretAngle);
    Logger.recordOutput("Subsystems/Shooter/Turret/Mechanism2D/" + key, turretMechanism);

    Pose3d shooterPose =
        new Pose3d(
            Units.inchesToMeters(turretXOffset.get()),
            Units.inchesToMeters(turretYOffset.get()),
            Units.inchesToMeters(turretZOffset.get()),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(turretAngle)));
    Pose3d hoodPose =
        shooterPose.transformBy(
            new Transform3d(
                hoodXOffset.get(),
                hoodYOffset.get(),
                hoodZOffset.get(),
                new Rotation3d(0, Units.degreesToRadians(hoodAngle), 0)));
    Logger.recordOutput("Subsystems/Shooter/Turret/Mechanism3D/" + key, shooterPose);
    Logger.recordOutput("Subsystems/Shooter/Hood/Mechanism3D/" + key, hoodPose);
  }
}
