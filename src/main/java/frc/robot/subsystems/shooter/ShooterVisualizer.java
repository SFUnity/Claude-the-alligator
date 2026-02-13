package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

    // Pose3d shooterPose = new Pose3d();
    // Logger.recordOutput("Subsystems/Shooter/Mechanism3D/" + key, shooterPose);
  }
}
