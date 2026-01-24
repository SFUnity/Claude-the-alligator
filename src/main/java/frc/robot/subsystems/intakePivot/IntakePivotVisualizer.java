package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakePivotVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d intake;
  private final String key;

  private final LoggedTunableNumber xOffset = new LoggedTunableNumber("Intake/xOffset", -10.2);
  private final LoggedTunableNumber yOffset = new LoggedTunableNumber("Intake/yOffset", -8);
  private final LoggedTunableNumber zOffset = new LoggedTunableNumber("Intake/zOffset", 9);

  public IntakePivotVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(1, 1, new Color8Bit(Color.kBlack));
    root = mechanism.getRoot("Intake Root", 0, Units.inchesToMeters(9.063));
    intake =
        new LoggedMechanismLigament2d(
            "Intake",
            armLengthMeters,
            Units.radiansToDegrees(maxAngleRads),
            8,
            new Color8Bit(color));

    root.append(intake);
  }

  /** Update intake visualizer with current intake angle */
  public void update(Angle angle) {
    angle = Degrees.of(-1.0758 * angle.in(Degrees) + 89);

    // Log Mechanism2d
    intake.setAngle(angle.in(Degrees));
    Logger.recordOutput("Intake/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    // The inches to meters stuff here will have to be changed
    Pose3d carriagePose =
        new Pose3d(
            Units.inchesToMeters(xOffset.get()),
            Units.inchesToMeters(yOffset.get()),
            Units.inchesToMeters(zOffset.get()),
            new Rotation3d(0.0, angle.in(Radians), 0.0));
    Logger.recordOutput("Intake/Mechanism3d/" + key, carriagePose);
  }
}


