package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class S_ResetNavx extends CommandBase {
  private SwerveSubsystem swerveSubs; 
  /** Creates a new S_ResetNavx. */
  public S_ResetNavx(SwerveSubsystem swerveSubs) {
    this.swerveSubs = swerveSubs; 

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubs.resetNavx();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
