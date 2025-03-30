// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.videoplayer;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SimulationVideoConstants;
import frc.robot.subsystems.videoplayer.VideoPlayer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlayVideoCmd extends Command {
  private final VideoPlayer videoPlayer;
  private final Timer timer = new Timer();
  private final List<Pose2d> poses;
  private final boolean logAscii;
  private int currentFrame = 0;

  /** Creates a new PlayVideo. */
  public PlayVideoCmd(VideoPlayer videoPlayer) {
    this(videoPlayer, videoPlayer.getPoses());
  }

  /** Creates a new PlayVideo. */
  public PlayVideoCmd(VideoPlayer videoPlayer, List<Pose2d> poses) {
    this(videoPlayer, poses, false);
  }

  /** Creates a new PlayVideo. */
  public PlayVideoCmd(VideoPlayer videoPlayer, List<Pose2d> poses, boolean logAscii) {
    this.videoPlayer = videoPlayer;
    this.poses = poses;
    this.logAscii = logAscii;
    Logger.recordOutput("VideoPlayer/Display/Head", currentFrame);
    addRequirements(videoPlayer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    videoPlayer.resetPoses();
    currentFrame = 0;
    timer.restart();
    Logger.recordOutput("VideoPlayer/Display/Head", currentFrame);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!videoPlayer.isAllowed()) {
      return;
    }
    if (timer.hasElapsed(SimulationVideoConstants.kDisplayDeltaSeconds)) {
      timer.restart();
      Boolean[][] frame = videoPlayer.getFrame(currentFrame);
      String consoleScreen = "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
      for (int y = 0; y < frame.length; y++) {
        consoleScreen += "\n";
        Boolean[] row = frame[y];
        for (int x = 0; x < row.length; x++) {
          Pose2d defaultPose = videoPlayer.calculateDefaultPose(x, y);
          Boolean chunkEnabled = (row[x] == null ? false : row[x]);
          poses.set((y * SimulationVideoConstants.kDisplayWidth) + x,
              chunkEnabled ? defaultPose
                  : new Pose2d(defaultPose.getX() + 100, defaultPose.getY() + 100, Rotation2d.kZero));
          consoleScreen += chunkEnabled ? "###" : "   ";
        }
      }
      logAscii(currentFrame + "\n" + consoleScreen);
      currentFrame++;
      Logger.recordOutput("VideoPlayer/Display/Head", currentFrame);
    }
  }

  private void logAscii(String output) {
    if (logAscii) {
      System.out.println(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    videoPlayer.resetPoses();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentFrame >= videoPlayer.getFrameCount() || !videoPlayer.isAllowed();
  }
}
