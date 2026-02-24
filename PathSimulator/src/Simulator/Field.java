package Simulator;

import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JPanel;

import GVF.Drivetrain;
import GVF.Path;
import GVF.Pose2d;
import GVF.RepulsionPoint;

public class Field extends JPanel {
	private ArrayList<RepulsionPoint> repulsionPoints;
	private Path p;
	private Drivetrain dt;
	
	private Pose2d lastPose;
	
	public Field (Pose2d startPose) {
		repulsionPoints = new ArrayList<>();
		
		
		p = new Path(new Pose2d(0, 0, 0), repulsionPoints)
				.addPoint(new Pose2d(48, 0, 0), false, false)
				.addPoint(new Pose2d(48, 48, Math.PI/2), false, false)
				.addPoint(new Pose2d(0, 48, Math.PI), false, false)
				.addPoint(new Pose2d(0, 0, Math.PI * 3/2), false, false);
		
		
		dt = new Drivetrain(startPose.clone());
		lastPose = startPose.clone();
		dt.setPath(p);
	}
	
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		
		dt.update();
		
		Pose2d pixel1 = convertPoseToPixels(lastPose);
		Pose2d pixel2 = convertPoseToPixels(dt.getPose());
		g.drawLine((int) pixel1.x, (int) pixel1.y, (int) pixel2.x, (int) pixel2.y);
		
		lastPose = dt.getPose();
	}
	
	private Pose2d convertPoseToPixels(Pose2d robot) {
		return new Pose2d(robot.x * 4.5 + 360, robot.y * 4.5 + 360);
	}
	
	public boolean isFinished() {
		return dt.state != Drivetrain.State.WAIT;
	}
}
