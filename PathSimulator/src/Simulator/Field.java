package Simulator;

import java.awt.Graphics;

import javax.swing.JPanel;

import GVF.Pose2d;

public class Field extends JPanel {

	protected void paintComponent(Graphics g, Pose2d robot) {
		super.paintComponent(g);
		
		
	}
	
	private Pose2d convertPoseToPixels(Pose2d robot) {
		return new Pose2d(robot.x * 4.5 + 360, robot.y * 4.5 + 360);
	}
}
