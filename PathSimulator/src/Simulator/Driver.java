package Simulator;

import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import GVF.*;

public class Driver {
	
	public static void main(String[] args) {
		JFrame frame = new JFrame("Clueless");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(648, 648);
		frame.add(new JLabel(new ImageIcon("C:/Users/houch/Pictures/DECODE Field.jpg/")));
		frame.setVisible(true);
		
		
		ArrayList<RepulsionPoint> repulsionPoints = new ArrayList<>();
		Path p = new Path(new Pose2d(0, 0), repulsionPoints)
				.addPoint(new Pose2d(48, 0, 0), false, false)
				.addPoint(new Pose2d(48, 48, Math.PI/2), false, false)
				.addPoint(new Pose2d(0, 48, Math.PI), false, false)
				.addPoint(new Pose2d(0, 0, Math.PI * 3/2), false, false);
		
		
		Drivetrain dt = new Drivetrain(new Pose2d(0, 0));
		dt.setPath(p);
		
		do {
			
		} while (dt.state != Drivetrain.State.WAIT);
	}
}
