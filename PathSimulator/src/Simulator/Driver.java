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
		frame.setSize(720, 720);
		frame.add(new JLabel(new ImageIcon("C:/Users/houch/Pictures/DECODE Field.jpg/")));
		frame.setVisible(true);
		
		Field f = new Field(new Pose2d(0, 0, 0));
		frame.add(f);
		do {
			f.paintComponent(frame.getGraphics());
		} while (!f.isFinished());	
	}
}
