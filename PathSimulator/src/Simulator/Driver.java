package Simulator;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

public class Driver {
	
	public static void main(String[] args) {
		JFrame frame = new JFrame("Clueless");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(648, 648);
		frame.add(new JLabel(new ImageIcon("C:/Users/houch/Pictures/DECODE Field.jpg/")));
		
		frame.setVisible(true);
	}
}
