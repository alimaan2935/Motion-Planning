package solution;

import tester.Tester;
import visualiser.Visualiser;

import javax.swing.*;


public class Main {

    public static void main(String[] args) {

        Tester tester = new Tester();

        String[] arg = new String[2];
        arg[0] = "testcases/3ASV-easy.txt";
        arg[1] = "testcases/3ASV-easy-sol.txt";

        tester.main(arg);

        JFrame frame = new JFrame("Assignment 1 visualiser");
        Visualiser vis = new Visualiser(frame);
        vis.main(arg);


    }
}
