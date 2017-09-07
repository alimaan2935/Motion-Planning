package solution;

import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

public class Testing {



    public static void main(String[] args) {
        try {

            ProblemSpec ps = new ProblemSpec();
            ps.loadProblem("testcases/3ASV-easy.txt");

            Rectangle2D rect = ps.getObstacles().get(0).getRect();

            System.out.println("x: "+ rect.getX());
            System.out.println("y: "+ rect.getY());

            PathIterator it = rect.getPathIterator(null);

            double[] list = new double[2];
            for (int i=0; i<6; i++) {
                it.currentSegment(list);
                for (double j : list)
                    System.out.println(j);
                it.next();
            }

            Rectangle2D rect1 = new Rectangle2D.Double(0.375,0.1,0.25,0.35);

            Point2D p1 = new Point2D.Double(0.150, 0.225);
            Point2D p2 = new Point2D.Double(0.850, 0.225);


            System.out.println();









        }catch (Exception e) {
            System.out.println("Exceptionnnnnnnnnnn");
        }

    }
}
