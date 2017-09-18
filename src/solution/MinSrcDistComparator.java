package solution;

import java.util.Comparator;

/**
 * A Comparator for priority queue.
 * It compares the node's distance from source and assign priority based on the
 * lowest distance.
 *
 * @author Ali Nawaz Maan <a.maan@uqconnect.edu.au>
 *
 */

public class MinSrcDistComparator implements Comparator<PathPoint> {

    @Override
    public int compare(PathPoint p1, PathPoint p2) {

        return Double.compare(p1.getDistanceFromStart(), p2.getDistanceFromStart());
    }
}
