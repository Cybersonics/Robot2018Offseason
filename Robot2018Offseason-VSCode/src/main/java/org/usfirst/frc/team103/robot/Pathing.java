package org.usfirst.frc.team103.robot;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.*;

public class Pathing {
	private static final double EPSILON = 0.1;

	private static final Point2D.Double[] BOUNDARY_POINTS = {
			new Point2D.Double(52.5, 0.0),
			new Point2D.Double(137.0, 0.0),
			new Point2D.Double(137.0, 329.44),
			new Point2D.Double(120.44, 329.44),
			new Point2D.Double(120.44, 269.09),
			new Point2D.Double(84.78, 269.09),
			new Point2D.Double(84.78, 240.84),
			new Point2D.Double(-84.78, 240.84),
			new Point2D.Double(-84.78, 269.09),
			new Point2D.Double(-120.44, 269.09),
			new Point2D.Double(-120.44, 329.44),
			new Point2D.Double(-137.0, 329.44),
			new Point2D.Double(-137.0, 0.0),
			new Point2D.Double(-52.5, 0.0),
			new Point2D.Double(-52.5, 114.44),
			new Point2D.Double(-101.75, 114.44),
			new Point2D.Double(-101.75, 235.44),
			new Point2D.Double(101.75, 235.44),
			new Point2D.Double(101.75, 114.44),
			new Point2D.Double(52.5, 114.44)
	};
	private static final int BOUNDARY_COUNT = BOUNDARY_POINTS.length;
	private static final Line2D.Double[] BOUNDARY_EDGES = new Line2D.Double[BOUNDARY_COUNT];
	private static final Path2D.Double BOUNDARY_SHAPE = new Path2D.Double();
	private static final Map<Point2D.Double, Set<Point2D.Double>> BOUNDARY_VISIBILITY = new LinkedHashMap<>();

	static {
		for (int i = 0; i < BOUNDARY_COUNT; i++) {
			Point2D.Double p = BOUNDARY_POINTS[i];
			Point2D.Double q = BOUNDARY_POINTS[(i + 1) % BOUNDARY_COUNT];
			BOUNDARY_EDGES[i] = new Line2D.Double(p.x, p.y, q.x, q.y);
		}

		BOUNDARY_SHAPE.moveTo(BOUNDARY_POINTS[0].x, BOUNDARY_POINTS[0].y);
		for (int i = 1; i < BOUNDARY_COUNT; i++) {
			BOUNDARY_SHAPE.lineTo(BOUNDARY_POINTS[i].x, BOUNDARY_POINTS[i].y);
		}
		BOUNDARY_SHAPE.closePath();

		for (int currentIndex = 0; currentIndex < BOUNDARY_COUNT; currentIndex++) {
			Point2D.Double current = BOUNDARY_POINTS[currentIndex];

			int previousIndex = (currentIndex - 1 + BOUNDARY_COUNT) % BOUNDARY_COUNT;
			int nextIndex = (currentIndex + 1) % BOUNDARY_COUNT;
			addVisible(current, BOUNDARY_POINTS[previousIndex]);
			addVisible(current, BOUNDARY_POINTS[nextIndex]);

			for (int testIndex = 0; testIndex < BOUNDARY_COUNT; testIndex++) {
				if (testIndex == previousIndex || testIndex == currentIndex || testIndex == nextIndex) {
					continue;
				}

				Point2D.Double test = BOUNDARY_POINTS[testIndex];

				boolean hasVisibility = true;
				for (int edgeIndex = 0; edgeIndex < BOUNDARY_COUNT; edgeIndex++) {
					int edgeEndIndex = (edgeIndex + 1) % BOUNDARY_COUNT;
					if (currentIndex == edgeIndex || currentIndex == edgeEndIndex || testIndex == edgeIndex || testIndex == edgeEndIndex) {
						continue;
					}

					Line2D.Double edge = BOUNDARY_EDGES[edgeIndex];
					if (edge.intersectsLine(current.x, current.y, test.x, test.y)) {
						hasVisibility = false;
						break;
					}
				}

				if (hasVisibility) {
					Point2D.Double previous = BOUNDARY_POINTS[previousIndex];
					Point2D.Double next = BOUNDARY_POINTS[nextIndex];
					double crossNextPrevious = cross(current, next, current, previous);
					double crossNextTest = cross(current, next, current, test);
					double crossTestPrevious = cross(current, test, current, previous);
					if (crossNextPrevious >= 0.0) {
						if (crossNextTest >= 0.0 && crossTestPrevious >= 0.0) {
							addVisible(current, test);
						}
					} else {
						if (!(crossNextTest < 0.0 && crossTestPrevious < 0.0)) {
							addVisible(current, test);
						}
					}
				}
			}
		}
	}

	private static void addVisible(Point2D.Double p, Point2D.Double q) {
		BOUNDARY_VISIBILITY.computeIfAbsent(p, k -> new LinkedHashSet<>()).add(q);
	}

	private static double cross(Point2D.Double a1, Point2D.Double a2, Point2D.Double b1, Point2D.Double b2) {
		return (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);
	}

	public static List<Point2D.Double> shortestPath(Point2D.Double from, Point2D.Double to) {
		Map<Point2D.Double, Set<Point2D.Double>> visibility = new LinkedHashMap<>();
		for (Map.Entry<Point2D.Double, Set<Point2D.Double>> entry : BOUNDARY_VISIBILITY.entrySet()) {
			visibility.put(entry.getKey(), new LinkedHashSet<>(entry.getValue()));
		}
		visibility.put(from, new LinkedHashSet<>());
		visibility.put(to, new LinkedHashSet<>());

		List<Point2D.Double> pointsToUpdate = new ArrayList<>();

		for (Point2D.Double p : Arrays.asList(from, to)) {
			if (BOUNDARY_SHAPE.contains(p.x, p.y)) {
				pointsToUpdate.add(p);
			} else {
				Line2D.Double closestEdge = Collections.min(
						Arrays.asList(BOUNDARY_EDGES),
						Comparator.comparing(edge -> edge.ptSegDistSq(p))
				);

				double closestDistance = closestEdge.ptSegDist(p);
				double normalX = -(closestEdge.y2 - closestEdge.y1);
				double normalY = closestEdge.x2 - closestEdge.x1;
				double normalLength = Math.hypot(normalX, normalY);
				double interiorX = p.x + normalX / normalLength * (closestDistance + EPSILON);
				double interiorY = p.y + normalY / normalLength * (closestDistance + EPSILON);
				Point2D.Double interiorPoint = new Point2D.Double(interiorX, interiorY);

				visibility.get(p).add(interiorPoint);
				visibility.computeIfAbsent(interiorPoint, k -> new LinkedHashSet<>()).add(p);
				pointsToUpdate.add(interiorPoint);
			}
		}

		for (Point2D.Double current : pointsToUpdate) {
			for (Point2D.Double test : visibility.keySet()) {
				if (current.equals(test)) {
					continue;
				}

				boolean hasVisibility = true;
				for (Line2D.Double edge : BOUNDARY_EDGES) {
					if (current.equals(edge.getP1()) || current.equals(edge.getP2()) || test.equals(edge.getP1()) || test.equals(edge.getP2())) {
						continue;
					}
					if (edge.intersectsLine(current.x, current.y, test.x, test.y)) {
						hasVisibility = false;
						break;
					}
				}

				if (hasVisibility) {
					visibility.get(current).add(test);
					visibility.get(test).add(current);
				}
			}
		}

		Map<Point2D.Double, Map<Point2D.Double, Double>> edges = new LinkedHashMap<>();
		for (Map.Entry<Point2D.Double, Set<Point2D.Double>> entry : visibility.entrySet()) {
			Map<Point2D.Double, Double> neighbors = new LinkedHashMap<>();
			Point2D.Double p = entry.getKey();
			for (Point2D.Double neighbor : entry.getValue()) {
				neighbors.put(neighbor, Math.hypot((neighbor.x - p.x), (neighbor.y - p.y)));
			}
			edges.put(p, neighbors);
		}

		Set<Point2D.Double> unvisited = new LinkedHashSet<>(edges.keySet());
		Map<Point2D.Double, Double> distance = new LinkedHashMap<>();
		for (Point2D.Double p : edges.keySet()) {
			distance.put(p, Double.POSITIVE_INFINITY);
		}
		distance.put(from, 0.0);

		Map<Point2D.Double, Point2D.Double> previous = new LinkedHashMap<>();
		while (!unvisited.isEmpty()) {
			Point2D.Double current = Collections.min(unvisited, Comparator.comparingDouble(distance::get));
			unvisited.remove(current);
			for (Map.Entry<Point2D.Double, Double> neighborEntry : edges.get(current).entrySet()) {
				Point2D.Double neighbor = neighborEntry.getKey();
				double altDistance = distance.get(current) + neighborEntry.getValue();
				if (altDistance < distance.get(neighbor)) {
					distance.put(neighbor, altDistance);
					previous.put(neighbor, current);
				}
			}
		}

		List<Point2D.Double> waypoints = new ArrayList<>();
		Point2D.Double p = to;
		while (p != null && !p.equals(from)) {
			waypoints.add(p);
			p = previous.get(p);
		}
		waypoints.add(from);
		Collections.reverse(waypoints);

		return waypoints;
	}
}
