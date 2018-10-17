package org.usfirst.frc.team103.command;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.usfirst.frc.team103.subsystem.Elevator;
import org.usfirst.frc.team103.robot.RobotMap;
import org.usfirst.frc.team103.util.DynamicCommandGroup;

public class PathTo extends DynamicCommandGroup {
	private final double targetX, targetY;
	private final double targetHeading;
	private final double cruiseSpeed, endSpeed;
	
	public PathTo(double targetX, double targetY, double targetHeading, double cruiseSpeed, double endSpeed) {
		this.targetX = targetX;
		this.targetY = targetY;
		this.targetHeading = targetHeading;
		this.cruiseSpeed = cruiseSpeed;
		this.endSpeed = endSpeed;

		requires(RobotMap.drive);
		requires(RobotMap.elevator);
	}
	
	@Override
	protected void dynamicInitialize() {
		Point robotPosition = new Point(RobotMap.positioning.getX(), RobotMap.positioning.getY());
		Point targetPosition = new Point(targetX, targetY);
		double currentHeading = RobotMap.positioning.getHeading();
		System.out.println("Pathing to " + targetPosition + " from " + robotPosition);
		
		// Update visibility graph
		Map<Point, Set<Point>> visibility = new LinkedHashMap<>();
		for (Map.Entry<Point, Set<Point>> entry : BOUNDARY_VISIBILITY.entrySet()) {
			Set<Point> visible = new LinkedHashSet<>();
			visible.addAll(entry.getValue());
			visibility.put(entry.getKey(), visible);
		}
		visibility.put(robotPosition, new LinkedHashSet<>());
		visibility.put(targetPosition, new LinkedHashSet<>());
		
		List<Point> pointsToUpdate = new LinkedList<>();
		for (Point p : Arrays.asList(robotPosition, targetPosition)) {
			if (BOUNDARY_SHAPE.contains(p.x, p.y)) {
				pointsToUpdate.add(p);
			} else {
				double closestDistance = Double.MAX_VALUE;
				Point closestStart = null, closestEnd = null;
				for (int i = 0; i < BOUNDARY.length; i++) {
					Point start = BOUNDARY[i];
					Point end = BOUNDARY[(i + 1) % BOUNDARY.length];
					double distance = Line2D.ptSegDist(start.x, start.y, end.x, end.y, p.x, p.y);
					if (distance < closestDistance) {
						closestDistance = distance;
						closestStart = start;
						closestEnd = end;
					}
				}
				double normalX = -(closestEnd.y - closestStart.y);
				double normalY = closestEnd.x - closestStart.x;
				double normalLength = Math.hypot(normalX, normalY);
				double insideX = p.x + normalX / normalLength * (closestDistance + EPSILON);
				double insideY = p.y + normalY / normalLength * (closestDistance + EPSILON);
				Point insidePoint = new Point(insideX, insideY);
				visibility.get(p).add(insidePoint);
				Set<Point> insideVisible = new LinkedHashSet<>();
				insideVisible.add(p);
				visibility.put(insidePoint, insideVisible);
				pointsToUpdate.add(insidePoint);
			}
		}

		for (Point p : pointsToUpdate) {
			for (Point q : visibility.keySet()) {
				if (p.equals(q)) {
					continue;
				}
				boolean hasVisibility = true;
				for (int i = 0; i < BOUNDARY.length; i++) {
					Point e1 = BOUNDARY[i];
					Point e2 = BOUNDARY[(i + 1) % BOUNDARY.length];
					if (q.equals(e1) || q.equals(e2)) {
						continue;
					}
					if (intersects(p, q, e1, e2)) {
						hasVisibility = false;
						break;
					}
				}
				if (hasVisibility) {
					visibility.get(p).add(q);
					visibility.get(q).add(p);
				}
			}
		}
		
		// Compute edges with distances
		Map<Point, Map<Point, Double>> edges = new LinkedHashMap<>();
		for (Map.Entry<Point, Set<Point>> entry : visibility.entrySet()) {
			Map<Point, Double> pEdges = new LinkedHashMap<>();
			Point p = entry.getKey();
			for (Point q : entry.getValue()) {
				pEdges.put(q, Math.hypot((q.x - p.x), (q.y - p.y)));
			}
			edges.put(p, pEdges);
		}
		
		// Calculate shortest path
		Set<Point> unvisited = new LinkedHashSet<>();
		unvisited.addAll(edges.keySet());
		Map<Point, Double> distance = new LinkedHashMap<>();
		for (Point p : edges.keySet()) {
			distance.put(p, Double.POSITIVE_INFINITY);
		}
		distance.put(robotPosition, 0.0);
		Map<Point, Point> previous = new LinkedHashMap<>();
		while (!unvisited.isEmpty()) {
			Point current = Collections.min(unvisited, (Point a, Point b) -> Double.compare(distance.get(a), distance.get(b)));
			unvisited.remove(current);
			for (Map.Entry<Point, Double> neighborEntry : edges.get(current).entrySet()) {
				Point neighbor = neighborEntry.getKey();
				double altDistance = distance.get(current) + neighborEntry.getValue();
				if (altDistance < distance.get(neighbor)) {
					distance.put(neighbor, altDistance);
					previous.put(neighbor, current);
				}
			}
		}
		List<Point> waypoints = new LinkedList<>();
		Point p = targetPosition;
		while (p != null && !p.equals(robotPosition)) {
			waypoints.add(p);
			p = previous.get(p);
		}
		Collections.reverse(waypoints);
		System.out.println(waypoints);
		
		int lastIndex = waypoints.size() - 1;
		Point previousWaypoint = robotPosition;
		double previousHeading = currentHeading;
		for (int i = 0; i < lastIndex; i++) {
			Point waypoint = waypoints.get(i);
			double travelHeading = (Math.toDegrees(Math.atan2(previousWaypoint.y - waypoint.y, waypoint.x - previousWaypoint.x)) + 360.0) % 360.0;
			if (Math.abs(travelHeading - previousHeading) > 90.0) {
				travelHeading = (travelHeading + 180.0) % 360.0;
			}
			previousWaypoint = waypoint;
			previousHeading = travelHeading;
			addSequential(new DriveTo(waypoint.x, waypoint.y, travelHeading, cruiseSpeed, TURN_SLOW_DOWN_RATIO * cruiseSpeed, 5.0, 5.0));
		}
		Point last = waypoints.get(lastIndex);
		addSequential(new DriveTo(last.x, last.y, targetHeading, cruiseSpeed, endSpeed, 3.0, 5.0));
		
		RobotMap.elevator.setHeight(Elevator.CLEAR_OBSTACLE_HEIGHT);
	}
	
	private static final Point[] BOUNDARY = {
		new Point(52.5, 0.0),
		new Point(137.0, 0.0),
		new Point(137.0, 329.44),
		new Point(120.44, 329.44),
		new Point(120.44, 269.09),
		new Point(84.78, 269.09),
		new Point(84.78, 240.84),
		new Point(-84.78, 240.84),
		new Point(-84.78, 269.09),
		new Point(-120.44, 269.09),
		new Point(-120.44, 329.44),
		new Point(-137.0, 329.44),
		new Point(-137.0, 0.0),
		new Point(-52.5, 0.0),
		new Point(-52.5, 114.44),
		new Point(-101.75, 114.44),
		new Point(-101.75, 225.44),
		new Point(101.75, 225.44),
		new Point(101.75, 114.44),
		new Point(52.5, 114.44)
	};

	private static final Map<Point, Set<Point>> BOUNDARY_VISIBILITY = new LinkedHashMap<>();
	private static final Path2D.Double BOUNDARY_SHAPE = new Path2D.Double();
	private static final double EPSILON = 1.0;
	private static final double TURN_SLOW_DOWN_RATIO = 0.6;
	
	static {
		computeBoundaryVisibility();
		BOUNDARY_SHAPE.moveTo(BOUNDARY[0].x, BOUNDARY[0].y);
		for (int i = 1; i < BOUNDARY.length; i++) {
			BOUNDARY_SHAPE.lineTo(BOUNDARY[i].x, BOUNDARY[i].y);
		}
		BOUNDARY_SHAPE.closePath();
	}
	
	private static void computeBoundaryVisibility() {
		int n = BOUNDARY.length;
		for (int ip = 0; ip < n; ip++) {
			Point p = BOUNDARY[ip];
			addVisible(p, BOUNDARY[(ip - 1 + n) % n]);
			addVisible(p, BOUNDARY[(ip + 1) % n]);
			for (int iq = 0; iq < n; iq++) {
				if (ip == (iq - 1 + n) % n || ip == iq || ip == (iq + 1) % n) {
					continue;
				}
				Point q = BOUNDARY[iq];
				boolean hasVisibility = true;
				for (int i = 0; i < n; i++) {
					Point e1 = BOUNDARY[i];
					Point e2 = BOUNDARY[(i + 1) % n];
					if (p.equals(e1) || p.equals(e2) || q.equals(e1) || q.equals(e2)) {
						continue;
					}
					if (intersects(p, q, e1, e2)) {
						hasVisibility = false;
						break;
					}
				}
				if (hasVisibility) {
					Point a = BOUNDARY[(ip - 1 + n) % n];
					Point b = p;
					Point c = BOUNDARY[(ip + 1) % n];
					Point x = q;
					double crossca = cross(b, c, b, a);
					double crosscx = cross(b, c, b, x);
					double crossxa = cross(b, x, b, a);
					if (crossca >= 0.0) {
						if (crosscx >= 0.0 && crossxa >= 0.0) {
							addVisible(p, q);
						}
					} else if (crossca < 0.0) {
						if (!(-crossxa >= 0.0 && -crosscx >= 0.0)) {
							addVisible(p, q);
						}
					}
				}
			}
		}
	}

	private static void addVisible(Point p, Point q) {
		BOUNDARY_VISIBILITY.computeIfAbsent(p, k -> new LinkedHashSet<>()).add(q);
	}
	
	private static double cross(Point a0, Point a1, Point b0, Point b1) {
		return (a1.x - a0.x) * (b1.y - b0.y) - (a1.y - a0.y) * (b1.x - b0.x);
	}
	
	private static boolean intersects(Point a0, Point a1, Point b0, Point b1) {
		return Line2D.linesIntersect(a0.x, a0.y, a1.x, a1.y, b0.x, b0.y, b1.x, b1.y);
	}
	
	private static class Point {
		final double x, y;

		public Point(double x, double y) {
			this.x = x;
			this.y = y;
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			long temp;
			temp = Double.doubleToLongBits(x);
			result = prime * result + (int) (temp ^ (temp >>> 32));
			temp = Double.doubleToLongBits(y);
			result = prime * result + (int) (temp ^ (temp >>> 32));
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			Point other = (Point) obj;
			if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x))
				return false;
			if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return "(" + x + ", " + y + ")";
		}
	}

}
