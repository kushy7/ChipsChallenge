package edu.ncsu.csc411.ps06.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Stack;

import edu.ncsu.csc411.ps06.environment.Action;
import edu.ncsu.csc411.ps06.environment.Environment;
import edu.ncsu.csc411.ps06.environment.Position;
import edu.ncsu.csc411.ps06.environment.Tile;
import edu.ncsu.csc411.ps06.environment.TileStatus;


public class Robot {

	private final Environment env;
	private final Map<TileStatus, ArrayList<Position>> envPositions;
	private ArrayList<Position> chips;
	private Map<TileStatus, ArrayList<Position>> keys;
	private PriorityQueue<Node> frontier;
	private Map<Position, Position> cameFrom;
	private Map<Position, Integer> costSoFar;
	private Set<TileStatus> collectedKeys;
	private boolean mapLoaded = false;

	public Robot(final Environment env) {
		this.env = env;
		this.envPositions = env.getEnvironmentPositions();
		this.chips = envPositions.get(TileStatus.CHIP);
		this.keys = new HashMap<>();
		this.collectedKeys = new HashSet<>();
		initializeKeys();
	}

	private void initializeKeys() {
		if (!mapLoaded) {
			for (final TileStatus keyColor : TileStatus.values()) {
				if (keyColor.name().startsWith("KEY_")) {
					final ArrayList<Position> keyPositions = envPositions.get(keyColor);
					if (keyPositions != null) {
						keys.put(keyColor, keyPositions);
					}
				}
			}
			mapLoaded = true;
		}
	}

	/**
	 * In my navigation strategy, I incorporated the A* pathfinding algorithm
	 * alongside logical decision-making related to the game's chips and key
	 * dynamics. Initially, the main challenge I encountered was navigating through
	 * doors locked by color-specific keys, which necessitated the prioritization of
	 * collecting keys before gathering chips. With all keys collected, all doors
	 * become accessible, removing any obstacles to chip collection. The A*
	 * algorithm was employed to efficiently locate and travel to the nearest key.
	 * This procedure was repeated until all keys were secured. Subsequently, the
	 * robot would switch to finding and reaching the nearest chips, again utilizing
	 * the A* algorithm for optimal path calculation. After gathering all chips, the
	 * robot employs the A* algorithm once more to navigate towards the goal. With
	 * all keys and chips collected, the path to the goal is clear, and the A*
	 * algorithm efficiently navigates past environmental obstacles like walls and
	 * water bodies.
	 */
	public Action getAction() {
		final Position selfPos = env.getRobotPosition(this);
		final Position goal = envPositions.get(TileStatus.GOAL).get(0);

		if (!allKeysCollected()) {
			TileStatus nearestKeyColor = findNearestKeyColor(selfPos);
			if (nearestKeyColor != null) {
				ArrayList<Position> keyPositions = keys.get(nearestKeyColor);
				return processTarget(selfPos, findNearestPosition(keyPositions), keyPositions, nearestKeyColor);
			}
		}

		if (!chips.isEmpty()) {
			return processTarget(selfPos, findNearestPosition(chips), chips, null);
		} else {
			return navigateToGoal(selfPos, goal);
		}
	}

	private boolean allKeysCollected() {
		return keys.values().stream().allMatch(ArrayList::isEmpty);
	}

	private TileStatus findNearestKeyColor(Position currentPos) {
		TileStatus nearestKeyColor = null;
		int minDistance = Integer.MAX_VALUE;

		for (Map.Entry<TileStatus, ArrayList<Position>> entry : keys.entrySet()) {
			for (Position keyPos : entry.getValue()) {
				int distance = heuristic(currentPos, keyPos);
				if (distance < minDistance) {
					minDistance = distance;
					nearestKeyColor = entry.getKey();
				}
			}
		}

		return nearestKeyColor;
	}

	private Action processTarget(Position currentPos, Position targetPos, ArrayList<Position> objectsList,
			TileStatus keyColor) {
		if (currentPos.equals(targetPos)) {
			if (keyColor != null) {
				collectedKeys.add(keyColor);
				objectsList.remove(targetPos);
			} else {
				chips.remove(targetPos);
			}
			return Action.DO_NOTHING;
		} else {
			return moveToPosition(currentPos, targetPos);
		}
	}

	private Action moveToPosition(Position start, Position end) {
		setupPathfinding(start);
		if (!calculatePath(end)) {
			return Action.DO_NOTHING;
		}
		return followPath(start, end);
	}

	private void setupPathfinding(Position start) {
		frontier = new PriorityQueue<>();
		frontier.add(new Node(start, 0));
		cameFrom = new HashMap<>();
		costSoFar = new HashMap<>();
		cameFrom.put(start, null);
		costSoFar.put(start, 0);
	}

	private boolean calculatePath(Position goal) {
		final Map<Position, Tile> positions = env.getTiles();

		while (!frontier.isEmpty()) {
			Node current = frontier.poll();
			if (current.getPosition().equals(goal)) {
				return true;
			}
			exploreNeighbors(current, positions, goal);
		}
		return false;
	}

	private void exploreNeighbors(Node current, Map<Position, Tile> positions, Position goal) {
		for (Entry<String, Position> next : env.getNeighborPositions(current.getPosition()).entrySet()) {
			Tile nextTile = positions.get(next.getValue());
			if (nextTile.getStatus() != TileStatus.WALL && nextTile.getStatus() != TileStatus.WATER) {
				int newCost = costSoFar.get(current.getPosition()) + 1;
				if (!costSoFar.containsKey(next.getValue()) || newCost < costSoFar.get(next.getValue())) {
					costSoFar.put(next.getValue(), newCost);
					int priority = newCost + heuristic(goal, next.getValue());
					frontier.add(new Node(next.getValue(), priority));
					cameFrom.put(next.getValue(), current.getPosition());
				}
			}
		}
	}

	private Action navigateToGoal(Position currentPos, Position goal) {
		if (currentPos.equals(goal)) {
			return Action.DO_NOTHING; // Already at goal
		} else {
			return moveToPosition(currentPos, goal); // Move towards the goal
		}
	}

	private Action followPath(Position start, Position goal) {
		Position current = goal;
		Stack<Position> path = new Stack<>();
		while (current != null && !current.equals(start)) {
			path.push(current);
			current = cameFrom.get(current);
		}

		return nextMoveFromPath(start, path);
	}

	private Action nextMoveFromPath(Position currentPos, Stack<Position> path) {
		Map<String, Position> neighbors = env.getNeighborPositions(currentPos);
		Position nextPos = path.empty() ? currentPos : path.peek();
		if (nextPos.equals(neighbors.get("above")))
			return Action.MOVE_UP;
		if (nextPos.equals(neighbors.get("below")))
			return Action.MOVE_DOWN;
		if (nextPos.equals(neighbors.get("left")))
			return Action.MOVE_LEFT;
		if (nextPos.equals(neighbors.get("right")))
			return Action.MOVE_RIGHT;
		return Action.DO_NOTHING;
	}

	private Position findNearestPosition(final ArrayList<Position> positionsList) {
		final Position currentPos = env.getRobotPosition(this);
		Position nearest = null;
		int minDistance = Integer.MAX_VALUE;

		for (final Position pos : positionsList) {
			int distance = heuristic(currentPos, pos);
			if (distance < minDistance) {
				minDistance = distance;
				nearest = pos;
			}
		}

		return nearest;
	}

	private int heuristic(final Position a, final Position b) {
		return Math.abs(a.getCol() - b.getCol()) + Math.abs(a.getRow() - b.getRow());
	}

	class Node implements Comparable<Node> {
		Position pos;
		int distance;

		public Node(final Position pos, final int distance) {
			this.pos = pos;
			this.distance = distance;
		}

		public Position getPosition() {
			return pos;
		}

		public int getDistance() {
			return distance;
		}

		@Override
		public int compareTo(final Node other) {
			return Integer.compare(this.distance, other.distance);
		}
	}
}
