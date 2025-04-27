package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

/**
 * Node connection matrix class, used to represent connection relationships
 * between nodes. -1 indicates no connection, other values indicate connection
 * distance
 */
public class NodeConnectionMatrix {
  private final double[][] connectionMatrix;
  private final int nodeCount;
  private final Pose2d[] nodePoses;

  /**
   * Constructor
   *
   * @param nodeCount Number of nodes
   */
  public NodeConnectionMatrix(int nodeCount) {
    this.nodeCount = nodeCount;
    this.connectionMatrix = new double[nodeCount][nodeCount];
    this.nodePoses = new Pose2d[nodeCount];
    // Initialize all connections to -1 (no connection)
    for (int i = 0; i < nodeCount; i++) {
      for (int j = 0; j < nodeCount; j++) {
        connectionMatrix[i][j] = -1;
      }
    }
  }

  public NodeConnectionMatrix(Pose2d[] nodePoses) {
    this.nodePoses = nodePoses;
    this.nodeCount = nodePoses.length;
    this.connectionMatrix = new double[nodeCount][nodeCount];
    for (int i = 0; i < nodeCount; i++) {
      for (int j = 0; j < nodeCount; j++) {
        connectionMatrix[i][j] = -1;
      }
    }
  }

  /**
   * Set connection between two nodes
   *
   * @param node1    First node index
   * @param node2    Second node index
   * @param distance Connection distance, -1 indicates no connection
   */
  public void setConnection(int node1, int node2, double distance) {
    if (node1 >= 0 && node1 < nodeCount && node2 >= 0 && node2 < nodeCount) {
      connectionMatrix[node1][node2] = distance;
      connectionMatrix[node2][node1] = distance; // Undirected graph, bidirectional connection
    }
  }

  public void setConnection(int node1, int node2) {
    if (node1 >= 0 && node1 < nodeCount && node2 >= 0 && node2 < nodeCount) {
      connectionMatrix[node1][node2] = nodePoses[node1].getTranslation().getDistance(nodePoses[node2].getTranslation());
      connectionMatrix[node2][node1] = nodePoses[node1].getTranslation().getDistance(nodePoses[node2].getTranslation());
    }
  }

  /**
   * Set node position
   *
   * @param node Node index
   * @param pose Node position
   */
  public void setNodePose(int node, Pose2d pose) {
    if (node >= 0 && node < nodeCount) {
      nodePoses[node] = pose;
    }
  }

  /**
   * Get node position
   *
   * @param node Node index
   * @return Node position, returns null if not set
   */
  public Pose2d getNodePose(int node) {
    return nodePoses[node];
  }

  /**
   * Get connection distance between two nodes
   *
   * @param node1 First node index
   * @param node2 Second node index
   * @return Connection distance, -1 indicates no connection
   */
  public double getConnection(int node1, int node2) {
    if (node1 >= 0 && node1 < nodeCount && node2 >= 0 && node2 < nodeCount) {
      return connectionMatrix[node1][node2];
    }
    return -1;
  }

  public int countConnections() {
    int count = 0;
    for (int i = 0; i < nodeCount; i++) {
      for (int j = 0; j < nodeCount; j++) {
        if (isConnected(i, j)) {
          count++;
        }
      }
    }
    return count;
  }

  /**
   * Check if two nodes are connected
   *
   * @param node1 First node index
   * @param node2 Second node index
   * @return Whether connected
   */
  public boolean isConnected(int node1, int node2) {
    return getConnection(node1, node2) >= 0;
  }

  /**
   * Get number of nodes
   *
   * @return Number of nodes
   */
  public int getNodeCount() {
    return nodeCount;
  }

  /**
   * Get connection matrix
   *
   * @return Connection matrix
   */
  public double[][] getConnectionMatrix() {
    return connectionMatrix;
  }

  /**
   * Get positions of all nodes
   *
   * @return Node position mapping
   */
  public Pose2d[] getNodePoses() {
    return nodePoses;
  }

  public Pose2d[] getShortestPath(
      Supplier<Pose2d> currentPoseSupplier, Supplier<Pose2d> targetPoseSupplier) {
    // First find the nearest nodes
    int startNode = getNearestNode(currentPoseSupplier.get());
    int endNode = getNearestNode(targetPoseSupplier.get());

    // if (startNode == endNode) {
    // return new Pose2d[] { getNodePose(endNode), targetPose };
    // }

    // Dijkstra's algorithm implementation
    double[] distances = new double[nodeCount];
    int[] previous = new int[nodeCount];
    boolean[] visited = new boolean[nodeCount];

    // Initialize distances
    for (int i = 0; i < nodeCount; i++) {
      distances[i] = Double.MAX_VALUE;
      previous[i] = -1;
    }
    distances[startNode] = 0;

    for (int i = 0; i < nodeCount; i++) {
      // Find the unvisited node with the smallest distance
      int currentNode = -1;
      double smallestDistance = Double.MAX_VALUE;
      for (int j = 0; j < nodeCount; j++) {
        if (!visited[j] && distances[j] < smallestDistance) {
          smallestDistance = distances[j];
          currentNode = j;
        }
      }

      // If we've visited all nodes or can't reach the remaining ones
      if (currentNode == -1) {
        break;
      }

      visited[currentNode] = true;

      // If we've reached the target node
      if (currentNode == endNode) {
        break;
      }

      // Update distances to neighbors
      for (int neighbor = 0; neighbor < nodeCount; neighbor++) {
        if (isConnected(currentNode, neighbor)) {
          double edgeDistance = getConnection(currentNode, neighbor);
          double totalDistance = distances[currentNode] + edgeDistance;

          if (totalDistance < distances[neighbor]) {
            distances[neighbor] = totalDistance;
            previous[neighbor] = currentNode;
          }
        }
      }
    }

    // Reconstruct the path
    List<Integer> path = new ArrayList<>();
    int currentNode = endNode;

    while (currentNode != -1) {
      path.add(currentNode);
      currentNode = previous[currentNode];
    }

    Collections.reverse(path);

    // Convert node indices to poses
    Pose2d[] posePath = new Pose2d[path.size() + 2];
    posePath[0] = currentPoseSupplier.get();
    for (int i = 0; i < path.size(); i++) {
      posePath[i + 1] = getNodePose(path.get(i));
    }
    // Add the target pose at the end
    posePath[path.size() + 1] = targetPoseSupplier.get();
    return posePath;
  }

  public int getNearestNode(Pose2d pose) {
    // 检查输入 pose 是否有效
    if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())) {
      throw new IllegalArgumentException("Invalid pose: contains NaN values");
    }

    double minDistance = Double.MAX_VALUE;
    int nearestNode = -1;
    for (int i = 0; i < nodeCount; i++) {
      double distance = pose.getTranslation().getDistance(nodePoses[i].getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        nearestNode = i;
      }
    }
    return nearestNode;
  }
}
