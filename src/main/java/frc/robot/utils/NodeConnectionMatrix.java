package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** 节点连接矩阵类，用于表示节点之间的连接关系 -1表示不连接，其他数值表示连接的距离 */
public class NodeConnectionMatrix {
  private final double[][] connectionMatrix;
  private final int nodeCount;
  private final Pose2d[] nodePoses;

  /**
   * 构造函数
   *
   * @param nodeCount 节点数量
   */
  public NodeConnectionMatrix(int nodeCount) {
    this.nodeCount = nodeCount;
    this.connectionMatrix = new double[nodeCount][nodeCount];
    this.nodePoses = new Pose2d[nodeCount];
    // 初始化所有连接为-1（不连接）
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
   * 设置两个节点之间的连接
   *
   * @param node1 第一个节点索引
   * @param node2 第二个节点索引
   * @param distance 连接距离，-1表示不连接
   */
  public void setConnection(int node1, int node2, double distance) {
    if (node1 >= 0 && node1 < nodeCount && node2 >= 0 && node2 < nodeCount) {
      connectionMatrix[node1][node2] = distance;
      connectionMatrix[node2][node1] = distance; // 无向图，双向连接
    }
  }

  public void setConnection(int node1, int node2) {
    if (node1 >= 0 && node1 < nodeCount && node2 >= 0 && node2 < nodeCount) {
      connectionMatrix[node1][node2] =
          nodePoses[node1].getTranslation().getDistance(nodePoses[node2].getTranslation());
      connectionMatrix[node2][node1] =
          nodePoses[node1].getTranslation().getDistance(nodePoses[node2].getTranslation());
    }
  }

  /**
   * 设置节点的位置
   *
   * @param node 节点索引
   * @param pose 节点位置
   */
  public void setNodePose(int node, Pose2d pose) {
    if (node >= 0 && node < nodeCount) {
      nodePoses[node] = pose;
    }
  }

  /**
   * 获取节点的位置
   *
   * @param node 节点索引
   * @return 节点位置，如果未设置则返回null
   */
  public Pose2d getNodePose(int node) {
    return nodePoses[node];
  }

  /**
   * 获取两个节点之间的连接距离
   *
   * @param node1 第一个节点索引
   * @param node2 第二个节点索引
   * @return 连接距离，-1表示不连接
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
   * 检查两个节点是否连接
   *
   * @param node1 第一个节点索引
   * @param node2 第二个节点索引
   * @return 是否连接
   */
  public boolean isConnected(int node1, int node2) {
    return getConnection(node1, node2) >= 0;
  }

  /**
   * 获取节点数量
   *
   * @return 节点数量
   */
  public int getNodeCount() {
    return nodeCount;
  }

  /**
   * 获取连接矩阵
   *
   * @return 连接矩阵
   */
  public double[][] getConnectionMatrix() {
    return connectionMatrix;
  }

  /**
   * 获取所有节点的位置
   *
   * @return 节点位置映射
   */
  public Pose2d[] getNodePoses() {
    return nodePoses;
  }

  public Pose2d[] getShortestPath(Pose2d currentPose, Pose2d targetPose) {
    // First find the nearest nodes
    int startNode = getNearestNode(currentPose);
    int endNode = getNearestNode(targetPose);

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
    posePath[0] = currentPose;
    for (int i = 0; i < path.size(); i++) {
      posePath[i + 1] = getNodePose(path.get(i));
    }
    // Add the target pose at the end
    posePath[path.size() + 1] = targetPose;
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
