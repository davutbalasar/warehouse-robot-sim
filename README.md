# warehouse-robot-sim
A ROS2 simulation of a Roomba like warehouse robot that patrols a corridor and collects trash from sections. Utilizes pygame for 2D and RViz2 for 3D visualization.

<a href="https://www.notion.so/Windows-zerinde-WSL-de-ROS-2-kurulumu-ve-kullan-m-3173a84cb5d180bc84e8fb37c6d04953" target="_blank"><b>A comprehensive guide in Turkish</b></a> created by Davut, Büşra and Fatih for setting up the project environment.

<br>
<br>

<h2>Architecture</h2>

<table>
  <thead>
    <tr>
      <th>Node</th>
      <th>File</th>
      <th>Role</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><b>environment_node</b></td>
      <td><code>environment_node.py</code></td>
      <td>Manages world state, spawns trash at random shelf positions</td>
    </tr>
    <tr>
      <td><b>robot_node</b></td>
      <td><code>robot_node.py</code></td>
      <td>FSM logic, wall-constrained movement, occlusion-aware vision, trash pickup</td>
    </tr>
    <tr>
      <td><b>visualizer_node</b></td>
      <td><code>visualizer_node.py</code></td>
      <td>Real-time 2D rendering with pygame</td>
    </tr>
    <tr>
      <td><b>rviz_publisher_node</b></td>
      <td><code>rviz_publisher_node.py</code></td>
      <td>Publishes RViz2 markers for 3D visualization</td>
    </tr>
  </tbody>
</table>

<br>

<h4>Robot FSM</h4>

<p>
  <code>patrol</code> &nbsp;→&nbsp;
  <code>go_to_trash</code> &nbsp;→&nbsp;
  <code>return</code> &nbsp;→&nbsp;
  <code>patrol</code>
</p>

<br>
<br>

<div align="center">
  <div>
    <img width="694" height="696" alt="trash_2d" src="https://github.com/user-attachments/assets/84eb7d07-9274-4258-8f0a-c5dfe8760e64" />
  </div>
  <p style="margin-top: 10px;">
    <i>2D Visualization using pygame</i>
  </p>
</div>

<br>
<br>

<div align="center">
  <div>
    <img width="800" height="465" alt="trash_3d" src="https://github.com/user-attachments/assets/0c175c1f-9fdd-4752-b58a-bc85a117db05" />
  </div>
  <p style="margin-top: 10px;">
    <i>3D Visualization using RViz2</i>
  </p>
</div>

<br>
<br>



