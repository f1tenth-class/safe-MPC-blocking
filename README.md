# Lightning MPCQueen - Safe MPC Blocking Strategy in F1/10 Autonomous Racing

This work introduces a model predictive control (MPC) based blocking mechanism, and contrasts its performance with an MPC system that strictly adheres to the optimal racing line. Experiments are conducted under two scenarios: one without active blocking and another implementing the proposed blocking strategy. Inspired by game theoretical approaches, the work diverges by manually controlling the overtaking car, thus simulating varied real-world racing behaviors. The results offer insights into the effectiveness of the blocking strategy in allowing a handicapped blocking car (with a handicap ratio of up to 5.0) to defeat the overtaking car.

## Blocking MPC
<img src="images/Blocking MPC Diagram.jpg" width="640" height="360">

The figure above shows the idea behind blocking MPC. The blocking car projects the trajectory of the overtaking car onto the road and uses MPC to track it. This occurs in real time and therefore prevents the opponent car from overtaking the blocking car. 

## Setup

<img src="images/Starting configuration.png" width="400" height="400">

The setup of the experiment is as above. The overtaking car is started 1 meter behind the blocking car on a different track. Once the program is turned on, the blocking car will attempt to track the projected trajectory (shown in green) of the overtaking car.

## Results

(below videos take you to YouTube)

### Manual overtake vs Blocking MPC - 0.66m/s

<div align="center">
  <a href="https://www.youtube.com/watch?v=DM6ZaSpqUa0"><img src="https://img.youtube.com/vi/DM6ZaSpqUa0/0.jpg" alt="Blocking MPC 0.6"></a>
</div>

In the video above, we see the overtaking car in blue, driven manually by a driver, attempt to overtake the autonomous blocking car in orange. The blocking car is driving at 0.5 m/s and the overtaking car is driving at 0.66 m/s.

### Manual overtake vs Blocking MPC - 2.68m/s

<div align="center">
  <a href="https://www.youtube.com/watch?v=5MUvOnlYGE4"><img src="https://img.youtube.com/vi/5MUvOnlYGE4/0.jpg" alt="Blocking MPC 2.6"></a>
</div>

In the video above, the overtaking car in blue, driven manually by a driver, attempt to overtake the autonomous blocking car in orange and succeeds. The blocking car is driving at 0.5 m/s and the overtaking car is driving at 2.68 m/s. The sheer speed of the overtaking car allows it to defeat the blocking car which is constrained in its blocking due to safety concerns.




