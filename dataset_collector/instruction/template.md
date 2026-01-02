为了增加指令的多样性和丰富性，我们采用模板槽位填充的技术，我们提供7类核心槽位，每一类指令定义20种模板。

## 核心槽位字典 (Slot Dictionary)
请在生成指令时，从以下选项中随机填充 {} 内容：
- {Landmark}: 入口 (Entrance), 出口 (Exit)
- {Color}: 红色, 蓝色, 白色, 黑色, 银色 ...
- {Side}: 左边, 右边, 前方左边, 前方右边, 左后方, 右后方
- {Order}: 第1个, 第2个, 第3个, 最后一个
- {Action}: 直行 (Go Straight), 左转 (Turn Left), 右转 (Turn Right), 倒退 (Reverse)
- {Distance}: 2米, 5米, 8米 (或其他短距离)
- {Crowding} (映射自然语言):
  - 宽敞的 (对应：两边都没车)
  - 半开放的 (对应：一边没车)
  - 中间的 (对应：两边都有车/拥挤)
  
## 目标约束型
核心逻辑：不限制怎么走，重点在于选择哪个具体的车位（基于颜色、拥挤度、相对方位）。

1.  寻找 **`{Landmark}`** 附近的空闲车位。
2.  停在 **`{Color}`** 车的 **`{Side}`**。 (*例：停在红车的左后方*)
3.  寻找一个 **`{Crowding}`** 车位泊车。 (*例：寻找一个宽敞的车位*)
4.  停在 **`{Color}`** 车和 **`{Color}`** 车 **`{Crowding}`** 车位。 (*例：停在红车和蓝车中间的车位*)
5.  寻找自车 **`{Side}`** 的 **`{Order}`** 空位。 (*例：寻找自车前方右边的第1个空位*)
6.  停在距离 **`{Landmark}`** 最近的一个 **`{Crowding}`** 车位。
7.  请停在 **`{Side}`** 的 **`{Crowding}`** 车位。
8.  寻找 **`{Color}`** 车旁边的 **`{Crowding}`** 车位。
9.  停在这一排 **`{Order}`** 的空位，要求是 **`{Crowding}`**。
10. 寻找 **`{Landmark}`** 对面的空位。
11. 停在 **`{Color}`** 车 **`{Side}`** 的那个空位。
12. 寻找任意一辆 **`{Color}`** 车，停在它 **`{Side}`**。
13. 忽略 **`{Color}`** 车，寻找它旁边的车位。
14. 停在 **`{Side}`** 的 **`{Order}`** 车位。 (*例：停在右后方的最后一个车位*)
15. 寻找一个 **`{Side}`** 没有任何车辆的 **`{Crowding}`** 车位。
16. 停在 **`{Landmark}`** 这一侧的 **`{Order}`** 车位。
17. 寻找两辆 **`{Color}`** 车 **`{Crowding}`** 车位。 (*例：寻找两辆白车中间的车位*)
18. 停在自车 **`{Side}`** 约 3-5 米处的空位。
19. 寻找 **`{Landmark}`** 区域内 **`{Color}`** 车旁边的空位。
20. 停在 **`{Side}`** 一排中 **`{Order}`** 的 **`{Crowding}`** 车位。

## 第二类：过程约束型
核心逻辑：约束过程
1.  先 **`{Action}`** **`{Distance}`**，然后寻找车位。 (*例：先倒退2米，然后找车位*)
2.  向 **`{Landmark}`** 方向 **`{Action}`**，然后泊车。
3.  **`{Action}`** 通过路口后，寻找最近的车位。
4.  先 **`{Action}`**，行驶 **`{Distance}`** 后再寻找车位。
5.  **`{Action}`** 调整车头，然后停在 **`{Side}`** 的空位。
6.  沿着当前道路 **`{Action}`**，直到看到空位。
7.  先 **`{Action}`**，然后 **`{Action}`**，最后泊车。 (*例：先直行，然后左转*)
8.  **`{Action}`** **`{Distance}`**，避开当前区域去别处找。
9.  向 **`{Side}`** **`{Action}`**，寻找视野内的车位。
10. 背对 **`{Landmark}`** **`{Action}`**，寻找车位。
11. 保持 **`{Action}`** 状态 **`{Distance}`**，然后停车。
12. 先 **`{Action}`**，忽略前 **`{Distance}`** 内的车位。
13. **`{Action}`** 绕过障碍物，寻找后方的车位。
14. 靠近 **`{Landmark}`** 后，**`{Action}`** 寻找车位。
15. 先 **`{Action}`** **`{Distance}`**，再 **`{Action}`** **`{Distance}`**。
16. 仅仅 **`{Action}`** **`{Distance}`**，就在附近泊车。
17. 远离 **`{Landmark}`**，**`{Action}`** 行驶一段距离后泊车。
18. 连续 **`{Action}`** 两次（例如连续左转），寻找车位。
19. **`{Action}`** 到道路尽头，然后泊车。
20. 在 **`{Action}`** **`{Distance}`** 的过程中，随时准备泊车。

## 混合约束型
1.  先 **`{Action}`** **`{Distance}`**，停在 **`{Color}`** 车旁边。
2.  向 **`{Landmark}`** **`{Action}`**，寻找一个 **`{Crowding}`** 车位。
3.  **`{Action}`** 后，停在 **`{Side}`** 的 **`{Order}`** 车位。
4.  寻找 **`{Color}`** 车，**`{Action}`** 到它 **`{Side}`** 停下。
5.  先 **`{Action}`**，找到两辆 **`{Color}`** 车 **`{Crowding}`** 车位。
6.  **`{Action}`** **`{Distance}`**，停在 **`{Landmark}`** 附近的空位。
7.  前往 **`{Landmark}`**，停在 **`{Side}`** 的 **`{Color}`** 车旁。
8.  先 **`{Action}`**，然后停在 **`{Crowding}`** 车位里。
9.  **`{Action}`** 调整位置，停在 **`{Color}`** 车的 **`{Side}`**。
10. 寻找 **`{Side}`** 的 **`{Color}`** 车，**`{Action}`** 停在它旁边。
11. **`{Action}`** **`{Distance}`**，停在 **`{Order}`** 的那个空位。
12. 远离 **`{Landmark}`**，**`{Action}`** 寻找 **`{Crowding}`** 车位。
13. 先 **`{Action}`**，如果看到 **`{Color}`** 车就停在它 **`{Side}`**。
14. **`{Action}`** 通过路口，寻找 **`{Side}`** 的 **`{Crowding}`** 车位。
15. 寻找 **`{Color}`** 车，**`{Action}`** **`{Distance}`** 后停在它后面。
16. 先 **`{Action}`**，找到一个 **`{Crowding}`** 车位，停在 **`{Color}`** 车旁。
17. 向 **`{Side}`** **`{Action}`**，停在 **`{Landmark}`** 前面的车位。
18. **`{Action}`** **`{Distance}`**，停在 **{Side}** 的 **{Color}** 车之间。
19. 找到 **`{Order}`** 的空位，**`{Action}`** 调整后泊入。
20. 先 **`{Action}`**，再 **`{Action}`**，最后停在 **`{Color}`** 车 **`{Side}`**。


英文版
---

### 🟢 Core Slot Dictionary (English Version)

Please map your perception/logic outputs to these English strings for the slots:

*   **`{Landmark}`**: *Entrance*, *Exit*
*   **`{Color}`**: *Red*, *Blue*, *White*, *Black*, *Silver*, *Green*
*   **`{Side}`**: *Left*, *Right*, *Front-Left*, *Front-Right*, *Rear-Left*, *Rear-Right*
*   **`{Order}`**: *1st*, *2nd*, *3rd*, *Last*
*   **`{Action}`**: *Go Straight*, *Turn Left*, *Turn Right*, *Reverse*
*   **`{Distance}`**: *2 meters*, *5 meters*, *8 meters*
*   **`{Crowding}`** (Semantics mapping):
    *   *Spacious* (No cars on either side)
    *   *Semi-open* (Car on one side)
    *   *Narrow* (Cars on both sides / Tight)

---

### Category 1: Target Constraints
*Focus: Where to park (Attributes & Relative Position)*

1.  Find an empty spot near the **{Landmark}**.
2.  Park on the **{Side}** of the **{Color}** car.
3.  Find a **{Crowding}** parking spot.
4.  Park in a **{Crowding}** spot between a **{Color}** car and a **{Color}** car.
5.  Find the **{Order}** empty spot on your **{Side}**.
6.  Park in the **{Crowding}** spot closest to the **{Landmark}**.
7.  Please park in a **{Crowding}** spot on the **{Side}**.
8.  Find a **{Crowding}** spot next to a **{Color}** car.
9.  Park in the **{Order}** vacancy in this row, make sure it is **{Crowding}**.
10. Find an empty spot opposite the **{Landmark}**.
11. Park in the spot to the **{Side}** of the **{Color}** car.
12. Find any **{Color}** car and park on its **{Side}**.
13. Ignore the **{Color}** car and find a spot next to it.
14. Park in the **{Order}** spot on the **{Side}**.
15. Find a **{Crowding}** spot on the **{Side}** with no adjacent vehicles.
16. Park in the **{Order}** spot on the side of the **{Landmark}**.
17. Find a **{Crowding}** spot between two **{Color}** cars.
18. Park in a spot about 3-5 meters to your **{Side}**.
19. Find a spot next to a **{Color}** car in the **{Landmark}** area.
20. Park in the **{Order}** **{Crowding}** spot in the **{Side}** row.

---

### Category 2: Process Constraints
*Focus: How to move (Actions, Distances, Adjustments)*

1.  First **{Action}** for **{Distance}**, then look for a spot.
2.  **{Action}** towards the **{Landmark}**, then park.
3.  After **{Action}** through the intersection, find the nearest spot.
4.  First **{Action}**, drive for **{Distance}**, then look for a spot.
5.  **{Action}** to adjust alignment, then park in a spot on the **{Side}**.
6.  **{Action}** along the current road until you see an empty spot.
7.  First **{Action}**, then **{Action}**, and finally park.
8.  **{Action}** for **{Distance}** to leave the current area, then look elsewhere.
9.  **{Action}** to the **{Side}** and look for a spot in view.
10. **{Action}** away from the **{Landmark}** and look for a spot.
11. Keep **{Action}** for **{Distance}**, then stop and park.
12. First **{Action}**, ignoring spots within the first **{Distance}**.
13. **{Action}** to avoid obstacles and find a spot behind.
14. Approach the **{Landmark}**, then **{Action}** to find a spot.
15. First **{Action}** for **{Distance}**, then **{Action}** for **{Distance}**.
16. Just **{Action}** for **{Distance}** and park nearby.
17. Drive away from the **{Landmark}**, **{Action}** for a distance, then park.
18. **{Action}** twice consecutively, then find a spot.
19. **{Action}** to the end of the road, then park.
20. Be ready to park while **{Action}** for **{Distance}**.

---

### Category 3: Hybrid Constraints
*Focus: Complex combination of Motion and Target*

1.  First **{Action}** for **{Distance}**, then park next to a **{Color}** car.
2.  **{Action}** towards the **{Landmark}** and find a **{Crowding}** spot.
3.  After **{Action}**, park in the **{Order}** spot on the **{Side}**.
4.  Find a **{Color}** car, **{Action}** to its **{Side}**, and stop.
5.  First **{Action}**, find a **{Crowding}** spot between two **{Color}** cars.
6.  **{Action}** for **{Distance}** and park in a spot near the **{Landmark}**.
7.  Go to the **{Landmark}** and park next to a **{Color}** car on the **{Side}**.
8.  First **{Action}**, then park in a **{Crowding}** spot.
9.  **{Action}** to align, then park on the **{Side}** of the **{Color}** car.
10. Find a **{Color}** car on the **{Side}**, **{Action}**, and park next to it.
11. **{Action}** for **{Distance}** and park in the **{Order}** empty spot.
12. Move away from the **{Landmark}**, **{Action}**, and find a **{Crowding}** spot.
13. First **{Action}**, and if you see a **{Color}** car, park on its **{Side}**.
14. **{Action}** through the intersection and find a **{Crowding}** spot on the **{Side}**.
15. Find a **{Color}** car, **{Action}** for **{Distance}**, and park behind it.
16. First **{Action}**, find a **{Crowding}** spot, and park next to a **{Color}** car.
17. **{Action}** to the **{Side}** and park in front of the **{Landmark}**.
18. **{Action}** for **{Distance}** and park between **{Color}** cars on the **{Side}**.
19. Find the **{Order}** spot, **{Action}** to adjust, and park.
20. First **{Action}**, then **{Action}**, and finally park on the **{Side}** of the **{Color}** car.