# 地图解析引擎
这里实现了简单的全局路径解析引擎。xml格式定义了车辆的行驶路线。

```xml
<RoadNet>
    <!-- 定义路径点，作为轨迹点，也就是道路中心点 -->
    <nodes>
        <!-- 属性 p="x y z theta" (单位: 米, 米, 米, 角度-180到180度) -->
        <n id="1" p="286.64 191.83 0.25 90.42" />
        <n id="2" p="286.52 209.31 0.32 90.44" />

    </nodes>

    <!-- 定义道路，道路只有3种类型：直行、左转、右转 -->
    <ways>
        <!-- type 枚举: Straight, LeftTurn, RightTurn -->
        <w id="101" nodes="1 2 3" type="Straight" />
        <w id="102" nodes="3 4 5" type="LeftTurn" />
        <w id="103" nodes="5 6" type="Straight" />
    </ways>

    <!-- 定义道路的连接关系，用来构建图Graph -->
    <relations>
        <r from="101" to="102" />
    </relations>
</RoadNet>

```
地图引擎通过relation建立道路连接的topo图，节点和边。计算两点之间的可达性首先通过道路way的图结构进行计算，然后再计算way内部的路径。