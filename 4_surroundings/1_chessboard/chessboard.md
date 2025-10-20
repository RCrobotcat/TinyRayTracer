```c++
bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N,
                     Material &material)
{
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) // 找到更近的交点 相当于深度测试
        {
            spheres_dist = dist_i;
            hit = orig + dir * dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }

    // chess board plane
    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(dir.y) > 1e-3)
    {
        float d = -(orig.y + 4) / dir.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + dir * d;
        if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheres_dist)
        {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0, 1, 0);
            material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? Vec3f(1, 1, 1) : Vec3f(1, .7, .3);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }
    return std::min(spheres_dist, checkerboard_dist) < 1000;
}

```
---

## 一、代码位置

棋盘逻辑出现在 `scene_intersect()` 函数的后半部分：

```cpp
// chess board plane
float checkerboard_dist = std::numeric_limits<float>::max();
if (fabs(dir.y) > 1e-3)
{
    float d = -(orig.y + 4) / dir.y; // the checkerboard plane has equation y = -4
    Vec3f pt = orig + dir * d;
    if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheres_dist)
    {
        checkerboard_dist = d;
        hit = pt;
        N = Vec3f(0, 1, 0);
        material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ?
                                 Vec3f(1, 1, 1) : Vec3f(1, .7, .3);
        material.diffuse_color = material.diffuse_color * .3;
    }
}
```

---

## 二、逻辑整体思路

这部分代码模拟了一个**水平放置的地板**，在世界坐标中位于 `y = -4`。
光线射入场景时，除了可能击中球体（`spheres`）之外，还可能与这个平面相交。

该平面带有“棋盘格”纹理：

* 不同格子交替显示两种颜色（白色和棕色）。
* 平面上还有限制，只绘制 `x ∈ [-10, 10]`, `z ∈ [-30, -10]` 的矩形区域。

---

##  三、数学部分：光线与平面的求交

平面方程为：
$$
y = -4
$$
光线参数方程为：
$$
P(t) = O + tD
$$
其中：

* ($O$)：光线起点 (`orig`)
* ($D$)：光线方向 (`dir`)
* ($t$)：参数（沿光线的距离）

要找到交点，我们需要求：
$$
O_y + tD_y = -4
$$
解出：
$$
t = \frac{-(O_y + 4)}{D_y}
$$
对应到代码：

```cpp
float d = -(orig.y + 4) / dir.y;
```

其中：

* `d` 就是参数 (t)，表示光线从相机到地面的距离；
* 要求 `fabs(dir.y) > 1e-3` 避免除以零（当光线几乎平行于地面时，不计算交点）。

---

## 四、交点位置与可见性判定

```cpp
Vec3f pt = orig + dir * d;
if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheres_dist)
```

解释：

* `d > 0`：交点要在光线前方（$t>0$）。
* `fabs(pt.x) < 10`：棋盘横向边界，x 方向从 -10 到 10。
* `pt.z < -10 && pt.z > -30`：棋盘纵向边界，z 方向从 -30 到 -10。
* `d < spheres_dist`：若棋盘比球体更近，则优先使用地面交点。

所以这一行保证了：
✅ 光线打在地板区域的矩形范围内
✅ 且距离比场景中其他球体更近
→ 那么交点就是在地板上。

---

## 五、棋盘格颜色生成逻辑

```cpp
material.diffuse_color =
   (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ?
   Vec3f(1, 1, 1) : Vec3f(1, .7, .3);
```

这行是核心：用**位运算 + int 强制取整**来实现黑白格交替。

具体思路：

1. 对 `x` 和 `z` 取 0.5 缩放：

   ```cpp
   int(.5 * hit.x + 1000) + int(.5 * hit.z)
   ```

    * `hit.x` 和 `hit.z` 是当前交点的坐标；
    * 乘 0.5 相当于控制格子的大小；
    * 加上 1000 是为了防止负数参与取整（仅用于偏移，不影响周期性）。

2. 再用 `& 1` 取最低位：

    * 偶数格：结果为 0；
    * 奇数格：结果为 1；
    * 这样就能在空间上交替变化，形成“棋盘格”图案。

3. 不同结果对应不同颜色：

    * `true` → 白色 `Vec3f(1, 1, 1)`
    * `false` → 棕色 `Vec3f(1, .7, .3)`

4. 最后整体暗化（调低亮度）：

   ```cpp
   material.diffuse_color = material.diffuse_color * .3;
   ```

---

## 六、法线与材质设置

```cpp
N = Vec3f(0, 1, 0);
```

* 平面的法线方向固定向上（y 正方向），
* 所以所有在地板上的光照计算都用这个法线。

`Material` 只设置了漫反射颜色（`diffuse_color`），
没有镜面反射或折射，所以棋盘只是一个**漫反射表面**。

---

## 七、视觉效果总结

最终渲染中：

* 球体漂浮在一个棋盘格地板上；
* 棋盘会根据光源产生阴影；
* 镜面球能反射出地板；
* 玻璃球能通过折射看到下面的棋盘；
* 棋盘自身不会反射或折射，只参与漫反射和阴影。

---

## 八、补充说明：为什么不单独建一个平面类？

因为平面很简单，只需要一个方程和固定法线，
而球体要完整的射线相交函数；
作者直接在 `scene_intersect()` 里硬编码了这个平面，避免创建额外的类，
让演示更加紧凑。

---

## ✅ 小结

| 步骤 | 功能            | 对应代码                                    |
| -- | ------------- | --------------------------------------- |
| 1  | 定义平面 `y = -4` | `float d = -(orig.y + 4)/dir.y`         |
| 2  | 判断是否命中矩形区域    | `fabs(pt.x)<10 && pt.z<-10 && pt.z>-30` |
| 3  | 检查是否比球体更近     | `d < spheres_dist`                      |
| 4  | 设置法线方向        | `N = Vec3f(0,1,0)`                      |
| 5  | 根据 (x,z) 交替上色 | `(int(.5*hit.x+1000)+int(.5*hit.z))&1`  |
| 6  | 调暗颜色          | `*0.3`                                  |
