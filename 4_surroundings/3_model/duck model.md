```c++
// models
float models_dist = std::numeric_limits<float>::max();
for (size_t i = 0; i < models.size(); i++)
{
    float dist_i, u, v;
    if (duck.ray_triangle_intersect(i, orig, dir, dist_i) && dist_i < spheres_dist && dist_i < checkerboard_dist
        && dist_i < models_dist)
    {
        models_dist = dist_i;
        hit = orig + dir * dist_i;
        Vec3f v0v1 = models[i][1] - models[i][0];
        Vec3f v0v2 = models[i][2] - models[i][0];
        N = cross(v0v1, v0v2).normalize();
        material = models[i].material;
    }
}

return std::min(std::min(spheres_dist, checkerboard_dist), models_dist) < 1000;
    
```
---
```c++
// Moller and Trumbore
bool Model::ray_triangle_intersect(const int &fi, const Vec3f &orig, const Vec3f &dir, float &tnear) {
    Vec3f edge1 = point(vert(fi, 1)) - point(vert(fi, 0));
    Vec3f edge2 = point(vert(fi, 2)) - point(vert(fi, 0));
    Vec3f pvec = cross(dir, edge2);
    float det = edge1 * pvec;
    if (det < 1e-5) return false;

    Vec3f tvec = orig - point(vert(fi, 0));
    float u = tvec * pvec;
    if (u < 0 || u > det) return false;

    Vec3f qvec = cross(tvec, edge1);
    float v = dir * qvec;
    if (v < 0 || u + v > det) return false;

    tnear = edge2 * qvec * (1. / det);
    return tnear > 1e-5;
}

```
---
非常好，这段代码是一个**支持折射、反射、环境贴图与OBJ模型加载（duck.obj）**的简易**光线追踪器（ray tracer）**。
我来分成三部分详细解释你问的内容：
-----------------

## 一、折射（`refract()`）的逻辑解析

###  1. 折射公式来源：Snell’s Law（斯涅尔定律）

当光线从一种介质（折射率 $n_1$）进入另一种介质（折射率 $n_2$）时，有：
$$
n_1 \sin \theta_1 = n_2 \sin \theta_2
$$
程序用向量形式表示，计算**折射方向向量**：

$$
\mathbf{T} = \eta \mathbf{I} + (\eta \cos i - \sqrt{1 - \eta^2 (1 - \cos^2 i)}) \mathbf{N}
$$
其中：

* $\mathbf{I}$：入射方向（normalize() 后）；
* $\mathbf{N}$：法线；
* $\eta = n_1 / n_2$；
* $\cos i = -(\mathbf{I} \cdot \mathbf{N})$；
* $k=1 - \eta^2 (1 - \cos^2 i)$：若根号内 $k < 0$，则发生 **全反射（Total Internal Reflection）**。

###  2. 代码实现解读

```cpp
Vec3f refract(const Vec3f &I, const Vec3f &N, const float &refractive_index)
{
    float cosi = -std::max(-1.f, std::min(1.f, I * N)); // 入射角余弦
    float etai = 1, etat = refractive_index; // 空气 -> 物体
    Vec3f n = N;
    if (cosi < 0) {
        cosi = -cosi;          // 光线从内部射出
        std::swap(etai, etat); // 交换折射率
        n = -N;                // 法线取反
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrtf(k));
}
```

👉 关键逻辑：

* **判断光线是否在物体内部**：
  若入射方向与法线点积为负，则光线从物体内部射出，需要反转法线并交换折射率。
* **计算比例 `eta`**：空气到玻璃通常是 1 / 1.5。
* **判断全反射**：若 `k < 0`，返回 `(0,0,0)` 表示没有折射光。
* **输出折射方向**：用于 `cast_ray()` 的递归追踪。

### 3. 折射在光线追踪主循环中的使用

```cpp
Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, models, lights, depth + 1);
```

* `refract_dir`：折射方向；
* `refract_orig`：略微偏移交点，避免“自遮挡（self intersection）”；
* `cast_ray(...)`：递归追踪折射光线；
* 结果乘上 `material.albedo[3]` 混合入最终颜色。

**玻璃球**就靠这个计算出透光与反射的组合效果。

---

## 二、Duck 模型渲染逻辑（OBJ 三角面求交）

Duck 模型（`duck.obj`）加载为多个三角面：

```cpp
for (int i = 0; i < duck.nfaces(); i++) {
    Vec3f v0 = duck.point(duck.vert(i, 0));
    Vec3f v1 = duck.point(duck.vert(i, 1));
    Vec3f v2 = duck.point(duck.vert(i, 2));
    duck_triangles.push_back(Triangle(v0, v1, v2, glass));
}
```

这意味着每个面都是一个 `Triangle`。

### 在 `scene_intersect()` 中：

```cpp
if (duck.ray_triangle_intersect(i, orig, dir, dist_i))
```

这里调用了 `Model::ray_triangle_intersect()` 来判断光线是否与模型某个三角面相交。

## 三、Möller–Trumbore 算法详解

- **详细推导：知乎博客 https://zhuanlan.zhihu.com/p/451582864**

该算法是目前最经典、最快速的“光线与三角形相交检测”方法。

### 原理简述

已知：

* 三角形顶点：`v0, v1, v2`
* 射线：`orig + t * dir`

要求是否存在 $t, u, v$ 满足：
$$
orig + t \cdot dir = v0 + u(v1 - v0) + v(v2 - v0)
$$
其中 $u,v \ge 0$, $u+v \le 1$。

### 向量化推导

令：
$$
\text{edge1} = v1 - v0, \quad \text{edge2} = v2 - v0
$$

$$
orig + t \cdot dir = v0 + u \cdot edge1 + v \cdot edge2
$$

两边相减得到：
$$
t \cdot dir - u \cdot edge1 - v \cdot edge2 = v0 - orig
$$
这是一个三元一次方程组，可用 **克莱姆法则（Cramer’s Rule）** 解出。

---

### 程序逻辑逐行解释

```cpp
Vec3f edge1 = point(vert(fi, 1)) - point(vert(fi, 0));
Vec3f edge2 = point(vert(fi, 2)) - point(vert(fi, 0));
Vec3f pvec = cross(dir, edge2);
float det = edge1 * pvec;
if (det < 1e-5) return false;
```

* `pvec` 是辅助向量（`dir × edge2`）；
* `det` 是行列式（用来判断光线是否与三角面平行）。

---

```cpp
Vec3f tvec = orig - point(vert(fi, 0));
float u = tvec * pvec;
if (u < 0 || u > det) return false;
```

* 计算参数 $u$；
* 如果 $u$ 不在 [0, det] 范围内，说明交点在三角形外侧。

---

```cpp
Vec3f qvec = cross(tvec, edge1);
float v = dir * qvec;
if (v < 0 || u + v > det) return false;
```

* 类似地计算参数 $v$；
* $u + v > det$ ⇒ 超出三角形边界。

---

```cpp
tnear = edge2 * qvec * (1. / det);
return tnear > 1e-5;
```

* 最后计算 $t$（交点在射线上距离原点的倍数）；
* 若 $t > 0$，说明交点在前方；
* `tnear` 传回最近交点距离。

---

### ✅ 输出结果

若相交：

* `tnear` 表示交点距离；
* 程序中再计算 `hit = orig + dir * tnear`；
* 法线由三角形叉积计算；
* 材质取 `models[i].material`（这里是玻璃）。

---

## 总结（整体渲染流程）

| 阶段    | 功能              | 核心公式 / 函数                                    |
| ----- | --------------- | -------------------------------------------- |
| 环境贴图  | 远处背景颜色          | `envmap_lookup()`                            |
| 球体求交  | 基于几何方程          | `Sphere::ray_intersect()`                    |
| 三角面求交 | Möller–Trumbore | `Model::ray_triangle_intersect()`            |
| 折射    | Snell 定律        | `refract()`                                  |
| 反射    | 镜面反射公式          | `reflect()`                                  |
| 光照    | Blinn–Phong 模型  | diffuse + specular                           |
| 阴影    | 二次光线遮挡检测        | `scene_intersect()` again                    |
| 混合    | albedo 权重混合     | diffuse + specular + reflection + refraction |
