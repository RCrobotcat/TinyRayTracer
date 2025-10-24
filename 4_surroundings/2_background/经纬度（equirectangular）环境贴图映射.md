```c++
Vec3f envmap_lookup(Vec3f &dir)
{
    // 将方向向量归一化
    Vec3f d = dir.normalize();

    // 球面坐标转换
    float phi = atan2(d.z, d.x);   // [-π, π]
    float theta = acos(d.y);       // [0, π]

    // 将球面坐标映射到 [0,1]
    float u = (phi + M_PI) / (2 * M_PI); // 水平方向
    float v = theta / M_PI;              // 垂直方向

    // 映射到贴图像素坐标
    int x = std::min(envmap_width  - 1, std::max(0, int(u * envmap_width)));
    int y = std::min(envmap_height - 1, std::max(0, int(v * envmap_height)));

    return envmap[x + y * envmap_width];
}

Vec3f cast_ray(const Vec3f &orig, Vec3f &dir, std::vector<Sphere> &spheres, const std::vector<Light> &lights,
               size_t depth = 0)
{
    Vec3f point, N;
    Material material;

    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material))
    {
        // return Vec3f(0.2, 0.7, 0.8); // background color
        return envmap_lookup(dir);
    }

    // mirror reflection
    Vec3f reflect_dir = reflect(dir, N).normalize();
    // offset the original point to avoid occlusion by the object itself
    Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);

    // refraction
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0;
    float specular_light_intensity = 0;
    for (int i = 0; i < lights.size(); i++)
    {
        Vec3f light_direction = (lights[i].position - point).normalize();

        // shadow check
        float light_distance = (lights[i].position - point).norm();
        Vec3f shadow_orig = light_direction * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
        // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_direction, spheres, shadow_pt, shadow_N, tmpmaterial) && (
                shadow_pt - shadow_orig).norm() < light_distance)
            continue;

        // Blin-Phong illumination model
        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_direction * N);

        Vec3f view_direction = (orig - point).normalize();
        Vec3f half_vector = (light_direction + view_direction).normalize();
        specular_light_intensity += lights[i].intensity * powf(std::max(0.f, half_vector * N),
                                                               material.specular_exponent);
    }

    return material.diffuse_color * diffuse_light_intensity * material.albedo[0]
           + Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1]
           + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}

```
---

## 一、总体流程回顾

```c++
Vec3f envmap_lookup(Vec3f &dir)
{
    // 将方向向量归一化
    Vec3f d = dir.normalize();

    // 球面坐标转换
    float phi = atan2(d.z, d.x);   // [-π, π]
    float theta = acos(d.y);       // [0, π]

    // 将球面坐标映射到 [0,1]
    float u = (phi + M_PI) / (2 * M_PI); // 水平方向
    float v = theta / M_PI;              // 垂直方向

    // 映射到贴图像素坐标
    int x = std::min(envmap_width  - 1, std::max(0, int(u * envmap_width)));
    int y = std::min(envmap_height - 1, std::max(0, int(v * envmap_height)));

    return envmap[x + y * envmap_width];
}

```

函数的目的是：给定一个方向向量 ( $\mathbf{d}$ )（例如反射／折射方向、或者视线方向），从事先加载好的“经纬度（equirectangular）环境贴图”中查出对应方向的颜色值，即将 3D 方向映射到贴图上的 UV 坐标，再读取像素。

你的代码流程是：

1. 归一化方向向量
   $$
   \mathbf{d} = \frac{\mathbf{dir}}{|\mathbf{dir}|}
   $$
2. 将归一化向量 $\mathbf{d} = (d_x, d_y, d_z)$ 转换为球面坐标角度：
   $$
   \phi = \operatorname{atan2}(d_z, d_x) \quad\text{（范围 ([-π, π])）} \\
   \theta = \arccos(d_y) \quad\text{（范围 ([0,π])）}
   $$
   
3. 将角度映射为 [0,1] 的纹理坐标：
   $$
   u = \frac{\phi + π}{2π} \\
   v = \frac{\theta}{π}
   $$
4. 将 (u, v) 映射为贴图像素坐标：
   $$
   x = \lfloor u \times \text{envmap\_width} \rfloor \\
   y = \lfloor v \times \text{envmap\_height} \rfloor
   $$
   并 clamp（限定）在合法范围内。
5. 返回颜色： `envmap[x + y * envmap_width]` 。

---

## 二、关键公式与几何含义

下面从几何背景、公式来源、各变量含义一步步说明。

### 2.1 球面坐标转换

对于**单位向量 $\mathbf{d}$** ：

* 通常定义：
  $$
  d_x = \cos\theta\cos\phi, \\
  d_y = \sin\theta, \\
  d_z = \cos\theta\sin\phi
  $$
  其中 $\theta$ 是纬度（或极角，从 “上” 到 “下”），$\phi$ 是经度。
  
* 你使用的定义略有不同（变体可能是：使用 $\theta = \arccos(d_y)$ 而非 $\arcsin)$：
  $$
  \phi = \operatorname{atan2}(d_z, d_x) \\
  \theta = \arccos(d_y)
  $$
  其中：

    * ( $\phi$ ) 给出围绕 y 轴（或 x-z 平面）旋转的角度，经度。
    * ( $\theta$ ) 给出从“顶点”（y=1）下降到向量的夹角。

### 2.2 映射到 UV 坐标

一旦得到角度 ( $\phi, \theta$ )：

* 水平映射（u）：
  $$
  u = \frac{\phi + π}{2π}
  $$
  这是将 $\phi \in [-π, π]$ 映为 $u \in [0,1]$。
* 垂直映射（v）：
  $$
  v = \frac{\theta}{π}
  $$
  
  因为 $\theta \in [0,π]$，所以 ($v \in [0,1]$)。

这样就得到了标准经纬度贴图（equirectangular map）中可用于纹理查找的 $(u,v)$ 坐标。

### 2.3 图像像素查找

* 得到 ( u, v ) 后，再乘以贴图分辨率（宽、高）即可得到像素索引 $(x,y)$：
  $$
  x = \lfloor u \cdot W \rfloor \\
  y = \lfloor v \cdot H \rfloor
  $$
  然后依据你的阵列结构 $\text{envmap}[x + y \cdot W]$ 得到颜色。

---

## 三、为什么这种映射可用作环境贴图查找

* 环境贴图往往是用 “360° × 180°” 的视野覆盖（全围绕）。经纬度贴图（equirectangular）就是把球面上的方向展开成一张2D图片。
* 给定一条射线或反射方向 ($\mathbf{d}$)，我们要知道该方向在环境中的颜色（即环境在该方向的亮度／颜色）。如果我们知道该方向向量指向球面某点，就可以查贴图中对应位置。
* 我们通过向量→角度→UV 的转换，把从三维空间中的方向映射到二维贴图上，从而实现背景颜色、反射、折射等效果。
* 在你的代码里，当场景中无交点或递归深度过大时，返回 `envmap_lookup(dir)` 即让该方向直接读取环境贴图，作为背景或环境反射。

---

## 四、与博客／技术文章对应参考

以下博客／技术文章详细介绍了经纬度贴图映射、球面坐标转换及 UV 查找，你可以拿来作为参考背景读物：

* How to map Equirectangular projection to Rectilinear projection by Nitish S. Mutha — 文章中讲解了从3D方向／经纬度到2D equirectangular贴图的映射关系。([NITISH MUTHA][1])
* Equirectangular Rendering (WebGPU Unleashed) — 第 5.3 节专门讨论如何在 shader 中从方向向量计算 UV 进行 equirectangular 贴图取样，其中也给出 GLSL 代码。([shi-yan.github.io][2])
* Understanding 360 images: Equations and code behind — 文章从 360° 图像／球面相机视角出发，介绍了方向向量 → 球面坐标 → 贴图坐标的推导。([Medium][3])

这些博客可以补充你的理解，特别是 “将方向映射到贴图” 的数学背景。

---

## 五、与你代码中可能需要注意或改进的点

* 是否需要 **v 方向翻转**？根据贴图加载方式，比如你这里 `for (int j = envmap_height - 1; j >= 0; j--)`，是以倒序 j 读取，可能需要 ( $v = 1 - \theta/π$ ) 或者 $y = envmap_height-1-int(v*H)$ 来保证上下方向正确。
* 插值（如双线性过滤）：你目前直接用整数索引取样，可能在低分辨率贴图或边缘处（如极点）有失真。若需要更好效果可考虑浮点索引 + 插值。
* 极点 (theta ≈ 0 或 π) 的收缩畸变：贴图上上极和下极可能形变严重，若效果可接受就不用优化。
* 使用 HDR 贴图或浮点贴图：若你环境贴图亮度范围很大（HDRI），你需要用浮点采样，并考虑重要性采样、mipmap 等。相关论文如 “Parameterization-Independent Importance Sampling of Environment Maps” 提出更高级方案。([arXiv][4])



---

`atan2(d.z, d.x)` 的数学定义是一个**带符号反正切函数**（two-argument arctangent function），
 它返回向量 $(x, z)$ 在平面上的极角 $\phi$，即从 x 轴正方向逆时针旋转到该向量的角度。

------

### ✅ 公式定义：

$$
\phi =
 \begin{cases}
 \arctan\left(\dfrac{d_z}{d_x}\right), & d_x > 0 \\
 \arctan\left(\dfrac{d_z}{d_x}\right) + \pi, & d_x < 0 \text{ 且 } d_z \ge 0 \\
 \arctan\left(\dfrac{d_z}{d_x}\right) - \pi, & d_x < 0 \text{ 且 } d_z < 0 \\
 +\dfrac{\pi}{2}, & d_x = 0,, d_z > 0 \\
 -\dfrac{\pi}{2}, & d_x = 0,, d_z < 0
 \end{cases}
$$

------

### ✅ 返回范围：

$$
\phi \in (-\pi,, \pi]
$$

------

### ✅ 几何意义：

- 输入：向量 $(d_x, d_z)$
- 输出：它在 x-z 平面上相对于 x 轴的角度。
- 比单参数的 `atan(d_z / d_x)` 更稳定，`atan2` 根据 $d_x、d_z$ 的符号自动判断象限，避免除以 0 或象限错误。

------

因此，在你的 `envmap_lookup()` 中：
$$
 \phi = \operatorname{atan2}(d_z, d_x)
$$
 计算的就是射线方向在 x-z 平面内的水平旋转角，用于确定环境贴图的 **水平方向 (u 坐标)**。



---

这句

```cpp
float theta = acos(d.y);
```

是在计算方向向量 **d** 与 **Y 轴正方向 (0, 1, 0)** 的夹角。它的数学公式源自**向量夹角的余弦定义**：

------

### ✅ 公式定义：

$$
\theta = \arccos(d_y)
$$

因为方向向量 **d** 是单位向量（已经 normalize），
 所以它满足：
$$
|\mathbf{d}| = 1 \Rightarrow d_x^2 + d_y^2 + d_z^2 = 1
$$

------

### ✅ 向量几何意义：

$$
\cos\theta = \frac{\mathbf{d} \cdot \mathbf{y}}{||\mathbf{d}|| \ ||\mathbf{y}||} = d_y
$$

 （其中 $\mathbf{y} = (0,1,0)$ 是 Y 轴单位向量）

因此反过来
$$
\theta = \arccos(d_y)
$$

------

### ✅ 结果范围：

$$
\theta \in [0, \pi]
$$

- 当 $d_y = 1$（即向上） → $\theta = 0$
- 当 $d_y = 0$（水平） → $\theta = \frac{\pi}{2}$
- 当 $d_y = -1$（向下） → $\theta = \pi$

------

### ✅ 在环境贴图中的意义：

这个角度表示从“上极点”到该方向的**垂直角**（纬度），
 随后通过：
$$
v = \frac{\theta}{\pi}
$$
将其线性映射为环境贴图中的 **垂直纹理坐标 v ∈ [0, 1]**。
