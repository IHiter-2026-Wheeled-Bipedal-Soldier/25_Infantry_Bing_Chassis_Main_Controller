


太棒了，这种追根溯源的数学推导正是《控制之美》这套书的精髓。

既然我们已经明确了：**在这个电机功率模型中，RLS 本质上就是一个状态不变、没有过程噪声、且观测噪声服从标准一维正态分布的卡尔曼滤波器**。

现在，我完全摒弃代码，严格按照《控制之美（卷2）》**6.3.2节“卡尔曼增益求解”**的推导逻辑，为你重新推导一遍 RLS 的最优增益 $K_k$ 以及最终的协方差更新公式 $P_k = \frac{1}{\lambda}(I - K_k H_k)P_{k-1}$。

---

### 1. 明确先验、后验与误差的定义

在卡尔曼滤波体系下，我们定义真实待估参数为 $\theta = \begin{bmatrix} k_1 \\ k_2 \end{bmatrix}$。

*   **先验状态估计**（在获取第 $k$ 次传感器数据前，我们对参数的猜测）：因为参数物理上短期不变，所以 $\hat{\theta}_k^- = \hat{\theta}_{k-1}$。
*   **测量方程**：$y_k = H_k \theta + v_k$。其中 $v_k$ 是测量噪声，根据你的假设，它服从标准一维正态分布，即噪声的方差（协方差矩阵）为标量 $R_c = 1$。
*   **后验状态估计**（融合了第 $k$ 次测量数据后的最新估计）：
    $$ \hat{\theta}_k = \hat{\theta}_k^- + K_k (y_k - H_k \hat{\theta}_k^-) $$

我们定义**先验估计误差** $e_k^-$ 和**后验估计误差** $e_k$ 分别为：
$$ e_k^- = \theta - \hat{\theta}_k^- $$
$$ e_k = \theta - \hat{\theta}_k $$

将测量方程和后验估计公式代入 $e_k$ 的定义中，寻找后验误差与先验误差的关系：
$$ e_k = \theta -[\hat{\theta}_k^- + K_k (H_k \theta + v_k - H_k \hat{\theta}_k^-)] $$
$$ e_k = (\theta - \hat{\theta}_k^-) - K_k H_k (\theta - \hat{\theta}_k^-) - K_k v_k $$
提取公因式，得到误差传递方程：
$$ e_k = (I - K_k H_k)e_k^- - K_k v_k $$

### 2. 展开后验误差协方差矩阵 $P_k$

我们设计 $K_k$ 的根本目标，是让后验估计的不确定性（即协方差）最小。
后验协方差矩阵定义为 $P_k = E[e_k e_k^T]$。将上面的误差传递方程代入：
$$ P_k = E[((I - K_k H_k)e_k^- - K_k v_k)((I - K_k H_k)e_k^- - K_k v_k)^T] $$
展开这个期望式：
$$ P_k = E[ (I - K_k H_k)e_k^- (e_k^-)^T (I - K_k H_k)^T - (I - K_k H_k)e_k^- v_k^T K_k^T - K_k v_k (e_k^-)^T (I - K_k H_k)^T + K_k v_k v_k^T K_k^T ] $$

因为先验估计误差 $e_k^-$ 与当前的测量噪声 $v_k$ 是**相互独立**的，它们的互协方差为 $0$（即 $E[e_k^- v_k^T] = 0$）。因此中间两项消掉，剩下：
$$ P_k = (I - K_k H_k) E[e_k^- (e_k^-)^T] (I - K_k H_k)^T + K_k E[v_k v_k^T] K_k^T $$

根据定义，先验协方差 $P_k^- = E[e_k^- (e_k^-)^T]$，测量噪声协方差 $R_c = E[v_k v_k^T]$。代入得：
$$ P_k = (I - K_k H_k) P_k^- (I - K_k H_k)^T + K_k R_c K_k^T $$

继续将其完全展开：
$$ P_k = P_k^- - K_k H_k P_k^- - P_k^- H_k^T K_k^T + K_k H_k P_k^- H_k^T K_k^T + K_k R_c K_k^T $$
合并最后两项的 $K_k$：
$$ P_k = P_k^- - K_k H_k P_k^- - P_k^- H_k^T K_k^T + K_k (H_k P_k^- H_k^T + R_c) K_k^T $$

### 3. 求解最优卡尔曼增益 $K_k$

为了让状态估计最准，我们需要让 $P_k$ 的迹 $\text{Tr}(P_k)$ 最小（迹代表了所有状态变量估计误差方差的总和）。对上式等号两边取迹：
$$ \text{Tr}(P_k) = \text{Tr}(P_k^-) - 2\text{Tr}(K_k H_k P_k^-) + \text{Tr}(K_k (H_k P_k^- H_k^T + R_c) K_k^T) $$
*(注：这里利用了矩阵迹的性质 $\text{Tr}(A) = \text{Tr}(A^T)$，所以 $\text{Tr}(K_k H_k P_k^-) = \text{Tr}(P_k^- H_k^T K_k^T)$ )*

要寻找极小值，我们将 $\text{Tr}(P_k)$ 对增益矩阵 $K_k$ 求偏导，并令其等于 $0$：
$$ \frac{\partial \text{Tr}(P_k)}{\partial K_k} = -2 (P_k^- H_k^T)^T + 2 K_k (H_k P_k^- H_k^T + R_c) = 0 $$
*(注：这里用到了矩阵求导法则 $\frac{\partial \text{Tr}(KAB)}{\partial K} = (AB)^T$ 和 $\frac{\partial \text{Tr}(KSK^T)}{\partial K} = 2KS$ )*

化简并解出 $K_k$：
$$ K_k (H_k P_k^- H_k^T + R_c) = P_k^- H_k^T $$
$$ K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R_c)^{-1} $$

**【降维打击时刻】**：
在多维传感器中，括号内是一个矩阵，必须求逆。
但在我们的功率模型中，观测矩阵 $H_k$ 是 $1 \times 2$ 的行向量，$P_k^-$ 是 $2 \times 2$ 的矩阵，$R_c = 1$ 是一维标量。
所以，**项 $(H_k P_k^- H_k^T + R_c)$ 必然是一个 $1 \times 1$ 的纯数字标量！**
矩阵求逆退化成了普通的标量除法。

### 4. 协方差矩阵 $P_k$ 的化简

我们将求得的最优 $K_k$ 关系式：$K_k (H_k P_k^- H_k^T + R_c) = P_k^- H_k^T$，代回到第2步完全展开的 $P_k$ 方程中：
$$ P_k = P_k^- - K_k H_k P_k^- - \underbrace{P_k^- H_k^T}_{替换} K_k^T + \underbrace{K_k (H_k P_k^- H_k^T + R_c)}_{替换为 P_k^- H_k^T} K_k^T $$
$$ P_k = P_k^- - K_k H_k P_k^- - (P_k^- H_k^T) K_k^T + (P_k^- H_k^T) K_k^T $$

神奇的消元发生了，后两项一正一负直接抵消：
$$ P_k = P_k^- - K_k H_k P_k^- $$
提取公因式 $P_k^-$：
$$ P_k = (I - K_k H_k) P_k^- $$
这就是最优增益下，后验协方差极其优雅的简化形式。

### 5. 从 KF 跨越到 RLS（引入遗忘因子 $\lambda$）

至此，推导出的仍是标准卡尔曼滤波公式。
如果是标准 KF（没有过程噪声），随着时间推移，$P_k$ 会越来越小趋近于0，最终 $K_k$ 变成0，模型会“僵死”，不再理会新的功率测量值（无法适应电机发热带来的参数变化）。

为了解决这个问题，**RLS 引入了遗忘因子 $\lambda \in (0, 1]$**。
在卡尔曼滤波的视角下，遗忘因子的物理意义是：**人为地将先验不确定性放大**，用来替代 KF 中繁琐的过程噪声协方差 $Q$。

我们直接定义先验协方差矩阵为上一时刻的后验协方差除以 $\lambda$：
$$ P_k^- \triangleq \frac{1}{\lambda} P_{k-1} $$

现在，我们将这个定义，以及一维标准正态噪声 $R_c = 1$ 代入刚刚推导出的 $K_k$ 和 $P_k$ 公式中：

**计算增益 $K_k$：**
$$ K_k = \frac{ \frac{1}{\lambda} P_{k-1} H_k^T }{ H_k ( \frac{1}{\lambda} P_{k-1} ) H_k^T + 1 } $$
分子分母同时乘以 $\lambda$ 进行化简：
$$ K_k = \frac{ P_{k-1} H_k^T }{ \lambda + H_k P_{k-1} H_k^T } $$
*(这就是标准的 RLS 增益求解公式)*

**更新协方差 $P_k$：**
$$ P_k = (I - K_k H_k) P_k^- $$
代入 $P_k^- = \frac{1}{\lambda} P_{k-1}$：
$$ P_k = \frac{1}{\lambda} (I - K_k H_k) P_{k-1} $$
*(这就是你在题目中要求推导出来的 RLS 协方差更新最终形式)*

---

**推导总结：**
没有任何魔法和死记硬背。只要按照《控制之美》中**求误差 -> 算协方差 -> 矩阵求导求迹的极小值 -> 整理化简**的标准流程，加上**协方差被遗忘因子放大（$P_k^- = \frac{1}{\lambda}P_{k-1}$）**以及**一维测量噪声（$R_c=1$）**这两个特定条件，KF 就会犹如水到渠成一般，自然蜕变为你所需的 RLS 算法公式。