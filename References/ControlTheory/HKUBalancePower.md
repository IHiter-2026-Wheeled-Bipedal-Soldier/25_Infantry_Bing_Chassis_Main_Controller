# 平衡步兵功率限制

根据上交模型，我们可以将左右轮的力矩分解为三项： $T _ { l } = M _ { l } + U _ { s p e e d } + U _ { y a w } ( 1 . 1 ) T _ { r } = M _ { r } + U _ { s p e e d } - U _ { y a w } ($ (1.2)其中 $M _ { l } = U _ { l e g } ^ { l } + U _ { p i t c h } ^ { l } , M _ { r } = U _ { l e g } ^ { r } + U _ { p i t c h } ^ { r } , U _ { l e g }$ $U _ { l e g }$ 是经过K矩阵输出的与控制腿摆角 $\theta$ 有关的力矩控制分量， $U _ { p i t c h }$ 是经过K矩阵输出与机体pitch角 $\phi$ 有关的力矩控制分量。

$U _ { s p e e d }$ 是经过K矩阵输出的机体位移 与速度 $\frac { d s } { d t }$ 力矩控制分量的线性叠加， $U _ { y a w }$ 是经过K矩阵输出的机体yaw角度力矩控制项。

# 功率限制思路：

不试图限制 $M _ { l }$ 和 $M _ { r }$ 来控制功率，因为这两项关乎到平衡步兵的姿态平衡  
当检测到当前指令预期功率 $P _ { c m d } > P _ { m a x }$ 时，我们尽可能地约束 $U _ { s p e e d } \to U _ { s p e e d } ^ { \prime }$ $U _ { y a w } \to U _ { y a w } ^ { \prime }$ 来限制功率，使得$P _ { c m d } ^ { \prime }  P _ { m a x }$   
引入约束条件 $U _ { s p e e d } ^ { \prime } = k U _ { y a w } ^ { \prime } ($ (2.1)， $\begin{array} { r } { k = \frac { U _ { s p e e d } } { U _ { y a w } } } \end{array}$ Uspeed Uyaw 并试图解出 $U _ { s p e e d } ^ { \prime }$ $U _ { y a w } ^ { \prime }$   
计算衰减因子 $k _ { s p e e d } = \frac { U _ { s p e e d } ^ { \prime } } { U _ { s p e e d } }$ Uspeed kyaw $k _ { y a w } = \frac { U _ { y a w } ^ { \prime } } { U _ { y a w } }$ Uyaw

# 电机模型

我们可以通过以下电机模型来预测底盘功率

$$
P = \tau \Omega + k _ {1} | \Omega | + k _ {2} \tau^ {2} + k _ {3} \tag {3.1}
$$

其中 为电机的实际输出力矩， $\Omega$ 为电机的实际角速度， $\boldsymbol { k } _ { 1 } , \boldsymbol { k } _ { 2 } , \boldsymbol { k } _ { 3 }$ 为常数。 $k _ { 3 }$ 可认为为底盘系统中非电机线圈上的常态损耗。

# 一些假设

我们可以认为电机的力矩闭环速度是很快的 $f > 1 0 0 0 H z \rangle$ )。 因此，我们可以假设力矩指令等于反馈力矩，即$\tau _ { c u r } = \tau _ { f e e d b a c k }$ 。为了方便表述，简称这两项为 。以下有关力矩的符号统一认为是指令力矩。  
在一个控制周期内 $( f = 1 0 0 0 H z )$ ，电机的实际角速度 $\Omega _ { c u r }$ 相对于实际力矩的变化是很小的。以下有关角速度的符号统一认为是实际电机角速度 $\Omega _ { c u r }$ 而不是期望电机角速度。

# 公式推导

将公式 代入到电机预测模型 中

$$
P _ {m a x} = \Omega_ {l} M _ {l} + \Omega_ {l} (k + 1) U _ {y a w} ^ {\prime} + k _ {1} (| \Omega_ {l} | + | \Omega_ {r} |) + \Omega_ {r} M _ {r} + \Omega_ {r} (k - 1) U _ {y a w} ^ {\prime} + k _ {2} \left[ M _ {l} ^ {2} + 2 (k + 1) M _ {l} U _ {y a w} ^ {\prime} + (k + 1) ^ {2} U _ {y a w} \right]
$$

令 $U _ { y a w } = x$ , 化简整理为标准一元二次方程的形式:

$$
k _ {2} (2 k ^ {2} + 2) x ^ {2} + [ 2 k _ {2} (k + 1) M _ {l} + 2 k _ {2} (k - 1) M _ {r} + \Omega_ {l} (k + 1) + \Omega_ {r} (k - 1) ] x + \Omega_ {l} M _ {l} + \Omega_ {r} M _ {r} + k _ {1} (| \Omega_ {l} | + | \Omega_ {r} |) + k _ {2} (
$$

其中

$$
A = k _ {2} \left(2 k ^ {2} + 2\right) \quad B = 2 k _ {2} (k + 1) M _ {l} + 2 k _ {2} (k - 1) M _ {r} + \Omega_ {l} (k + 1) + \Omega_ {r} (k - 1)
$$

$$
C = \Omega_ {l} M _ {l} + \Omega_ {r} M _ {r} + k _ {1} \left(\left| \Omega_ {l} \right| + \left| \Omega_ {r} \right|\right) + k _ {2} \left(M _ {r} ^ {2} + M _ {l} ^ {2}\right) + k _ {3} - P _ {\max }
$$

根据 $\Delta = B ^ { 2 } - 4 A C$ 来讨论根的取值

如果 $\Delta > 0$ ,我们可以解得两个根 $\begin{array} { r } { x _ { 1 } = \frac { - B + \sqrt { \Delta } } { 2 A } , x _ { 2 } = \frac { - B - \sqrt { \Delta } } { 2 A } } \end{array}$

选取与原 $U _ { y a w }$ 符号相同的有效解  
如果有多个解满足方程，尽可能选取绝对值最大的解  
如果没有有效解满足方程，直接取 $U _ { y a w } ^ { \prime } = x = 0$ 作为解

如果 $\Delta \le 0$ , 我们选取在实域上最接近的解 $\begin{array} { r } { U _ { y a w } ^ { \prime } = x = \frac { - B } { 2 A } } \end{array}$

选取与原 $U _ { y a w }$ 符号相同的有效解  
如果没有有效解满足方程，直接取 $U _ { y a w } ^ { \prime } = x = 0$ 作为解

最终我们分别计算出 $U _ { y a w }$ 和 $U _ { s p e e d }$ 的衰减系数 $k _ { s p e e d } = \frac { U _ { s p e e d } ^ { \prime } } { U _ { s p e e d } }$ Uspeed $k _ { y a w } = \frac { U _ { y a w } ^ { \prime } } { U _ { y a w } }$ 。 为了不影响控制效果，我们应该将衰减系数限制在 ， 从而得到最终的衰减系数。 将衰减系数等比缩放作用于腿电机相关项与轮电机相关项， 即可达到限制功率的效果。

# 功率环与能量环

以上功率限制方法可以认为是某种功率环，将底盘实际功率控制在当前最大允许功率附近。当底盘环路中接入了超级电容后，通过合理利用电容剩余能量 $E _ { c }$ 的方式，我们可以进一步显著地提升平衡步兵的底盘运动效果。

这个赛季，我们简单地利用P控制器或者PD控制器(防止超调)对电容剩余能量进行闭环。我们定义了两种模式“开电容”与“关电容”，其本质区别为电容剩余期望能量。假设电容总电量为 $C$ ,我们规定在开电容条件下电容剩余期望能量为 $E _ { o n } = 0 . 4 \times C ,$ 在关电容的模式下电容剩余期望能量为 $E _ { o f f } = 0 . 8 \times C$ 。请注意，不论如何，电容里应该留起码 $3 0 \%$ 左右的剩余电量以在起身、打滑等关键时刻维持平衡步兵的平衡姿态，同时防止出现超功率的情况。

假设当前的裁判系统最大允许功率为 $P _ { r }$ ，则设计P或者PD控制器如下。

$$
e (t) = E _ {t a r g e t} - E _ {c} P _ {m a x} = P _ {r} + K _ {p} e (t) + K _ {d} \frac {e (t) - e (t - 1)}{d t}
$$

此外，为了不显著影响平衡步兵的运动控制效果，我们应当将 $P _ { m a x }$ 限制在某个最小功率 $P _ { 0 }$ 以上。经过不断地调试，我们最终选定了 $P _ { 0 } = 3 5 W$ 。