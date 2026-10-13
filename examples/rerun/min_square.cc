#include <rerun.hpp>

#include <cmath>

// Adam: Adaptive Moment Estimation,自适应矩估计
// 它的核心是:用历史梯度来平滑更新方向,再用历史梯度的大小来缩放更新量.
// x_new = x_old + learning_rate*(修正后的梯度的平均)/(修正后的梯度平方平均+epsilon)
// 保存的两个历史状态:
//   m:最近的梯度总体偏向哪个方向？
//   v:最近的梯度通常有多大？

int
main()
{
  rerun::RecordingStream const rec("adam_demo");
  rec.spawn().exit_on_failure();

  // 设置曲线样式：样式固定，因此使用 log_static。
  auto const style = [&](char const* path, char const* name, rerun::Rgba32 color)
  {
    rec.log_static(
        path,
        rerun::SeriesLines().with_names(name).with_colors(color).with_widths(
            2.0F));
  };

  style("adam/parameter/x", "x", {80, 180, 255});
  style("adam/parameter/target", "target = 3", {100, 220, 120});

  style("adam/loss/cost", "cost", {255, 170, 70});

  style("adam/direction/gradient", "gradient", {255, 100, 100});
  style("adam/direction/m_hat", "m_hat", {80, 180, 255});

  style("adam/scale/rms", "sqrt(v_hat)", {190, 130, 255});

  style("adam/update/delta_x", "delta_x", {100, 220, 180});


  // 待优化的变量。
  double x = 0.0;
  // Adam 保存的历史状态，初始为零。
  double m = 0.0;
  double v = 0.0;

  // learning_rate is alpha
  double const learning_rate = 0.1;
  double const beta1 = 0.9;
  double const beta2 = 0.999;
  double const epsilon = 1e-8;

  auto objective = [](double value) { return (value - 3.0) * (value - 3.0); };

  auto const gradient = [](double value) { return 2.0 * (value - 3.0); };

  // 在当前迭代位置记录参数和损失。
  auto const log_state = [&]
  {
    rec.log("adam/parameter/x", rerun::Scalars(x));
    rec.log("adam/parameter/target", rerun::Scalars(3.0));
    rec.log("adam/loss/cost", rerun::Scalars(objective(x)));
  };

  // 第 0 步：还没有进行任何更新。
  rec.set_time_sequence("iteration", 0);
  log_state();

  for(int t = 1; t <= 200; ++t)
  {
    // 当前梯度使用更新前的 x 计算。
    double const g = gradient(x);

    // 一阶矩、二阶矩的指数移动平均。
    // 这里的“矩”来自概率论和统计学中的 moment（矩）。
    // m（代表的就是梯度本身）可以改写为如下形式:
    //  m = m +(1.0 - beta1)*(cur_grid - m)
    // 从旧值 m 出发，朝当前值 cur_grid 移动，移动距离是两者差距的 (1-beta1)
    // 单次看是两个值之间的加权平均；不断把结果存回 m，就形成了梯度的指数移动平均。
    // 本质就是: 一次线性插值, 当 beta1 越接近1, 就越使用之前的值, 当 beta1 越接近 0, 就越接近新的值, 响应就越快.
    // 最终 m 累积的信息就是带正负号的梯度
    // v（代表的就是梯度的平方） 的更新也是线性插值,是旧的 v, 和当前梯度的平方之间
    // 所以 v 累积的信息就是梯度的平方
    // 这里 m和v 本质上是在进行指数加权（指数加权移动平均）
    //      这里的指数指的是：历史梯度的权重，随着它距离当前的轮数，按指数规律衰减。
    // 看单次更新：旧值与当前值之间的线性插值。看整个历史：对历史数据进行指数加权移动平均。
    //      这里的指数指的是历史权重的衰减
    // 这里自己总是将指数和 e^x 联系起来。。。这里自己要克服两个关键点：
    //     1. 指数的底数不一定是 e
    //     2. 指数函数也可以表示衰减（自己之前确实更多的想到的是增加。。。）
    //        历史梯度的权重是 beta2^k，它也可以写成指数的形式：e^(k*ln(beta2))，只是这里额外乘以了一个 (1-beta2)
    m = beta1 * m + (1.0 - beta1) * g;
    v = beta2 * v + (1.0 - beta2) * g * g;

    // 偏差修正。
    double const m_hat = m / (1.0 - std::pow(beta1, t));
    double const v_hat = v / (1.0 - std::pow(beta2, t));

    // 梯度尺度，以及带正负号的实际更新量。
    double const rms = std::sqrt(v_hat);
    double const delta_x = -learning_rate * m_hat / (rms + epsilon);

    x += delta_x;

    // 后续这些 log 都属于第 t 次迭代。
    rec.set_time_sequence("iteration", t);

    // 记录更新后的参数、损失。
    log_state();

    // 记录本次更新所使用的中间量。
    rec.log("adam/direction/gradient", rerun::Scalars(g));
    rec.log("adam/direction/m_hat", rerun::Scalars(m_hat));
    rec.log("adam/scale/rms", rerun::Scalars(rms));
    rec.log("adam/update/delta_x", rerun::Scalars(delta_x));
  }

  return 0;
}