#include <stdio.h>
#include <stdbool.h>

// ============================================================
// Protothread 核心宏
// 利用 C 语言 switch-case 的 fall-through 特性实现"在行号处暂停并恢复"
// ============================================================

struct pt {
    unsigned char line;  // 只存一个数字：下次从哪行恢复
};

#define PT_INIT(pt)       ((pt)->line = 0)
#define PT_BEGIN(pt)      switch ((pt)->line) { case 0:
#define PT_WAIT_UNTIL(pt, cond)  \
    (pt)->line = __LINE__;       \
    case __LINE__:                \
    if (!(cond)) { return 1; }   // ← 返回 1 表示"还没完成，继续调度我"
#define PT_END(pt)         } return 0;  // ← 返回 0 表示"我结束了"

// ============================================================
// 示例：一个传感器读取任务
// ============================================================

static int sensor_task(struct pt *pt)
{
    printf("      进入 sensor_task, pt->line = %d\n", pt->line);

    PT_BEGIN(pt);

    // —— 第一步 ——
    printf("      读取传感器...\n");
    PT_WAIT_UNTIL(pt, false);   // 等传感器就绪（简化为永远 false）
    printf("      传感器就绪，读取完成\n");

    // —— 第二步 ——
    printf("      处理数据...\n");
    PT_WAIT_UNTIL(pt, false);   // 等处理完成
    printf("      数据处理完成\n");

    // —— 第三步 ——
    printf("      发送结果...\n");
    PT_WAIT_UNTIL(pt, false);   // 等发送完成
    printf("      发送完成\n");

    PT_END(pt);
}

// ============================================================
// 示例：条件驱动的任务
// ============================================================

static int condition_task(struct pt *pt)
{
    static int count = 0;  // 必须 static！不然每次重新进入函数会重置

    PT_BEGIN(pt);

    printf("      条件任务启动, count = %d\n", count);

    while (1) {
        count++;
        printf("      计数 %d，等待 count >= 5...\n", count);
        PT_WAIT_UNTIL(pt, count >= 5);

        printf("      ★ count = %d >= 5，通过！", count);
        printf(" 继续等待 count >= 8...\n");
        PT_WAIT_UNTIL(pt, count >= 8);

        printf("      ★ count = %d >= 8，通过！回到循环头\n", count);
    }

    PT_END(pt);
}

// ============================================================
// 演示 Protothread 宏展开后真正干了什么
// 手动展开 sensor_task 来展示
// ============================================================

static int sensor_task_expanded(struct pt *pt)
{
    // PT_BEGIN(pt)
    switch (pt->line) { case 0:

    // —— 第一步 ——
    printf("      [展开版] 读取传感器...\n");

    // PT_WAIT_UNTIL(pt, false)
    pt->line = 22;        // ← 记住行号 22
    case 22:               // ← 下次从这行 case 进入
    if (!(false)) {        // ← 条件不满足，yield
        return 1;          // ← 返回 1，"还没完"
    }

    // PT_WAIT_UNTIL(pt, false)
    pt->line = 28;
    case 28:
    if (!(false)) {
        return 1;
    }

    // PT_WAIT_UNTIL(pt, false)
    pt->line = 34;
    case 34:
    if (!(false)) {
        return 1;
    }

    // PT_END(pt)
    } return 0;
}
/* 每次调用的流程：
 *
 * 第 1 次调用: pt->line = 0
 *   → switch(0) 进入 case 0
 *   → 打印 "读取传感器"
 *   → pt->line = 22; return 1
 *
 * 第 2 次调用: pt->line = 22
 *   → switch(22) 进入 case 22
 *   → if(!(false)) return 1   ← 条件永假，每次进来都直接 return
 *
 * 所以第 2 次及以后每次调用，都卡在 case 22 处，永远进不到后面的代码！
 * 这就是 "permanently blocked at a wait point" 的场景
 */

// ============================================================
// 主调度循环模拟
// ============================================================

int main(void)
{
    struct pt pt1, pt2;
    PT_INIT(&pt1);
    PT_INIT(&pt2);

    printf("========================================\n");
    printf("  Protothread 调度演示\n");
    printf("  返回 1 = 还在执行中，需要继续调度\n");
    printf("  返回 0 = 执行完毕，不再调度\n");
    printf("========================================\n\n");

    // ---- sensor_task 演示 ----
    printf("--- sensor_task 演示 ---\n");
    for (int round = 1; round <= 5; round++) {
        printf("  轮次 %d:\n", round);
        int ret = sensor_task(&pt1);
        printf("      返回 %d\n", ret);
    }

    printf("\n--- 重新初始化，再来一次 ---\n");
    PT_INIT(&pt1);
    int ret = sensor_task(&pt1);
    printf("  重新初始化后返回 %d (又回到 case 0)\n\n", ret);

    // ---- condition_task 演示 ----
    printf("--- condition_task 演示 ---\n");

    /* ⚠️ 注意：condition_task 用了 static count
     * 所以就算重新 PT_INIT，count 仍然保留之前的值！
     * 这里不重新 init，展示条件逐步满足的过程
     */
    for (int round = 1; round <= 12; round++) {
        printf("  轮次 %d:\n", round);
        int r = condition_task(&pt2);
        printf("      返回 %d\n", r);
    }

    return 0;
}
