/**
 * my_line_follow.c — 智能车巡线图像处理流水线
 *
 * 处理流程（每帧调用 ProcessFrame）：
 *   1. ApplyEdgeCompensation  — 区域阈值补偿 + 二值化
 *   2. InitImgInfo            — 初始化全局数组
 *   3. DrawFrameBorder        — 图像左右边缘画黑边
 *   4. FindImageTop           — 找截止行 + 最长白列
 *   5. FindSidelines          — 逐行搜索左右边线
 *   6. [FindGuaidians + Buxian] — 拐点检测 + 补线（可选）
 *   7. FindMidline            — 计算中线
 *   8. CalculateError         — 计算控制误差 Dir_err
 *
 * 坐标系约定：
 *   行号 0 = 图像最顶部（最远处），LINE_IMG_H-1 = 图像最底部（最近处）
 *   列号 0 = 最左侧，LINE_IMG_W-1 = 最右侧
 *   白色(255) = 赛道，黑色(0) = 背景/边线外
 */

#include "my_line_follow.h"

// ============================================================
// 全局数组 — 每行边线/中线/宽度数据
// 数组大小为 LINE_IMG_H+1，多出的一个元素作为哨兵，
// 避免 FindSidelines 处理最后一行时访问 Left[row+1] 越界
// ============================================================
int   Left[LINE_IMG_H + 1];            // 左边线列号
int   Right[LINE_IMG_H + 1];           // 右边线列号
int   Mid[LINE_IMG_H + 1];             // 中线列号
int   WhiteWidth[LINE_IMG_H + 1];      // 每行赛道宽度

// ============================================================
// 二值化结果数组
// ============================================================
uint8 line_binary[LINE_IMG_H][LINE_IMG_W];

// ============================================================
// 全局状态变量
// ============================================================
int   imgTop         = 0;                   // 截止行（最远可见行）
int   maxColumn      = LINE_IMG_W / 2;      // 最长白列列号
int   lastMid_Bottom = LINE_IMG_W / 2;      // 上一帧底行中线位置
float Dir_err        = 0.0f;                // 当前帧方向误差
float Last_Dir_err   = 0.0f;                // 上一帧方向误差
int   L_loseCount    = 0;                   // 左侧丢线计数
int   R_loseCount    = 0;                   // 右侧丢线计数

// ============================================================
// 内部辅助函数
// ============================================================

// 计算两点间斜率 k = Δ列 / Δ行
static float CalcSlope(int row1, int col1, int row2, int col2)
{
    if (row2 == row1) return 0.0f;
    return (float)(col2 - col1) / (float)(row2 - row1);
}

// 计算直线截距 b = col - k * row
static float CalcIntercept(int row1, int col1, int row2, int col2)
{
    float k = CalcSlope(row1, col1, row2, col2);
    return (float)col1 - k * (float)row1;
}

// ============================================================
// 3.1 ApplyEdgeCompensation — 区域阈值补偿 + 二值化
//
// 镜头存在径向畸变，图像边缘比中心暗。
// 对左右边缘区域降低二值化阈值，防止赛道边缘被误判为黑色。
// 输入：gray_img（原始灰度），otsu_threshold（大津法全局阈值）
// 输出：line_binary（0/255 二值图）
// ============================================================
static void ApplyEdgeCompensation(uint8 otsu_threshold,
                                  const uint8 gray_img[LINE_IMG_H][LINE_IMG_W])
{
    uint8 thresh = otsu_threshold;
    if (thresh < EDGE_COMP_THRESHOLD_MIN) {
        thresh = EDGE_COMP_THRESHOLD_MIN;
    }

    for (int row = 0; row < LINE_IMG_H; row++) {
        for (int col = 0; col < LINE_IMG_W; col++) {
            int local_threshold = (int)thresh;

            // 左边缘区域：降低阈值
            if (col <= EDGE_COMP_LEFT_BOUNDARY) {
                local_threshold = (int)thresh - EDGE_COMP_THRESHOLD_DELTA;
            }
            // 右边缘区域：降低阈值
            else if (col >= EDGE_COMP_RIGHT_LOW) {
                local_threshold = (int)thresh - EDGE_COMP_THRESHOLD_DELTA;
            }

            if (gray_img[row][col] > local_threshold) {
                line_binary[row][col] = 255;    // 白色 = 赛道
            } else {
                line_binary[row][col] = 0;      // 黑色 = 背景
            }
        }
    }
}

// ============================================================
// 3.2 InitImgInfo — 初始化图像信息
// ============================================================
static void InitImgInfo(void)
{
    for (int i = 0; i <= LINE_IMG_H; i++) {
        Left[i]       = 0;
        Right[i]      = LINE_IMG_W - 1;
        Mid[i]        = LINE_IMG_W / 2;
        WhiteWidth[i] = 0;
    }
    imgTop    = 0;
    maxColumn = LINE_IMG_W / 2;
}

// ============================================================
// 3.3 DrawFrameBorder — 画黑边
//
// 在左右边缘各画 1 列黑线，使边线搜索在触及图像边界时自然终止。
// ============================================================
static void DrawFrameBorder(void)
{
    for (int row = LINE_IMG_H - 1; row > imgTop; row--) {
        line_binary[row][0]                = 0;   // 最左列涂黑
        line_binary[row][LINE_IMG_W - 1]   = 0;   // 最右列涂黑
    }
}

// ============================================================
// 3.4 FindImageTop — 找截止行与最长白列 ★★★
//
// Step 1: 在底行，从上一帧中线位置向两侧搜索赛道边界（"白-黑-黑" 跳变）
// Step 2: 在左右边界之间，找最长白色列（从底部向上数连续白色像素最多的列）
// Step 3: 根据最长白列长度计算截止行 imgTop
//
// 物理含义：
//   imgTop 越小 → 赛道看得越远 → 当前大概率是直道
//   imgTop 越大 → 赛道看得越近 → 当前大概率是弯道
// ============================================================
static void FindImageTop(void)
{
    int bottom = LINE_IMG_H - 1;

    // ------ Step 1: 在底行找赛道左右边界 ------
    // 从上一帧中线位置向左搜索 "白-黑-黑" 跳变
    int l_side = 1;
    for (int col = lastMid_Bottom; col > 1; col--) {
        if ((line_binary[bottom][col] == 255
             && line_binary[bottom][col - 1] == 0
             && line_binary[bottom][col - 2] == 0)
            || col == 2) {
            l_side = col;
            break;
        }
    }

    // 从上一帧中线位置向右搜索 "白-黑-黑" 跳变
    int r_side = LINE_IMG_W - 2;
    for (int col = lastMid_Bottom; col < LINE_IMG_W - 2; col++) {
        if ((line_binary[bottom][col] == 255
             && line_binary[bottom][col + 1] == 0
             && line_binary[bottom][col + 2] == 0)
            || col == LINE_IMG_W - 3) {
            r_side = col;
            break;
        }
    }

    // ------ Step 2: 在左右边界之间找最长白色列 ------
    int maxLen   = 0;
    maxColumn    = LINE_IMG_W / 2;

    for (int col = l_side; col <= r_side; col++) {
        int len = bottom;
        // 从底部向上数连续白色像素
        while (len >= 0 && line_binary[len][col] == 255) {
            len--;
        }
        // 转换为从底部算起的白列长度
        len = LINE_IMG_H - len;
        if (len > maxLen) {
            maxLen    = len;
            maxColumn = col;
        }
    }

    // ------ Step 3: 计算截止行 ------
    imgTop = LINE_IMG_H - maxLen + 1;
    if (imgTop < 0) {
        imgTop = 0;
    }

    // 将底行搜索结果存入数组，避免 FindSidelines 重复搜索底行
    Left[bottom]  = l_side;
    Right[bottom] = r_side;
}

// ============================================================
// 3.5 FindSidelines — 逐行搜索左右边线 ★★★
//
// 从 startRow 向 endRow（逆序），逐行从 maxColumn 向两侧搜索边线。
// 搜索模式：寻找 "白-黑-黑" 跳变（确认赛道边缘）。
// 容错模式： "白-黑-白" 且距离上一行边线 < SIDELINE_TOLERANCE_COL 列。
// 丢线处理：maxColumn 处为黑色时，继承上一行边线位置。
// ============================================================
static void FindSidelines(int startRow, int endRow)
{
    L_loseCount = 0;
    R_loseCount = 0;

    for (int row = startRow; row >= endRow; row--) {

        // ------ 丢线检查：最长白列处为黑色（障碍/阴影）------
        if (line_binary[row][maxColumn] == 0) {
            Left[row]  = Left[row + 1];
            Right[row] = Right[row + 1];
            continue;
        }

        // ========== 找左边线 ==========
        bool foundLeft = false;
        for (int col = maxColumn; col > 1; col--) {
            // 模式1：白-黑-黑（标准边线）
            if (line_binary[row][col] == 255
                && line_binary[row][col - 1] == 0
                && line_binary[row][col - 2] == 0) {
                Left[row]  = col;
                foundLeft  = true;
                break;
            }
            // 模式2（容错）：白-黑-白，且距上一行边线在容差范围内
            if (line_binary[row][col] == 255
                && line_binary[row][col - 1] == 0
                && line_binary[row][col - 2] == 255
                && (col - Left[row + 1]) < SIDELINE_TOLERANCE_COL) {
                Left[row]  = col;
                foundLeft  = true;
                break;
            }
        }
        if (!foundLeft) {
            Left[row] = 1;      // 置为左边界
            L_loseCount++;
        }

        // ========== 找右边线 ==========
        bool foundRight = false;
        for (int col = maxColumn; col < LINE_IMG_W - 2; col++) {
            // 模式1：白-黑-黑（标准边线）
            if (line_binary[row][col] == 255
                && line_binary[row][col + 1] == 0
                && line_binary[row][col + 2] == 0) {
                Right[row]  = col;
                foundRight  = true;
                break;
            }
            // 模式2（容错）：白-黑-白，且距上一行边线在容差范围内
            if (line_binary[row][col] == 255
                && line_binary[row][col + 1] == 0
                && line_binary[row][col + 2] == 255
                && (Right[row + 1] - col) < SIDELINE_TOLERANCE_COL) {
                Right[row]  = col;
                foundRight  = true;
                break;
            }
        }
        if (!foundRight) {
            Right[row] = LINE_IMG_W - 2;   // 置为右边界
            R_loseCount++;
        }

        // 记录本行赛道宽度
        WhiteWidth[row] = Right[row] - Left[row];
    }
}

// ============================================================
// 3.6 FindMidline — 计算中线
// ============================================================
static void FindMidline(void)
{
    for (int row = LINE_IMG_H - 1; row > imgTop; row--) {
        Mid[row] = (Left[row] + Right[row]) / 2;
    }
    // 更新底行中线位置（供下一帧 FindImageTop 使用）
    lastMid_Bottom = Mid[LINE_IMG_H - 1];
}

// ============================================================
// 3.7 CalculateError — 计算控制误差 ★★★
//
// 偏差 = 图像中心列(IMG_W/2) - 当前行中线列
//   Dir_err > 0 → 中线偏左 → 车应左转
//   Dir_err < 0 → 中线偏右 → 车应右转
//
// 使用动态前瞻行：速度越快看得越远（forward 越小）。
// 误差做过限幅和帧间变化率限幅。
// ============================================================
static void CalculateError(void)
{
    // ------ Step 1: 逐行计算偏差 ------
    float Dir_Err[LINE_IMG_H];
    for (int row = imgTop + 1; row < LINE_IMG_H - 1; row++) {
        Dir_Err[row] = (float)(LINE_IMG_W / 2)
                       - ((float)Left[row] + (float)Right[row]) / 2.0f;
    }

    // ------ Step 2: 选择前瞻行 ------
    // forward = FORWARD_DEFAULT - (当前速度 / FORWARD_SPEED_DIVISOR)
    // TODO: 接入实际速度变量后替换 speed 值
    int speed   = 0;    // 当前速度（外部速度环提供）
    int forward = FORWARD_DEFAULT - (speed / FORWARD_SPEED_DIVISOR);
    if (forward < imgTop + 1) forward = imgTop + 1;
    if (forward > FORWARD_MAX) forward = FORWARD_MAX;

    Dir_err = Dir_Err[forward];

    // ------ Step 3: 误差限幅 ------
    if (Dir_err > DIR_ERR_MAX)  Dir_err = DIR_ERR_MAX;
    if (Dir_err < -DIR_ERR_MAX) Dir_err = -DIR_ERR_MAX;

    // ------ Step 4: 误差变化率限幅（抗突变）------
    float d_err = Dir_err - Last_Dir_err;
    if (d_err >= DIR_ERR_DELTA_MAX) {
        Dir_err = Last_Dir_err + DIR_ERR_DELTA_MAX;
    } else if (d_err <= -DIR_ERR_DELTA_MAX) {
        Dir_err = Last_Dir_err - DIR_ERR_DELTA_MAX;
    }

    Last_Dir_err = Dir_err;
}

// ============================================================
// 4.2 FindGuaidians — 拐点检测
//
// 拐点是边线方向突变的位置，用于判断赛道形状变化（环岛/十字等）。
//
// 四类拐点：
//   左上 L_h: 左边线突然向右内收（赛道变宽的开始，从上往下扫）
//   左下 L_l: 左边线突然向左外扩（赛道变窄的开始，从下往上扫）
//   右上 R_h: 右边线突然向左内收（从上往下扫）
//   右下 R_l: 右边线突然向右外扩（从下往上扫）
// ============================================================
static void FindGuaidians(GuaiPoint *L_h, GuaiPoint *L_l,
                          GuaiPoint *R_h, GuaiPoint *R_l)
{
    L_h->found = false; L_l->found = false;
    R_h->found = false; R_l->found = false;

    // ====== 找上拐点（从上往下扫）======
    for (int row = imgTop + 2; row <= LINE_IMG_H - 5; row++) {

        // --- 左上拐点 L_h ---
        if (!L_h->found && Left[row] > 3) {
            // 边线附近像素模式：当前行边线为白，下一行同列和内一列为白
            bool pattern_ok =
                (line_binary[row][Left[row]] == 255)
                && (line_binary[row + 1][Left[row]] == 255)
                && (line_binary[row + 1][Left[row] - 1] == 255);

            // 赛道宽度在增大（下方比上方宽 > GUAI_WIDTH_INCREASE_UP 列）
            bool width_increasing =
                (WhiteWidth[row + 2] - WhiteWidth[row] > GUAI_WIDTH_INCREASE_UP)
                && (WhiteWidth[row + 3] > WhiteWidth[row])
                && (WhiteWidth[row + 4] > WhiteWidth[row]);

            // 边线斜率合理（左侧斜率 < GUAI_SLOPE_LIMIT，即非水平）
            float sl = CalcSlope(row, Left[row], row + 1, Left[row + 1]);
            bool slope_ok = (sl < GUAI_SLOPE_LIMIT);

            if (pattern_ok && width_increasing && slope_ok) {
                L_h->row   = row + 1;
                L_h->col   = Left[row];
                L_h->found = true;
            }
        }

        // --- 右上拐点 R_h（对称逻辑）---
        if (!R_h->found && Right[row] < LINE_IMG_W - 3) {
            bool pattern_ok =
                (line_binary[row][Right[row]] == 255)
                && (line_binary[row + 1][Right[row]] == 255)
                && (line_binary[row + 1][Right[row] + 1] == 255);

            bool width_increasing =
                (WhiteWidth[row + 2] - WhiteWidth[row] > GUAI_WIDTH_INCREASE_UP)
                && (WhiteWidth[row + 3] > WhiteWidth[row])
                && (WhiteWidth[row + 4] > WhiteWidth[row]);

            float sl = CalcSlope(row, Right[row], row + 1, Right[row + 1]);
            bool slope_ok = (sl > -GUAI_SLOPE_LIMIT);

            if (pattern_ok && width_increasing && slope_ok) {
                R_h->row   = row + 1;
                R_h->col   = Right[row];
                R_h->found = true;
            }
        }
    }

    // ====== 找下拐点（从下往上扫）======
    for (int row = LINE_IMG_H - 3; row >= imgTop + 3; row--) {

        // --- 左下拐点 L_l ---
        if (!L_l->found && Left[row] > 3) {
            bool pattern_ok =
                (line_binary[row][Left[row]] == 255)
                && (line_binary[row - 1][Left[row]] == 255)
                && (line_binary[row - 1][Left[row] - 1] == 255);

            // 赛道宽度在减小（上方比当前宽 > GUAI_WIDTH_INCREASE_DOWN 列）
            bool width_narrowing =
                (WhiteWidth[row - 2] - WhiteWidth[row + 1] > GUAI_WIDTH_INCREASE_DOWN)
                && (WhiteWidth[row - 2] > WhiteWidth[row])
                && (WhiteWidth[row - 3] > WhiteWidth[row]);

            if (pattern_ok && width_narrowing && row > L_h->row) {
                L_l->row   = row - 1;
                L_l->col   = Left[row];
                L_l->found = true;
            }
        }

        // --- 右下拐点 R_l（对称逻辑）---
        if (!R_l->found && Right[row] < LINE_IMG_W - 3) {
            bool pattern_ok =
                (line_binary[row][Right[row]] == 255)
                && (line_binary[row - 1][Right[row]] == 255)
                && (line_binary[row - 1][Right[row] + 1] == 255);

            bool width_narrowing =
                (WhiteWidth[row - 2] - WhiteWidth[row + 1] > GUAI_WIDTH_INCREASE_DOWN)
                && (WhiteWidth[row - 2] > WhiteWidth[row])
                && (WhiteWidth[row - 3] > WhiteWidth[row]);

            if (pattern_ok && width_narrowing && row > R_h->row) {
                R_l->row   = row - 1;
                R_l->col   = Right[row];
                R_l->found = true;
            }
        }
    }
}

// ============================================================
// 4.3 Buxian — 拐点间补线
//
// 当某些行的边线丢失时，通过已检测到的拐点进行线性补线。
// 补线规则按优先级匹配：
//   1. 四拐点全存在              → 左上?左下连线，右上?右下连线
//   2. 左上+左下+右上            → 左侧两点连线，右侧从右上到底部连线
//   3. 右上+右下+左上            → 右侧两点连线，左侧从左上到底部连线
//   4. 仅左上+右上（下有丢线）   → 各自到底部连线
//   5. 仅左上+左下               → 两点间连线
//   6. 仅右上+右下               → 两点间连线
//   7. 仅左上（且双边丢线）       → 从左上到底部连线
//   8. 仅右上（且双边丢线）      → 从右上到底部连线
// ============================================================
static void Buxian(GuaiPoint L_h, GuaiPoint L_l,
                   GuaiPoint R_h, GuaiPoint R_l)
{
    int  bottom_row      = LINE_IMG_H - 1 - BUXIAN_BOTTOM_ROW_OFFSET;
    int  bottom_left_col = Left[bottom_row] - BUXIAN_BOTTOM_COL_OFFSET;
    int  bottom_right_col= Right[bottom_row] + BUXIAN_BOTTOM_COL_OFFSET;
    if (bottom_left_col  < 0)            bottom_left_col  = 0;
    if (bottom_right_col >= LINE_IMG_W)  bottom_right_col = LINE_IMG_W - 1;
    float k, b;

    // ---- 规则1: 四拐点全存在 ----
    if (L_h.found && L_l.found && R_h.found && R_l.found) {
        // 左侧：左上→左下连线
        k = CalcSlope(L_h.row, L_h.col, L_l.row, L_l.col);
        b = CalcIntercept(L_h.row, L_h.col, L_l.row, L_l.col);
        for (int i = L_h.row; i <= L_l.row; i++) {
            Left[i] = (int)(k * i + b);
        }
        // 右侧：右上→右下连线
        k = CalcSlope(R_h.row, R_h.col, R_l.row, R_l.col);
        b = CalcIntercept(R_h.row, R_h.col, R_l.row, R_l.col);
        for (int i = R_h.row; i <= R_l.row; i++) {
            Right[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则2: 左上+左下+右上 ----
    if (L_h.found && L_l.found && R_h.found) {
        k = CalcSlope(L_h.row, L_h.col, L_l.row, L_l.col);
        b = CalcIntercept(L_h.row, L_h.col, L_l.row, L_l.col);
        for (int i = L_h.row; i <= L_l.row; i++) {
            Left[i] = (int)(k * i + b);
        }
        k = CalcSlope(R_h.row, R_h.col, bottom_row, bottom_right_col);
        b = CalcIntercept(R_h.row, R_h.col, bottom_row, bottom_right_col);
        for (int i = R_h.row; i <= bottom_row; i++) {
            Right[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则3: 右上+右下+左上 ----
    if (R_h.found && R_l.found && L_h.found) {
        k = CalcSlope(R_h.row, R_h.col, R_l.row, R_l.col);
        b = CalcIntercept(R_h.row, R_h.col, R_l.row, R_l.col);
        for (int i = R_h.row; i <= R_l.row; i++) {
            Right[i] = (int)(k * i + b);
        }
        k = CalcSlope(L_h.row, L_h.col, bottom_row, bottom_left_col);
        b = CalcIntercept(L_h.row, L_h.col, bottom_row, bottom_left_col);
        for (int i = L_h.row; i <= bottom_row; i++) {
            Left[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则4: 仅左上+右上（下有丢线）----
    if (L_h.found && R_h.found && !L_l.found && !R_l.found) {
        k = CalcSlope(L_h.row, L_h.col, bottom_row, bottom_left_col);
        b = CalcIntercept(L_h.row, L_h.col, bottom_row, bottom_left_col);
        for (int i = L_h.row; i <= bottom_row; i++) {
            Left[i] = (int)(k * i + b);
        }
        k = CalcSlope(R_h.row, R_h.col, bottom_row, bottom_right_col);
        b = CalcIntercept(R_h.row, R_h.col, bottom_row, bottom_right_col);
        for (int i = R_h.row; i <= bottom_row; i++) {
            Right[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则5: 仅左上+左下 ----
    if (L_h.found && L_l.found && !R_h.found) {
        k = CalcSlope(L_h.row, L_h.col, L_l.row, L_l.col);
        b = CalcIntercept(L_h.row, L_h.col, L_l.row, L_l.col);
        for (int i = L_h.row; i <= L_l.row; i++) {
            Left[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则6: 仅右上+右下 ----
    if (R_h.found && R_l.found && !L_h.found) {
        k = CalcSlope(R_h.row, R_h.col, R_l.row, R_l.col);
        b = CalcIntercept(R_h.row, R_h.col, R_l.row, R_l.col);
        for (int i = R_h.row; i <= R_l.row; i++) {
            Right[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则7: 仅左上（且双边丢线）----
    if (L_h.found && !L_l.found && !R_h.found && !R_l.found) {
        k = CalcSlope(L_h.row, L_h.col, bottom_row, bottom_left_col);
        b = CalcIntercept(L_h.row, L_h.col, bottom_row, bottom_left_col);
        for (int i = L_h.row; i <= bottom_row; i++) {
            Left[i] = (int)(k * i + b);
        }
        return;
    }

    // ---- 规则8: 仅右上（且双边丢线）----
    if (R_h.found && !R_l.found && !L_h.found && !L_l.found) {
        k = CalcSlope(R_h.row, R_h.col, bottom_row, bottom_right_col);
        b = CalcIntercept(R_h.row, R_h.col, bottom_row, bottom_right_col);
        for (int i = R_h.row; i <= bottom_row; i++) {
            Right[i] = (int)(k * i + b);
        }
        return;
    }
}

// ============================================================
// ProcessFrame — 每帧处理入口
//
// 前置条件：otsu_threshold 已由大津法从原始灰度图计算得到
// 后置条件：Dir_err 已就绪，Left/Right/Mid 数组已更新，
//           line_binary 包含边缘补偿后的二值化图像
// ============================================================
void ProcessFrame(uint8 otsu_threshold,
                  const uint8 gray_img[LINE_IMG_H][LINE_IMG_W])
{
    // Step 1: 区域阈值补偿 + 二值化
    ApplyEdgeCompensation(otsu_threshold, gray_img);

    // Step 2: 初始化全局数组
    InitImgInfo();

    // Step 3: 找截止行 + 最长白列
    FindImageTop();

    // Step 4: 画黑边（需在 FindImageTop 之后，使用正确的 imgTop）
    DrawFrameBorder();

    // Step 5: 逐行搜索左右边线（底行已在 FindImageTop 中搜索，从倒数第二行开始）
    FindSidelines(LINE_IMG_H - 1, imgTop + 1);

#if ENABLE_GUAI_DETECTION
    // Step 6a: 拐点检测
    GuaiPoint L_h, L_l, R_h, R_l;
    FindGuaidians(&L_h, &L_l, &R_h, &R_l);

    // Step 6b: 补线（修改 Left/Right 数组）
    Buxian(L_h, L_l, R_h, R_l);

    // 补线后重算赛道宽度（不重跑 FindSidelines，避免覆盖补线结果）
    if (L_h.found || L_l.found || R_h.found || R_l.found) {
        for (int row = LINE_IMG_H - 1; row > imgTop; row--) {
            WhiteWidth[row] = Right[row] - Left[row];
        }
    }
#endif

    // Step 7: 计算中线
    FindMidline();

    // Step 8: 计算方向误差
    CalculateError();

    // 至此，Dir_err 已就绪，可传递给 PID 控制器
    // Dir_err > 0 → 左转，Dir_err < 0 → 右转
}
