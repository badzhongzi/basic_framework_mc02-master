#ifndef ENGINEER_COMPONENTS_C_MATRIX_H
#define ENGINEER_COMPONENTS_C_MATRIX_H

/*
 * 通用定长矩阵接口。
 *
 * 提供最大 EC_MATRIX_MAX_ROWS × EC_MATRIX_MAX_COLS 的浮点矩阵运算，
 * 实际使用尺寸由 rows/cols 字段指定，不超过上限即可。
 *
 * 特点：
 *   - 数据按行优先存储：元素 (i,j) 存储于 v[i * cols + j]
 *   - 不进行堆内存分配，所有数据内嵌于结构体
 *   - 编译前可通过 EC_MATRIX_MAX_ROWS / EC_MATRIX_MAX_COLS 覆盖上限以节省 RAM
 *   - 使用前必须先调用 ec_matrix_init() 设置有效尺寸
 *
 * 本模块是 ec_math.h 中 Mat3/Mat4 的泛化版本，
 * 适用于雅可比矩阵（6×N）等尺寸不固定的场景。
 * 对于固定 3×3 / 4×4 运算，优先使用 ec_math.h 中的专用函数，性能更高。
 *
 * ────────────────────────────────────────────────────────────────
 * 当前状态：本模块在框架中暂无调用者。
 *
 * 框架核心运算（正运动学、逆解、规划等）全部使用 ec_math.h 中的
 * EC_Mat3 / EC_Mat4 定尺寸矩阵。雅可比矩阵当前通过
 * ec_serial_link_jacobian() 直接输出为裸 float[] 数组（6行×N列，行优先），
 * 未使用 EC_Matrix 封装。
 *
 * 未来扩展备选方案：
 *   若需要为雅可比矩阵（6×N）、质量矩阵（N×N）、科里奥利矩阵（N×N）等
 *   变尺寸矩阵提供类型安全的封装，可将 ec_serial_link_jacobian() 等接口
 *   改为输出 EC_Matrix，例如：
 *
 *     bool ec_serial_link_jacobian(const EC_SerialLink *serial,
 *                                  EC_Matrix *jac);  // 6×N
 *
 *   这样可利用 EC_Matrix 的尺寸字段做边界检查，并通过 ec_matrix_mul() 等
 *   完成雅可比伪逆、惯量逆等后续运算，无需调用方手动管理裸数组的行列布局。
 *
 *   若确认不需要此扩展，可安全删除 ec_matrix.h / ec_matrix.c 及
 *   engineer_components_c.h 中的 #include "ec_matrix.h" 以减小编译体积。
 * ────────────────────────────────────────────────────────────────
 */

#include <stdbool.h>
#include <stddef.h>

#ifndef EC_MATRIX_MAX_ROWS
/* 编译时可覆盖的最大行数；默认 16 覆盖常见机械臂的雅可比矩阵需求（6行×N列）。 */
#define EC_MATRIX_MAX_ROWS 16u
#endif
#ifndef EC_MATRIX_MAX_COLS
/* 编译时可覆盖的最大列数；默认 16 覆盖 EC_MAX_JOINTS 的默认值。 */
#define EC_MATRIX_MAX_COLS 16u
#endif

typedef struct {
  size_t rows;                                      /* 有效行数，由 ec_matrix_init() 设置。 */
  size_t cols;                                      /* 有效列数，由 ec_matrix_init() 设置。 */
  float v[EC_MATRIX_MAX_ROWS * EC_MATRIX_MAX_COLS]; /* 行优先数据；实际使用 rows×cols 个元素，其余未使用。 */
} EC_Matrix;

#ifdef __cplusplus
extern "C" {
#endif

/* ── 初始化 ── */

/* 初始化矩阵的有效尺寸。
 * rows, cols: 必须 > 0 且分别 <= EC_MATRIX_MAX_ROWS / EC_MATRIX_MAX_COLS。
 * 成功返回 true；参数非法或空指针返回 false。
 * 初始化后整个数据缓冲区会被清零；如需单位阵，再调用 ec_matrix_identity()。 */
bool ec_matrix_init(EC_Matrix *matrix, size_t rows, size_t cols);

/* ── 赋值 ── */

/* 将有效区域（rows×cols）全部清零。 */
void ec_matrix_zero(EC_Matrix *matrix);

/* 将矩阵清零并设置可用主对角线元素为 1.0f。
 * 对非方阵，会设置 min(rows, cols) 个对角元素。 */
void ec_matrix_identity(EC_Matrix *matrix);

/* ── 元素访问 ── */

/* 读取元素 (row, col)；越界读取返回 0.0f。 */
float ec_matrix_get(const EC_Matrix *matrix, size_t row, size_t col);

/* 写入元素 (row, col) = value；越界写入返回 false。 */
bool ec_matrix_set(EC_Matrix *matrix, size_t row, size_t col, float value);

/* ── 算术运算 ── */

/* 矩阵加法：out = a + b。
 * out 必须已初始化为与 a、b 相同的尺寸；尺寸不匹配时返回 false。 */
bool ec_matrix_add(EC_Matrix *out, const EC_Matrix *a, const EC_Matrix *b);

/* 矩阵减法：out = a - b。
 * 要求同 ec_matrix_add。 */
bool ec_matrix_sub(EC_Matrix *out, const EC_Matrix *a, const EC_Matrix *b);

/* 矩阵数乘：out = a × scalar。
 * out 尺寸必须与 a 相同。 */
bool ec_matrix_scale(EC_Matrix *out, const EC_Matrix *a, float scalar);

/* 矩阵乘法：out = a × b。
 * 要求 a.cols == b.rows，out 尺寸必须为 a.rows × b.cols；不满足时返回 false。
 * 支持 out 与 a 或 b 指向同一矩阵。 */
bool ec_matrix_mul(EC_Matrix *out, const EC_Matrix *a, const EC_Matrix *b);

/* 转置矩阵：out = inputᵀ。
 * out 必须已初始化为 input.cols × input.rows 尺寸；方阵支持原地转置。 */
bool ec_matrix_transpose(EC_Matrix *out, const EC_Matrix *input);

/* ── 标量度量 ── */

/* 计算主对角线之和：trace = Σ v[i][i]，i = 0..min(rows,cols)-1。 */
float ec_matrix_trace(const EC_Matrix *matrix);

/* 计算 Frobenius 范数：‖A‖_F = sqrt(Σᵢⱼ aᵢⱼ²)。 */
float ec_matrix_norm(const EC_Matrix *matrix);

#ifdef __cplusplus
}
#endif

#endif
