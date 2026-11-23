#ifndef CASADI_FLOAT_TRIG_H
#define CASADI_FLOAT_TRIG_H

#include <math.h>

/* CasADi 生成的 float 版 rnea/crba 仍调用 sin()/cos(), 会走 double 库并做
 * float<->double 转换. 仅对 rnea_6axis.o / crba_6axis.o 通过 -include 注入,
 * 重新生成 Generated/ 时不会被覆盖. */
#define sin(x) sinf(x)
#define cos(x) cosf(x)

#endif
