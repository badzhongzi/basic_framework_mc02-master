#include "ec_matrix.h"

#include <math.h>
#include <string.h>

static bool valid(const EC_Matrix *m) { return m && m->rows>0 && m->cols>0 && m->rows<=EC_MATRIX_MAX_ROWS && m->cols<=EC_MATRIX_MAX_COLS; }
bool ec_matrix_init(EC_Matrix *m,size_t rows,size_t cols){if(!m||rows==0||cols==0||rows>EC_MATRIX_MAX_ROWS||cols>EC_MATRIX_MAX_COLS)return false;m->rows=rows;m->cols=cols;ec_matrix_zero(m);return true;}
void ec_matrix_zero(EC_Matrix *m){if(m)memset(m->v,0,sizeof(m->v));}
void ec_matrix_identity(EC_Matrix *m){size_t i,n;if(!valid(m))return;ec_matrix_zero(m);n=m->rows<m->cols?m->rows:m->cols;for(i=0;i<n;++i)m->v[i*m->cols+i]=1;}
float ec_matrix_get(const EC_Matrix *m,size_t r,size_t c){if(!valid(m)||r>=m->rows||c>=m->cols)return 0;return m->v[r*m->cols+c];}
bool ec_matrix_set(EC_Matrix *m,size_t r,size_t c,float value){if(!valid(m)||r>=m->rows||c>=m->cols)return false;m->v[r*m->cols+c]=value;return true;}
bool ec_matrix_add(EC_Matrix *out,const EC_Matrix *a,const EC_Matrix *b){size_t i,n;if(!valid(out)||!valid(a)||!valid(b)||a->rows!=b->rows||a->cols!=b->cols||out->rows!=a->rows||out->cols!=a->cols)return false;n=a->rows*a->cols;for(i=0;i<n;++i)out->v[i]=a->v[i]+b->v[i];return true;}
bool ec_matrix_sub(EC_Matrix *out,const EC_Matrix *a,const EC_Matrix *b){size_t i,n;if(!valid(out)||!valid(a)||!valid(b)||a->rows!=b->rows||a->cols!=b->cols||out->rows!=a->rows||out->cols!=a->cols)return false;n=a->rows*a->cols;for(i=0;i<n;++i)out->v[i]=a->v[i]-b->v[i];return true;}
bool ec_matrix_scale(EC_Matrix *out,const EC_Matrix *a,float scalar){size_t i,n;if(!valid(out)||!valid(a)||out->rows!=a->rows||out->cols!=a->cols)return false;n=a->rows*a->cols;for(i=0;i<n;++i)out->v[i]=a->v[i]*scalar;return true;}
bool ec_matrix_mul(EC_Matrix *out,const EC_Matrix *a,const EC_Matrix *b){EC_Matrix temporary;EC_Matrix *target=out;size_t i,j,k,n;if(!valid(out)||!valid(a)||!valid(b)||a->cols!=b->rows||out->rows!=a->rows||out->cols!=b->cols)return false;if(out==a||out==b){temporary.rows=out->rows;temporary.cols=out->cols;target=&temporary;}ec_matrix_zero(target);for(i=0;i<a->rows;++i)for(j=0;j<b->cols;++j)for(k=0;k<a->cols;++k)target->v[i*target->cols+j]+=a->v[i*a->cols+k]*b->v[k*b->cols+j];if(target!=out){n=out->rows*out->cols;memcpy(out->v,target->v,n*sizeof(float));}return true;}
bool ec_matrix_transpose(EC_Matrix *out,const EC_Matrix *input){size_t i,j;if(!valid(out)||!valid(input)||out->rows!=input->cols||out->cols!=input->rows)return false;if(out==input){for(i=0;i<input->rows;++i)for(j=i+1;j<input->cols;++j){float value=out->v[i*out->cols+j];out->v[i*out->cols+j]=out->v[j*out->cols+i];out->v[j*out->cols+i]=value;}return true;}for(i=0;i<input->rows;++i)for(j=0;j<input->cols;++j)out->v[j*out->cols+i]=input->v[i*input->cols+j];return true;}
float ec_matrix_trace(const EC_Matrix *m){size_t i,n;float out=0;if(!valid(m))return 0;n=m->rows<m->cols?m->rows:m->cols;for(i=0;i<n;++i)out+=m->v[i*m->cols+i];return out;}
float ec_matrix_norm(const EC_Matrix *m){size_t i,n;float out=0;if(!valid(m))return 0;n=m->rows*m->cols;for(i=0;i<n;++i)out+=m->v[i]*m->v[i];return sqrtf(out);}
