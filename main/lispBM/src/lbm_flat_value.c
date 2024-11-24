/*
    Copyright 2023, 2024 Joel Svensson    svenssonjoel@yahoo.se
              2023       Benjamin Vedder

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <lbm_flat_value.h>
#include <eval_cps.h>
#include <stack.h>

#include <setjmp.h>

// ------------------------------------------------------------
// Access to GC from eval_cps
int lbm_perform_gc(void);


// ------------------------------------------------------------
// Flatteners

bool lbm_start_flatten(lbm_flat_value_t *v, size_t buffer_size) {
  bool res = false;
  uint8_t *data = lbm_malloc_reserve(buffer_size);
  if (data) {
    v->buf = data;
    v->buf_size = buffer_size;
    v->buf_pos = 0;
    res = true;
  }
  return res;
}

bool lbm_finish_flatten(lbm_flat_value_t *v) {
  lbm_uint size_words;
  if (v->buf_pos % sizeof(lbm_uint) == 0) {
    size_words = v->buf_pos / sizeof(lbm_uint);
  } else {
    size_words = (v->buf_pos / sizeof(lbm_uint)) + 1;
  }
  if (v->buf_size  <= size_words * sizeof(lbm_uint)) return true;
  v->buf_size = size_words * sizeof(lbm_uint);
  return (lbm_memory_shrink((lbm_uint*)v->buf, size_words) >= 0);
}

static bool write_byte(lbm_flat_value_t *v, uint8_t b) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + 1) {
    v->buf[v->buf_pos++] = b;
    res = true;
  }
  return res;
}

static bool write_bytes(lbm_flat_value_t *v, uint8_t *data,lbm_uint num_bytes) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + num_bytes) {
    memcpy(v->buf + v->buf_pos, data, num_bytes);
    v->buf_pos += num_bytes;
    res = true;
  }
  return res;
}

static bool write_word(lbm_flat_value_t *v, uint32_t w) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + 4) {
    v->buf[v->buf_pos++] = (uint8_t)(w >> 24);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 16);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 8);
    v->buf[v->buf_pos++] = (uint8_t)w;
    res = true;
  }
  return res;
}

static bool write_dword(lbm_flat_value_t *v, uint64_t w) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + 8) {
    v->buf[v->buf_pos++] = (uint8_t)(w >> 56);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 48);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 40);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 32);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 24);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 16);
    v->buf[v->buf_pos++] = (uint8_t)(w >> 8);
    v->buf[v->buf_pos++] = (uint8_t)w;
    res = true;
  }
  return res;
}

bool f_cons(lbm_flat_value_t *v) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + 1) {
    v->buf[v->buf_pos++] = S_CONS;
    res = true;
  }
  return res;
}

bool f_lisp_array(lbm_flat_value_t *v, uint32_t size) {
  // arrays are smaller than 2^32 elements long
  bool res = true;
  res = res && write_byte(v, S_LBM_LISP_ARRAY);
  res = res && write_word(v, size); // number of elements.
  return res;
}

bool f_sym(lbm_flat_value_t *v, lbm_uint sym_id) {
  bool res = true;
  res = res && write_byte(v,S_SYM_VALUE);
  #ifndef LBM64
  res = res && write_word(v,sym_id);
  #else
  res = res && write_dword(v,sym_id);
  #endif
  return res;
}

bool f_sym_string(lbm_flat_value_t *v, char *str) {
  bool res = false;
  if (str) {
    lbm_uint sym_bytes = strlen(str) + 1;
    if (write_byte(v, S_SYM_STRING) &&
	write_bytes(v, (uint8_t*)str, sym_bytes)) {
      res = true;
    }
  }
  return res;
}

// Potentially a difference between 32/64 bit version.
// strlen returns size_t which is different on 32/64 bit platforms.
int f_sym_string_bytes(lbm_value sym) {
  int res = FLATTEN_VALUE_ERROR_FATAL;
  if (lbm_is_symbol(sym)) {
    lbm_uint s = lbm_dec_sym(sym);
    char *sym_str = (char*)lbm_get_name_by_symbol(s);
    if (sym_str) {
      lbm_uint sym_bytes = strlen(sym_str) + 1;
      res = (int)sym_bytes;
    }
  }
  return res;
}

bool f_i(lbm_flat_value_t *v, lbm_int i) {
  bool res = true;
#ifndef LBM64
  res = res && write_byte(v,S_I28_VALUE);
  res = res && write_word(v,(uint32_t)i);
#else
  res = res && write_byte(v,S_I56_VALUE);
  res = res && write_dword(v, (uint64_t)i);
#endif
  return res;
}

bool f_u(lbm_flat_value_t *v, lbm_uint u) {
  bool res = true;
#ifndef LBM64
  res = res && write_byte(v,S_U28_VALUE);
  res = res && write_word(v,(uint32_t)u);
#else
  res = res && write_byte(v,S_U56_VALUE);
  res = res && write_dword(v,(uint64_t)u);
#endif
  return res;
}

bool f_b(lbm_flat_value_t *v, uint8_t b) {
  bool res = true;
  res = res && write_byte(v,S_BYTE_VALUE);
  res = res && write_byte(v,b);
  return res;
}

bool f_i32(lbm_flat_value_t *v, int32_t w) {
  bool res = true;
  res = res && write_byte(v, S_I32_VALUE);
  res = res && write_word(v, (uint32_t)w);
  return res;
}

bool f_u32(lbm_flat_value_t *v, uint32_t w) {
  bool res = true;
  res = res && write_byte(v, S_U32_VALUE);
  res = res && write_word(v, w);
  return res;
}

bool f_float(lbm_flat_value_t *v, float f) {
  bool res = true;
  res = res && write_byte(v, S_FLOAT_VALUE);
  uint32_t u;
  memcpy(&u, &f, sizeof(uint32_t));
  res = res && write_word(v, (uint32_t)u);
  return res;
}

bool f_double(lbm_flat_value_t *v, double d) {
  bool res = true;
  res = res && write_byte(v, S_DOUBLE_VALUE);
  uint64_t u;
  memcpy(&u, &d, sizeof(uint64_t));
  res = res && write_dword(v, u);
  return res;
}

bool f_i64(lbm_flat_value_t *v, int64_t w) {
  bool res = true;
  res = res && write_byte(v, S_I64_VALUE);
  res = res && write_dword(v, (uint64_t)w);
  return res;
}

bool f_u64(lbm_flat_value_t *v, uint64_t w) {
  bool res = true;
  res = res && write_byte(v, S_U64_VALUE);
  res = res && write_dword(v, w);
  return res;
}

// num_bytes is specifically an uint32_t
bool f_lbm_array(lbm_flat_value_t *v, uint32_t num_bytes, uint8_t *data) {
  bool res = write_byte(v, S_LBM_ARRAY);
  res = res && write_word(v, num_bytes);
  res = res && write_bytes(v, data, num_bytes);
  return res;
}

static int flatten_maximum_depth = FLATTEN_VALUE_MAXIMUM_DEPTH;

void lbm_set_max_flatten_depth(int depth) {
  flatten_maximum_depth = depth;
}

void flatten_error(jmp_buf jb, int val) {
  longjmp(jb, val);
}

int flatten_value_size_internal(jmp_buf jb, lbm_value v, int depth) {
  if (depth > flatten_maximum_depth) {
    flatten_error(jb, FLATTEN_VALUE_ERROR_MAXIMUM_DEPTH);
  }

  lbm_uint t = lbm_type_of(v);
  if (t >= LBM_POINTER_TYPE_FIRST && t < LBM_POINTER_TYPE_LAST) {
    //  Clear constant bit, it is irrelevant to flattening
    t = t & ~(LBM_PTR_TO_CONSTANT_BIT);
  }

  switch (t) {
  case LBM_TYPE_CONS: {
    int res = 0;
    int s1 = flatten_value_size_internal(jb,lbm_car(v), depth + 1);
    if (s1 > 0) {
      int s2 = flatten_value_size_internal(jb,lbm_cdr(v), depth + 1);
      if (s2 > 0) {
        res = (1 + s1 + s2);
      }
    }
    return res;
  }
  case LBM_TYPE_LISPARRAY: {
    int sum = 4 + 1; // sizeof(uint32_t) + 1;
    lbm_array_header_t *header = (lbm_array_header_t*)lbm_car(v);
    lbm_value *arrdata = (lbm_value*)header->data;
    lbm_uint size = header->size / sizeof(lbm_value);
    for (lbm_uint i = 0; i < size; i ++ ) {
      sum += flatten_value_size_internal(jb, arrdata[i], depth + 1);
    }
    return sum;
  }
  case LBM_TYPE_BYTE:
    return 1 + 1;
  case LBM_TYPE_U: /* fall through */
  case LBM_TYPE_I:
#ifndef LBM64
    return 1 + 4;
#else
    return 1 + 8;
#endif
  case LBM_TYPE_U32: /* fall through */
  case LBM_TYPE_I32:
  case LBM_TYPE_FLOAT:
    return 1 + 4;
  case LBM_TYPE_U64: /* fall through */
  case LBM_TYPE_I64:
  case LBM_TYPE_DOUBLE:
    return 1 + 8;
  case LBM_TYPE_SYMBOL: {
    int s = f_sym_string_bytes(v);
    if (s > 0) return 1 + s;
    flatten_error(jb, (int)s);
  } return 0; // already terminated with error
  case LBM_TYPE_ARRAY: {
    // Platform dependent size.
    // TODO: Something needs to be done to these inconsistencies.
    lbm_int s = lbm_heap_array_get_size(v);
    if (s > 0)
      return 1 + 4 + (int)s;
    flatten_error(jb, (int)s);
  } return 0; // already terminated with error
  default:
    return FLATTEN_VALUE_ERROR_CANNOT_BE_FLATTENED;
  }
}

int flatten_value_size(lbm_value v, int depth) {
  jmp_buf jb;
  int r = setjmp(jb);
  if (r != 0) {
    return r;
  }
  return flatten_value_size_internal(jb, v, depth);
}

int flatten_value_c(lbm_flat_value_t *fv, lbm_value v) {

  lbm_uint t = lbm_type_of(v);
  if (t >= LBM_POINTER_TYPE_FIRST && t < LBM_POINTER_TYPE_LAST) {
    //  Clear constant bit, it is irrelevant to flattening
    t = t & ~(LBM_PTR_TO_CONSTANT_BIT);
  }

  switch (t) {
  case LBM_TYPE_CONS: {
    bool res = true;
    res = res && f_cons(fv);
    if (res) {
      int fv_r = flatten_value_c(fv, lbm_car(v));
      if (fv_r == FLATTEN_VALUE_OK) {
        fv_r = flatten_value_c(fv, lbm_cdr(v));
      }
      return fv_r;
    }
  }break;
  case LBM_TYPE_LISPARRAY: {
    lbm_array_header_t *header = (lbm_array_header_t*)lbm_car(v);
    lbm_value *arrdata = (lbm_value*)header->data;
    lbm_uint size = header->size / sizeof(lbm_value);
    if (!f_lisp_array(fv, size)) return FLATTEN_VALUE_ERROR_NOT_ENOUGH_MEMORY;
    int fv_r = FLATTEN_VALUE_OK;
    for (lbm_uint i = 0; i < size; i ++ ) {
      fv_r =  flatten_value_c(fv, arrdata[i]);
      if (fv_r != FLATTEN_VALUE_OK) {
        break;
      }
    }
    return fv_r;
  } break;
  case LBM_TYPE_BYTE:
    if (f_b(fv, (uint8_t)lbm_dec_as_char(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_U:
    if (f_u(fv, lbm_dec_u(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_I:
    if (f_i(fv, lbm_dec_i(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_U32:
    if (f_u32(fv, lbm_dec_as_u32(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_I32:
    if (f_i32(fv, lbm_dec_as_i32(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_U64:
    if (f_u64(fv, lbm_dec_as_u64(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_I64:
    if (f_i64(fv, lbm_dec_as_i64(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_FLOAT:
    if (f_float(fv, lbm_dec_as_float(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_DOUBLE:
    if (f_double(fv, lbm_dec_as_double(v))) {
      return FLATTEN_VALUE_OK;
    }
    break;
  case LBM_TYPE_SYMBOL: {
    char *sym_str = (char*)lbm_get_name_by_symbol(lbm_dec_sym(v));
    if (f_sym_string(fv, sym_str)) {
      return FLATTEN_VALUE_OK;
    }
  } break;
  case LBM_TYPE_ARRAY: {
    lbm_int s = lbm_heap_array_get_size(v);
    const uint8_t *d = lbm_heap_array_get_data_ro(v);
    if (s > 0 && d != NULL) {
      if (f_lbm_array(fv, (uint32_t)s, (uint8_t*)d)) {
        return FLATTEN_VALUE_OK;
      }
    } else {
      return FLATTEN_VALUE_ERROR_ARRAY;
    }
  }break;
  default:
    return FLATTEN_VALUE_ERROR_CANNOT_BE_FLATTENED;
  }
  return FLATTEN_VALUE_ERROR_BUFFER_TOO_SMALL;
}

lbm_value handle_flatten_error(int err_val) {
  switch (err_val) {
  case FLATTEN_VALUE_ERROR_CANNOT_BE_FLATTENED:
    return ENC_SYM_EERROR;
  case FLATTEN_VALUE_ERROR_BUFFER_TOO_SMALL: /* fall through */
  case FLATTEN_VALUE_ERROR_FATAL:
    return ENC_SYM_FATAL_ERROR;
  case FLATTEN_VALUE_ERROR_CIRCULAR: /* fall through */
  case FLATTEN_VALUE_ERROR_MAXIMUM_DEPTH:
    return ENC_SYM_EERROR;
  case FLATTEN_VALUE_ERROR_ARRAY: /* fall through */
  case FLATTEN_VALUE_ERROR_NOT_ENOUGH_MEMORY:
    return ENC_SYM_MERROR;
  }
  return ENC_SYM_NIL;
}

lbm_value flatten_value(lbm_value v) {

  lbm_value array_cell = lbm_heap_allocate_cell(LBM_TYPE_CONS, ENC_SYM_NIL, ENC_SYM_ARRAY_TYPE);

  if (array_cell == ENC_SYM_MERROR) {
    return array_cell;
  }

  lbm_flat_value_t fv;

  lbm_array_header_t *array = NULL;
  int required_mem = flatten_value_size(v, 0);
  if (required_mem > 0) {
    array = (lbm_array_header_t *)lbm_malloc(sizeof(lbm_array_header_t));
    if (array == NULL) {
      lbm_set_car_and_cdr(array_cell, ENC_SYM_NIL, ENC_SYM_NIL);
      return ENC_SYM_MERROR;
    }

    bool r = lbm_start_flatten(&fv, (lbm_uint)required_mem);
    if (!r) {
      lbm_free(array);
      lbm_set_car_and_cdr(array_cell, ENC_SYM_NIL, ENC_SYM_NIL);
      return ENC_SYM_MERROR;
    }

    if (flatten_value_c(&fv, v) == FLATTEN_VALUE_OK) {
      // it would be wasteful to run finish_flatten here.
      r = true;
    } else {
      r = false;
    }

    if (r)  {
      // lift flat_value
      array->data = (lbm_uint*)fv.buf;
      array->size = fv.buf_size;
      lbm_set_car(array_cell, (lbm_uint)array);
      array_cell = lbm_set_ptr_type(array_cell, LBM_TYPE_ARRAY);
      return array_cell;
    } 
  }
  lbm_set_car_and_cdr(array_cell, ENC_SYM_NIL, ENC_SYM_NIL);
  return handle_flatten_error(required_mem);
}

// ------------------------------------------------------------
// Unflattening
static bool extract_byte(lbm_flat_value_t *v, uint8_t *r) {
  if (v->buf_size >= v->buf_pos + 1) {
    *r = v->buf[v->buf_pos++];
    return true;
  }
  return false;
}

static bool extract_word(lbm_flat_value_t *v, uint32_t *r) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + 4) {
    uint32_t tmp = 0;
    tmp |= (lbm_value)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint32_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint32_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint32_t)v->buf[v->buf_pos++];
    *r = tmp;
    res = true;
  }
  return res;
}

static bool extract_dword(lbm_flat_value_t *v, uint64_t *r) {
  bool res = false;
  if (v->buf_size >= v->buf_pos + 8) {
    uint64_t tmp = 0;
    tmp |= (lbm_value)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    tmp = tmp << 8 | (uint64_t)v->buf[v->buf_pos++];
    *r = tmp;
    res = true;;
  }
  return res;
}

/* Recursive and potentially stack hungry for large flat values */
static int lbm_unflatten_value_internal(lbm_flat_value_t *v, lbm_value *res) {
  if (v->buf_size == v->buf_pos) return UNFLATTEN_MALFORMED;

  uint8_t curr = v->buf[v->buf_pos++];

  switch(curr) {
  case S_CONS: {
    lbm_value a;
    lbm_value b;
    int r = lbm_unflatten_value_internal(v, &a);
    if (r == UNFLATTEN_OK) {
      r = lbm_unflatten_value_internal(v, &b);
      if (r == UNFLATTEN_OK) {
        lbm_value c;
        c = lbm_cons(a,b);
        if (lbm_is_symbol_merror(c)) return UNFLATTEN_GC_RETRY;
        *res = c;
        r = UNFLATTEN_OK;
      }
    }
    return r;
  }
  case S_LBM_LISP_ARRAY: {
    uint32_t size;
    bool b = extract_word(v, &size);
    int r = UNFLATTEN_MALFORMED;
    if (b) {
      lbm_value array;
      lbm_heap_allocate_lisp_array(&array, size);
      lbm_array_header_t *header = (lbm_array_header_t*)lbm_car(array);
      lbm_value *arrdata = (lbm_value*)header->data;
      if (lbm_is_symbol_merror(array)) return UNFLATTEN_GC_RETRY;
      lbm_value a;
      for (uint32_t i = 0; i < size; i ++) {
        r = lbm_unflatten_value_internal(v, &a);
        if (r == UNFLATTEN_OK) {
          arrdata[i] = a;
        } else {
          break;
        }
      }
      *res = array;
    }
    return r;
  }
  case S_SYM_VALUE: {
    lbm_uint tmp;
    bool b;
#ifndef LBM64
    b = extract_word(v, &tmp);
#else
    b = extract_dword(v, &tmp);
#endif
    if (b) {
      *res = lbm_enc_sym(tmp);
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_BYTE_VALUE: {
    uint8_t tmp;
    bool b = extract_byte(v, &tmp);
    if (b) {
      *res = lbm_enc_char((uint8_t)tmp);
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_I28_VALUE: {
    uint32_t tmp;
    bool b;
    b = extract_word(v, &tmp);
    if (b) {
      *res = lbm_enc_i((int32_t)tmp);
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_U28_VALUE: {
    uint32_t tmp;
    bool b;
    b = extract_word(v, &tmp);
    if (b) {
      *res = lbm_enc_u((uint32_t)tmp);
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_I56_VALUE: {
    uint64_t tmp;
    bool b;
    b = extract_dword(v, &tmp);
    if (b) {
#ifndef LBM64
      *res = lbm_enc_i64((int64_t)tmp);
#else
      *res = lbm_enc_i((int64_t)tmp);
#endif
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_U56_VALUE: {
    uint64_t tmp;
    bool b;
    b = extract_dword(v, &tmp);
    if (b) {
#ifndef LBM64
      *res = lbm_enc_u64(tmp);
#else
      *res = lbm_enc_u(tmp);
#endif
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_FLOAT_VALUE: {
    uint32_t tmp;
    bool b;
    b = extract_word(v, &tmp);
    if (b) {
      lbm_float f;
      memcpy(&f, &tmp, sizeof(lbm_float));
      lbm_value im  = lbm_enc_float(f);
      if (lbm_is_symbol_merror(im)) {
        return UNFLATTEN_GC_RETRY;
      }
      *res = im;
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_DOUBLE_VALUE: {
    uint64_t tmp;
    bool b;
    b = extract_dword(v, &tmp);
    if (b) {
      double f;
      memcpy(&f, &tmp, sizeof(uint64_t));
      lbm_value im  = lbm_enc_double(f);
      if (lbm_is_symbol_merror(im)) {
        return UNFLATTEN_GC_RETRY;
      }
      *res = im;
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_I32_VALUE: {
   uint32_t tmp;
    if (extract_word(v, &tmp)) {
      lbm_value im = lbm_enc_i32((int32_t)tmp);
      if (lbm_is_symbol_merror(im)) {
        return UNFLATTEN_GC_RETRY;
      }
      *res = im;
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_U32_VALUE: {
    uint32_t tmp;
    if (extract_word(v, &tmp)) {
      lbm_value im = lbm_enc_u32(tmp);
      if (lbm_is_symbol_merror(im)) {
        return UNFLATTEN_GC_RETRY;
      }
      *res = im;
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_I64_VALUE: {
   uint64_t tmp = 0;
    if (extract_dword(v, &tmp)) {
      lbm_value im = lbm_enc_i64((int64_t)tmp);
      if (lbm_is_symbol_merror(im)) {
        return UNFLATTEN_GC_RETRY;
      }
      *res = im;
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_U64_VALUE: {
    uint64_t tmp = 0;
    if (extract_dword(v, &tmp)) {
      lbm_value im = lbm_enc_u64(tmp);
      if (lbm_is_symbol_merror(im)) {
        return UNFLATTEN_GC_RETRY;
      }
      *res = im;
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_LBM_ARRAY: {
    uint32_t num_elt;
    if (extract_word(v, &num_elt)) {
      if (lbm_heap_allocate_array(res, num_elt)) {
        lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(*res);
        lbm_uint num_bytes = num_elt;
        memcpy(arr->data, v->buf + v->buf_pos, num_bytes);
        v->buf_pos += num_bytes;
      } else {
        return UNFLATTEN_GC_RETRY;
      }
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_MALFORMED;
  }
  case S_SYM_STRING: {
    lbm_uint sym_id;
    if (lbm_add_symbol((char *)(v->buf + v->buf_pos), &sym_id)) {
      lbm_uint num_bytes = strlen((char*)(v->buf + v->buf_pos)) + 1;
      v->buf_pos += num_bytes;
      *res = lbm_enc_sym(sym_id);
      return UNFLATTEN_OK;
    }
    return UNFLATTEN_GC_RETRY;
  }
  default:
    return UNFLATTEN_MALFORMED;
  }
}

bool lbm_unflatten_value(lbm_flat_value_t *v, lbm_value *res) {
  bool b = false;
#ifdef LBM_ALWAYS_GC
  lbm_perform_gc();
#endif
  int r = lbm_unflatten_value_internal(v,res);
  if (r == UNFLATTEN_GC_RETRY) {
    lbm_perform_gc();
    v->buf_pos = 0;
    r = lbm_unflatten_value_internal(v,res);
  }
  if (r == UNFLATTEN_MALFORMED) {
    *res = ENC_SYM_EERROR;
  } else if (r == UNFLATTEN_GC_RETRY) {
    *res = ENC_SYM_MERROR;
  } else {
    b = true;
  }
  // Do not free the flat value buffer here.
  // there are 2 cases:
  // 1: unflatten was called from lisp code -> GC removes the buffer.
  // 2: unflatten called from event processing -> event processor frees buffer.
  return b;
}
