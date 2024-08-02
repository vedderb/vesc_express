/*
 *  Lowzip -- memory-optimized unzip library with built-in inflate support.
 *
 *  About ZIP parsing
 *  =================
 *
 *  https://pkware.cachefly.net/webdocs/casestudies/APPNOTE.TXT
 *
 *  About the inflate algorithm
 *  ===========================
 *
 *  While the algorithm is generic, the main use case is decompressing deflate
 *  compressed ZIP file entries.  Main goals are: (1) a small code/memory
 *  footprint and (2) making minimal platform/compiler assumptions to maximize
 *  portability.  Significant speed trade-offs are accepted to minimize
 *  code/memory footprint which is usually the most pressing concern in low
 *  memory targets.
 *
 *  The inflate state structure has a fixed size and is allocated by the
 *  caller.  This avoids any dependencies on external allocation functions,
 *  and minimizes memory churn/fragmentation.  The caller may also be able
 *  to reuse some already allocated but currently unused memory areas for
 *  decompression.
 *
 *  Input data is read using a user callback.  This allows input to be
 *  streamed off flash without any buffering -- though a reasonably sized
 *  temporary buffer (e.g. 64 bytes) should be used by the callback itself.
 *  The callback must detect end of input and return 0x100 to signify an out
 *  of bounds read.  Such reads may happen for broken inputs, and the inflate
 *  algorithm may attempt any number of such reads which must be handled in
 *  a memory safe manner, always returning 0x100.
 *
 *  Output data is written out into a user provided fixed buffer.  If the
 *  output buffer is too small for the output, the inflate algorithm remains
 *  memory safe and won't overstep the buffer, and an error will be signalled.
 *  Needing to know the output length (or an upwards estimate of it) limits
 *  some use cases, but is not an issue for ZIP files where the uncompressed
 *  size is described by ZIP file headers.  The output area is also used by
 *  the inflate algorithm for backwards references so that a separate state is
 *  not needed for them.
 *
 *  Besides ZIP files, the inflate function can be used for any other inputs
 *  where the output size is known, can be estimated, or can be limited to a
 *  certain maximum value.
 *
 *  https://www.ietf.org/rfc/rfc1951.txt
 *
 *  Type assumptions
 *  ================
 *
 *    - sizeof(char) == 1
 *    - sizeof(short) == 2
 *    - sizeof(int) >= 4
 */

#include <string.h>  /* memset(), strlen() */
#include <stddef.h>  /* ptrdiff_t */
#include "lowzip.h"

/*
 *  ZIP defines (see ZIP APPNOTE)
 */

#define LOWZIP_MIN_EOCDIR_LENGTH     22
#define LOWZIP_MAX_EOCDIR_LENGTH     (65535L + 22L)
#define LOWZIP_MIN_CDIRFILE_LENGTH   46
#define LOWZIP_MIN_LOCFILE_LENGTH    30
#define LOWZIP_COMPRESSION_STORE     0
#define LOWZIP_COMPRESSION_DEFLATE   8

/*
 *  Inflate defines and tables
 */

/* Scratch area offset for literal/length Huffman tree. */
#define LOWZIP_SCRATCH_HUFF_LIT   0

/* Scratch area offset for distance Huffman tree. */
#define LOWZIP_SCRATCH_HUFF_DIST  604

/* Extra bits for 'length', from RFC 1951 Section 3.2.5.  Index is code - 257,
 * value is extra bits to read for the length value.
 */
static const unsigned char lowzip_len_bits[29] = {
		0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 1U, 1U, 1U, 2U, 2U, 2U, 2U,
		3U, 3U, 3U, 3U, 4U, 4U, 4U, 4U, 5U, 5U, 5U, 5U, 0U
};

/* Base value for 'length', from RFC 1951 Section 3.2.5.  Index is code - 257,
 * value is lowest length value for that code - 3.  The subtraction is used so
 * that the maximum value 258 - 3 = 255 fits into 1 byte, which saves 19 bytes
 * of footprint minus an 'add 3' at the single lookup site.
 */
static const unsigned char lowzip_len_base[29] = {
		0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 10U, 12U, 14U, 16U, 20U, 24U, 28U,
		32U, 40U, 48U, 56U, 64U, 80U, 96U, 112U, 128U, 160U, 192U, 224U, 255U
};

/* Extra bits for 'distance', from RFC 1951 Section 3.2.5.  Index is code,
 * value is extra bits to read for the distance value.
 */
static const unsigned char lowzip_dist_bits[30] = {
		0U, 0U, 0U, 0U, 1U, 1U, 2U, 2U, 3U, 3U, 4U, 4U, 5U, 5U, 6U, 6U, 7U, 7U,
		8U, 8U, 9U, 9U, 10U, 10U, 11U, 11U, 12U, 12U, 13U, 13U
};

/* Base value for 'distance', from RFC 1951 Section 3.2.5.  Index is code,
 * value is lowest distance value for that code.
 */
static const unsigned short lowzip_dist_base[30] = {
		1U, 2U, 3U, 4U, 5U, 7U, 9U, 13U, 17U, 25U, 33U, 49U, 65U, 97U, 129U,
		193U, 257U, 385U, 513U, 769U, 1025U, 1537U, 2049U, 3073U, 4097U,
		6145U, 8193U, 12289U, 16385U, 24577U
};

/* Permutation order for code length alphabet, RFC 1951 Section 3.2.7. */
static const unsigned char lowzip_codelen_order[19] = {
		16U, 17U, 18U, 0U, 8U, 7U, 9U, 6U, 10U, 5U, 11U, 4U, 12U, 3U, 13U, 2U,
		14U, 1U, 15U
};

/*
 *  Read/write helpers
 */

/* Write an output byte.  If end of output encountered, flag an error and
 * do nothing.
 */
static void lowzip_write_byte(lowzip_state *st, unsigned char ch) {
	if (st->output_next >= st->output_end) {
		st->have_error = 1;
	} else {
		if (st->write_callback) {
			st->write_callback(st->udata_write, ch);
		} else {
			*st->output_next = ch;
		}

		st->output_next++;
	}
}

/* Read an N-byte little-endian value at given offset. */
static unsigned int lowzip_read_little_endian(lowzip_state *st, unsigned int offset, unsigned int count) {
	unsigned int res;
	unsigned int t;

	res = 0;
	while (count-- > 0) {
		t = st->read_callback(st->udata, offset + count);
		if (t & 0x100U) {
			st->have_error = 1;
			res = 0;
			break;
		} else {
			res = (res << 8U) + t;
		}
	}
	return res;
}

/* Read a 4-byte little endian value at given offset. */
static unsigned int lowzip_read4(lowzip_state *st, unsigned int offset) {
	return lowzip_read_little_endian(st, offset, 4);
}

/* Read a 2-byte little endian value at given offset. */
static unsigned int lowzip_read2(lowzip_state *st, unsigned int offset) {
	return lowzip_read_little_endian(st, offset, 2);
}

/* Read a single byte at given offset. */
static unsigned int lowzip_read1(lowzip_state *st, unsigned int offset) {
	return lowzip_read_little_endian(st, offset, 1);
}

/* Read next input byte using st->read_offset.  When out of input, feed in
 * zeroes and flag an error.  The zeroes are processed as if they were in the
 * input (which must be memory safe because such an input might exist without
 * an overrun too), and the error is detected with some delay.
 *
 * If footprint/portability were not a concern, we'd ideally either return an
 * error indication or use a longjmp-like mechanism to bail out directly.
 * However, an error return value would need to be checked by a lot of call
 * sites, and a longjmp (or similar) is a portability concern.
 */
static unsigned int lowzip_read_byte(lowzip_state *st) {
	unsigned int x;

	x = st->read_callback(st->udata, st->read_offset);
	if (!(x & 0x100U)) {
		st->read_offset++;
	} else {
		/* Flag overrun for later detection. */
		st->have_error = 1;
		x = 0;
	}
	return x;
}

/* Read 'nbits' bits from the input in "non-Huffman" order, see RFC 1951
 * Section 3.1.1.
 */
static unsigned int lowzip_read_bits(lowzip_state *st, unsigned int nbits) {
	unsigned int curr;
	unsigned int have;
	unsigned int x;
	unsigned int mask;
	unsigned int res;

	curr = st->curr;
	have = st->have;

	while (have < nbits) {
		x = lowzip_read_byte(st);
		curr |= (x << have);
		have += 8;
	}

	mask = (1U << nbits) - 1U;
	res = curr & mask;
	st->have = have - nbits;
	st->curr = curr >> nbits;

	return res;
}

/* Read 'nbits' bits from the input in "Huffman" order, see RFC 1951
 * Section 3.1.1.  This is only needed by static Huffman block decoding;
 * dynamic Huffman blocks decode one bit at a time.  So a trivial (but
 * slow) algorithm is used.
 */
#if 0  /* This is (a bit surprisingly) larger on x64. */
static unsigned int lowzip_read_bits_reversed(lowzip_state *st, unsigned int nbits) {
	unsigned int res;

	res = 0;
	while (nbits-- > 0) {
		res = (res << 1U) + lowzip_read_bits(st, 1);
	}
	return res;
}
#else
static unsigned int lowzip_read_bits_reversed(lowzip_state *st, unsigned int nbits) {
	unsigned int tmp;
	unsigned int res;
	unsigned int mask1;
	unsigned int mask2;

	tmp = lowzip_read_bits(st, nbits);
	res = 0;
	mask1 = (1U << nbits);
	mask2 = 1U;
	for (;;) {
		mask1 >>= 1U;
		if (mask1 == 0) {
			break;
		}
		if (tmp & mask1) {
			res += mask2;
		}
		mask2 <<= 1U;
	}

	return res;
}
#endif

static void lowzip_reset_bitstate(lowzip_state *st) {
#if 0
	st->curr = 0;  /* Not necessary because of masking. */
#endif
	st->have = 0;
}

/*
 *  Huffman decoding
 */

/* Prepare a Huffman decoding table for a sequence of code lengths.
 * Code lengths are given as 8-bit values in 'code_lens', with
 * 'code_lens_count' elements.
 *
 * Conceptual outputs are (1) "counts" and (2) "codes", see details in
 * inline comments.  Both outputs are written to a single output buffer,
 * 'out_huff', which should be a 2-byte aligned area.
 *
 * Counts: number of terminal codes for each code bit length 0-15.  The
 * counts output always has 16 entries, 2 bytes each for a total of 32 bytes.
 * 16-bit values are needed because it's possible for the code count for
 * a certain length to be larger than 255 (Scriptorium terra.dll is one such
 * input).  The zero length entry is ignored.
 *
 * Codes: there are 'code_lens_count' 16-bit terminal values.  16-bit values
 * are needed for literal/length codes (which go up to 285).  While 8 bits
 * would suffice for distance codes, it's not worth it to support them
 * specially.  The largest literal/length Huffman table is 286 entries,
 * taking 286x2 = 572 bytes.  The largest distance Huffman table is 32
 * entries, taking 32x2 = 64 bytes.
 *
 * Both counts and codes outputs are written to the same output buffer,
 * which represents the Huffman table.  Counts are written first, followed
 * by codes.
 *
 * Maximum literal/length output size is: 32 + 572 = 604 bytes.
 * Maximum distance output size is: 32 + 64 = 96 bytes.
 * Total Huffman table size for literal/length + distance: 700 bytes.
 */
static void lowzip_prepare_huffman(lowzip_state *st,
		unsigned char *code_lens,
		unsigned int code_lens_count,
		unsigned short *out_huff) {     /* See required size in comments above. */
	unsigned int i, j, t;
	unsigned short *out_counts;
	unsigned short *out_codes;

	/* Count number of codes (terminals) for each code length.
	 * Zero length, signifying an unused terminal, is counted but
	 * ignored.
	 */

	out_counts = out_huff;
	memset((void *) out_counts, 0, 16 * sizeof(unsigned short));
	for (i = 0; i < code_lens_count; i++) {
		t = code_lens[i];
		if (t > 15) {
			/* Maximum code length is 15. */
			goto format_error;
		}
		out_counts[t]++;
	}

	/* The 'codes' array is used together with 'counts' in decoding.
	 * It gives the terminal values for each code length tree level
	 * in sequence.
	 *
	 * This is quite slow now because we make a lot of passes over the
	 * code lengths to avoid temporaries.  A faster in-place solution
	 * should be possible.
	 */

	/* XXX: codes are limited to 9 bits, so maybe the highest bit could
	 * be stored into a bit-packed array?  This would save 250 bytes for
	 * the length/literal Huffman table for example, with a footprint
	 * cost of probably less than that.
	 */

	out_codes = out_huff + 16;
	for (i = 1; i <= 15; i++) {  /* Code length (bits), ignore zero length codes. */
		for (j = 0; j < code_lens_count; j++) {  /* Input code lengths. */
			if (code_lens[j] == i) {
				*out_codes++ = j;  /* Index = terminal value. */
			}
		}
	}

	return;

	format_error:
	st->have_error = 1;
}

/* Huffman decode a terminal value from the input. */
static unsigned int lowzip_decode_huffman(lowzip_state *st, unsigned short *huff) {
	unsigned int code;
	unsigned int code_start;
	unsigned short *counts_ptr;
	unsigned int codes_offset;
	int i;

	code = 0;
	code_start = 0;
	codes_offset = 0;
	counts_ptr = huff;  /* Ignore zero length code. */

	for (i = 1; i <= 15; i++) {  /* Valid bit lengths are 1 to 15. */
		unsigned int curr_count;

		code = (code << 1U) + lowzip_read_bits(st, 1);
		curr_count = counts_ptr[i];

		if (code - code_start < curr_count) {
			/* No limit check is needed regardless of input (even
			 * crafted) if counts/codes structures are prepared
			 * correctly.
			 */
			unsigned short *codes_ptr;
			codes_ptr = huff + 16;
			return codes_ptr[codes_offset + code - code_start];
		} else {
			code_start = (code_start + curr_count) << 1U;
			codes_offset += curr_count;
		}
	}

	/* Code didn't terminate within 15 bits.  This is memory safe: we've
	 * exhausted 'counts' but haven't accessed the codes part so it's OK
	 * that codes_offset is potentially out of bounds.
	 */

	st->have_error = 1;
	return 0;
}

/*
 *  Inflate block decoding
 */

/* Decode an uncompressed block. */
static void lowzip_decode_uncompressed_block(lowzip_state *st) {
	unsigned int len;

	/* Discard unused partially read bits. */
	lowzip_reset_bitstate(st);

	/* Parse block length.  Ignore one's complement of length which
	 * is for error checking.  Checking it would be OK but somewhat
	 * pointless because no other part of the deflate stream has any
	 * redundancy checks.
	 */
	len = lowzip_read_byte(st);
	len += lowzip_read_byte(st) << 8U;
	lowzip_read_byte(st);  /* Skip NLEN. */
	lowzip_read_byte(st);

	/* Copy bytes to output verbatim. */
	while (len-- > 0) {
		lowzip_write_byte(st, lowzip_read_byte(st));
	}
}

/* Decode compressed data using static or dynamic length/literal and distance
 * Huffman trees.  Static trees are defined in RFC 1951 Section 3.2.6, decoded
 * manually instead of using an actual tree.
 */
static void lowzip_decode_huffman_block_data(lowzip_state *st, int static_huffman) {
	unsigned int t;

	for (;;) {
		if (st->have_error) {
			break;
		}

		if (!static_huffman) {
			/* Dynamic Huffman. */
			t = lowzip_decode_huffman(st, (unsigned short *) ((unsigned char *) st->scratch + LOWZIP_SCRATCH_HUFF_LIT));
		} else {
			/* Static Huffman, hand-crafted decoder. */
			t = lowzip_read_bits_reversed(st, 7);  /* Minimum code length is 7. */
			if (t <= 0x17U) {
				t += 256;
			} else if (t <= 0x5f) {
				t = (t << 1U) + lowzip_read_bits(st, 1) - 48;
			} else if (t <= 0x63) {
				t = (t << 1U) + lowzip_read_bits(st, 1) + 88;
			} else {
				t = (t << 2U) + lowzip_read_bits_reversed(st, 2) - 256;
			}
		}

		if (t < 256) {
			lowzip_write_byte(st, (unsigned char) t);
		} else if (t == 256) {
			break;
		} else {
			unsigned int back_len;
			unsigned int back_dist;

			if (t > 285) {
				goto format_error;
			}
			t -= 257;

			back_len = (unsigned int) lowzip_len_base[t] + 3U + lowzip_read_bits(st, lowzip_len_bits[t]);

			if (!static_huffman) {
				/* Dynamic Huffman. */
				t = lowzip_decode_huffman(st, (unsigned short *) ((unsigned char *) st->scratch + LOWZIP_SCRATCH_HUFF_DIST));
			} else {
				/* Static Huffman, hand-crafted decoder. */
				t = lowzip_read_bits_reversed(st, 5);  /* Fixed 5-bit code, use as is. */
			}
			if (t > 29) {
				goto format_error;
			}

			back_dist = lowzip_dist_base[t] + lowzip_read_bits(st, lowzip_dist_bits[t]);

			if ((ptrdiff_t) back_dist > (ptrdiff_t) (st->output_next - st->output_start)) {
				/* Back-reference goes too far back. */
				goto format_error;
			}
			if ((ptrdiff_t) back_len > (ptrdiff_t) (st->output_end - st->output_next)) {
				/* Not enough space for output. */
				goto buffer_error;
			}
			/* back_dist cannot be 0: lowzip_dist_base[] entries
			 * are > 0 and the index was checked explicitly.
			 */
#if 0
			if (back_dist == 0) {
				goto format_error;
			}
#endif

			/* Repetition of previous output.  Deflate allows
			 * the repetition input to overlap with the output,
			 * so e.g. distance=2 and length=5 is fine.  By
			 * using a running pointer this gets handled without
			 * any special casing.  Output space has already been
			 * checked for above.
			 */
			while (back_len-- > 0) {
				if (st->write_callback) {
					st->write_callback(st->udata_write, -((int)back_dist));
				} else {
					*st->output_next = *(st->output_next - back_dist);
				}

				st->output_next++;
			}
		}
	}

	return;

	format_error:
	buffer_error:
	st->have_error = 1;
}

/* Decode a static Huffman block.  Conceptually initialize or use a
 * pre-initialized Huffman tree specified in RFC 1951 Section 3.2.6.
 * In practice the static Huffman tree is simple enough to be decoded
 * without an explicit Huffman tree.
 */
static void lowzip_decode_static_huffman_block(lowzip_state *st) {
	lowzip_decode_huffman_block_data(st, 1 /*static_huffman*/);
}

/* Decode a dynamic Huffman block.  Initialize length/literal and distance
 * Huffman trees using a temporary code length Huffman tree.  Then proceed
 * to decode block data using the trees.
 */
static void lowzip_decode_dynamic_huffman_block(lowzip_state *st) {
	unsigned int nlit;
	unsigned int ndist;
	unsigned int nclen;
	unsigned int i;

	unsigned char *codelen_code_lens;
	unsigned char *temp_code_lens;

	/* Decode dynamic Huffman table length fields.
	 *
	 * Maximum values:
	 *   - nlit: 31 + 257 = 288
	 *   - ndist: 31 + 1 = 32
	 *   - nclen: 15 + 4 = 19
	 *   - nlit + ndist: 288 + 320
	 */
	nlit = lowzip_read_bits(st, 5) + 257;
	ndist = lowzip_read_bits(st, 5) + 1;
	nclen = lowzip_read_bits(st, 4) + 4;

	/* Initialize code length alphabet, a small Huffman table used to
	 * init other Huffman tables.  Use the shared 'scratch' area for
	 * temporaries and the Huffman table itself:
	 *
	 *    [0,70[:   code length Huffman table (32 + 19x2 = 70)
	 *    [70,89[:  codelen_code_lens
	 *
	 * Code length alphabet uses codes 0-18.
	 */

	codelen_code_lens = (unsigned char *) ((unsigned char *) st->scratch + 70);
	memset((void *) codelen_code_lens, 0, 19);
	for (i = 0; i < nclen; i++) {
		codelen_code_lens[lowzip_codelen_order[i]] = lowzip_read_bits(st, 3);
	}
	lowzip_prepare_huffman(st,
			codelen_code_lens,
			19,
			(unsigned short *) st->scratch);
	if (st->have_error) {
		/* Quick detect for Huffman prepare failures so that
		 * uninitialized Huffman tables are not used.
		 */
		return;
	}

	/* First step of preparing Huffman tables for literal/length and
	 * distance alphabets is to decode the code lengths for these tables.
	 * The code lengths are encoded as a single integer sequence using
	 * the code length Huffman table.  The code length decoding supports
	 * repetition and zero filling.
	 */

	temp_code_lens = (unsigned char *) st->scratch + sizeof(st->scratch) - 320;
	for (i = 0; i != nlit + ndist;) {
		unsigned int rep_count;
		unsigned char rep_code;
		unsigned int t;

		t = lowzip_decode_huffman(st, (unsigned short *) st->scratch);
		if (t < 16) {
			rep_code = t;
			rep_count = 1;
		} else if (t == 16) {
			if (i == 0) {
				goto format_error;
			}
			rep_code = temp_code_lens[i - 1];
			rep_count = 3 + lowzip_read_bits(st, 2);
		} else if (t == 17) {
			rep_code = 0;
			rep_count = 3 + lowzip_read_bits(st, 3);
		} else {
			if (t != 18) {
				/* This should never happen regardless of
				 * input (even crafted) if the Huffman table
				 * has been prepared correctly.
				 */
				goto format_error;
			}
			rep_code = 0;
			rep_count = 11 + lowzip_read_bits(st, 7);
		}

		/* rep_count is at least 1 in every case, so progress
		 * is guaranteed.
		 */
		while (rep_count-- > 0) {
			if (i >= nlit + ndist) {
				/* Bounds check is needed for broken or
				 * crafted input.
				 */
				goto format_error;
			}
			temp_code_lens[i++] = rep_code;
		}
	}
	/* When we finish, i == nlit + ndist and the sequence has ended
	 * exactly without overwrite.
	 */

	/* We now have the code lengths and are ready to prepare the actual
	 * literal/length and distance Huffman tables.  The code length
	 * alphabet is no longer needed and we can overwrite it.
	 */

	lowzip_prepare_huffman(st,
			temp_code_lens,
			nlit,
			(unsigned short *) ((unsigned char *) st->scratch + LOWZIP_SCRATCH_HUFF_LIT));
	if (st->have_error) {
		return;
	}
	lowzip_prepare_huffman(st,
			temp_code_lens + nlit,
			ndist,
			(unsigned short *) ((unsigned char *) st->scratch + LOWZIP_SCRATCH_HUFF_DIST));
	if (st->have_error) {
		return;
	}

	/* Finally, decode the block contents. */

	lowzip_decode_huffman_block_data(st, 0 /*static_huffman*/);
	return;

	format_error:
	st->have_error = 1;
}

/* Deflate stream decoder, decode blocks until last block found. */
static void lowzip_decode_inflate_blocks(lowzip_state *st) {
	for (;;) {
		unsigned int blockhdr;

		if (st->have_error) {
			return;
		}

		/* Block header is BFINAL (1 bit) and BTYPE (2 bits).  Read
		 * as a single 3-bit field.  Due to deflate bit order, we get
		 * (BTYPE << 1) + BFINAL.
		 */
		blockhdr = lowzip_read_bits(st, 3);

		switch (blockhdr >> 1U) {
		case 0:
			/* Uncompressed. */
			lowzip_decode_uncompressed_block(st);
			break;
		case 1:
			/* Static Huffman. */
			lowzip_decode_static_huffman_block(st);
			break;
		case 2:
			/* Dynamic Huffman. */
			lowzip_decode_dynamic_huffman_block(st);
			break;
		default:
			/* Reserved/error. */
			st->have_error = 1;
			break;  /* Bail out on next loop. */
		}
		if (blockhdr & 0x01U) {
			/* BFINAL set, done. */
			break;
		}
	}
}

/* Main caller entrypoint.  Caller initializes the entire state structure
 * before making the call, and must check st->have_error after the call.
 * Decoded output is in [st->output_start,st->output_next[.
 */
void lowzip_inflate_raw(lowzip_state *st) {
	lowzip_reset_bitstate(st);
	lowzip_decode_inflate_blocks(st);
}

/*
 *  ZIP CRC32
 */

static unsigned int lowzip_zip_crc32(unsigned char *p_start, unsigned char *p_end) {
	unsigned int crc = 0xffffffffUL;
	int i;

	while (p_start < p_end) {
		crc ^= (unsigned int) (*p_start++);
		for (i = 0; i < 8; i++) {
			if (crc & 0x01UL) {
				crc = (crc >> 1U) ^ 0xedb88320UL;
			} else {
				crc = (crc >> 1U);
			}
		}
	}

	return crc ^ 0xffffffffUL;
}

/*
 *  ZIP operations
 */

/* Scan central directory for a file by index or name.  If found, return a
 * lowzip_file struct pointer.  The struct is allocated from a shared scratch
 * area in 'st' and is invalidated by another lowzip_locate_file() or a
 * lowzip_get_data() operation.  If file is not found, returns NULL and sets
 * st->have_error.
 */
lowzip_file *lowzip_locate_file(lowzip_state *st, int idx, const char *name) {
	unsigned int offset;
	unsigned int t;
	unsigned int filename_length;
	unsigned int i, n;
	unsigned int lhdr_offset;
	int found = 0;
	lowzip_file *fi;
	size_t name_length = 0;

	st->have_error = 0;

	if (name) {
		name_length = strlen(name);
	}

	offset = st->central_dir_offset;
	for (;;) {
		if (lowzip_read4(st, offset) != 0x02014b50UL) {
			/* Magic no longer matches, assume end of directory.
			 *
			 * There could also be a count/length limit, but that
			 * shouldn't be necessary: the central directory
			 * entries are followed by another header type (if
			 * nothing else, end of central directory header) so
			 * we'll always terminate.  If the file ends, we'll
			 * read in zeroes and terminate.
			 */
			break;
		}

		filename_length = lowzip_read2(st, offset + 28);
#if 0
		fprintf(stderr, "[");
		for (i = 0; i < filename_length; i++) {
			t = lowzip_read1(st, offset + LOWZIP_MIN_CDIRFILE_LENGTH + i);
			fprintf(stderr, "%c", (int) t);
		}
		fprintf(stderr, "]\n");
		fflush(stderr);
#endif
		if (name) {
			if (filename_length == name_length) {
				for (i = 0; i < filename_length; i++) {
					t = lowzip_read1(st, offset + LOWZIP_MIN_CDIRFILE_LENGTH + i);
					if (t != (unsigned int) ((unsigned char *) name)[i]) {
						break;
					}
				}
				if (i == filename_length) {
					found = 1;
				}
			}
		} else {
			if (idx == 0) {
				found = 1;
			}
			idx--;
		}

		if (!found) {
			t = filename_length;
			t += lowzip_read2(st, offset + 30);  /* extra length */
			t += lowzip_read2(st, offset + 32);  /* comment length */
			offset += LOWZIP_MIN_CDIRFILE_LENGTH + t;
			continue;
		}

		/* Matching entry found.  Store the local file header offset
		 * as the "current file" in the lowzip state.  File info
		 * queries and content reading will parse the local file
		 * header which duplicates most of the central directory
		 * fields.
		 */
		lhdr_offset = lowzip_read4(st, offset + 42);

		t = lowzip_read4(st, lhdr_offset);
		if (t != 0x04034b50UL) {
			/* Local file header corrupt. */
			break;
		}

		fi = (lowzip_file *) st->scratch;

		fi->compression_method = lowzip_read2(st, lhdr_offset + 8);
		fi->crc32 = lowzip_read4(st, lhdr_offset + 14);
		fi->compressed_size = lowzip_read4(st, lhdr_offset + 18);
		fi->uncompressed_size = lowzip_read4(st, lhdr_offset + 22);
		t = lowzip_read2(st, lhdr_offset + 26);
		t += lowzip_read2(st, lhdr_offset + 28);
		fi->data_offset = lhdr_offset + LOWZIP_MIN_LOCFILE_LENGTH + t;

		n = filename_length > sizeof(fi->filename) - 1 ? sizeof(fi->filename) - 1 : filename_length;
		for (i = 0; i < n; i++) {
			t = lowzip_read1(st, offset + LOWZIP_MIN_CDIRFILE_LENGTH + i);
			fi->filename[i] = (unsigned char) t;
		}
		fi->filename[n] = 0;

		/* 'fi' is valid for current file until the file data is
		 * read; that may involve inflating which overwrites the
		 * scratch area.
		 */
		return fi;
	}

	st->have_error = 1;
	return NULL;
}

/* Open a ZIP archive.  User code initializes the state argument 'st',
 * filling required fields like the read callback.  There's no need for
 * a free mechanism at present.  If init fails, st->have_error is set.
 *
 * At present initialization simply means locating the end of central
 * directory header and storing an offset to the central directory into
 * 'st' for later use.
 *
 * There's no 100% reliable method for scanning the ZIP file end of central
 * directory: it must be scanned backwards from the end of the file, but
 * there's a variable size comment field which may contain data matching the
 * end of central directory structure.  Our approach (which is good enough in
 * practice) is to check that the magic matches, and that the comment length
 * matches file size.
 * See https://github.com/thejoshwolfe/yauzl/issues/48#issuecomment-266587526.
 */
void lowzip_init_archive(lowzip_state *st) {
	int offset;
	unsigned int cdir_offset;

	st->have_error = 0;

	/* 'offset' is signed on purpose so that if st->zip_length is very
	 * small, the loop breaks out immediately.
	 */
	for (offset = (int) st->zip_length - LOWZIP_MIN_EOCDIR_LENGTH; offset >= (int) st->zip_length - LOWZIP_MAX_EOCDIR_LENGTH; offset--) {
		if (st->have_error) {
			break;
		}
		if (lowzip_read4(st, offset) != 0x06054b50UL) {
			continue;
		}

		unsigned int size = offset + LOWZIP_MIN_EOCDIR_LENGTH + lowzip_read2(st, offset + 20);

		// zip_length - 1 is a hack to make this work with imports from VESC Tool
		if (size != st->zip_length && size != (st->zip_length - 1)) {
			continue;
		}

		/* Central directory starting offset.  Ignores multiple disk
		 * ZIP files, i.e. the starting disk number (multiple disks
		 * are not supported -nor- checked for).
		 */
		cdir_offset = lowzip_read4(st, offset + 16);
		st->central_dir_offset = cdir_offset;
		return;
	}

	/* Not found. */
	st->have_error = 1;
}

/* Read the data for a file most recently located using lowzip_locate_file().
 * File data can be Store or Deflate compressed.  Getting the data invalidates
 * the lowzip_file struct data returned by lowzip_locate_file().
 *
 * The caller must provide space for the file data, and initialize
 * st->output_start, st->output_end, and st->output_next (set to the same
 * value as st->output_start).  If the space is too small, an error will be
 * signalled without any memory unsafe behavior.
 *
 * If getting the file data is successful, st->have_error will be zero and the
 * file data has been written to [st->output_start,st->output_next[ (end point
 * exclusive).
 *
 * If any error occurs, st->have_error will be set.  The st->output_next
 * pointer may have been advanced, but the data written should be ignored.
 */
void lowzip_get_data(lowzip_state *st) {
	lowzip_file *fi;
	unsigned int t;
	unsigned int offset;
	unsigned int offset_end;
	unsigned int header_crc32;
	unsigned int header_uncompressed_size;
	unsigned int computed_crc32;

	st->have_error = 0;

	fi = (lowzip_file *) st->scratch;
	header_crc32 = fi->crc32;
	header_uncompressed_size = fi->uncompressed_size;

	if (fi->compression_method == LOWZIP_COMPRESSION_STORE) {
		offset = fi->data_offset;
		offset_end = fi->data_offset + fi->uncompressed_size;
		for (; offset < offset_end; offset++) {
			if (st->have_error) {
				goto fail;
			}
			t = lowzip_read1(st, offset);
			lowzip_write_byte(st, (unsigned char) t);
		}
	} else if (fi->compression_method == LOWZIP_COMPRESSION_DEFLATE) {
		st->read_offset = fi->data_offset;
		lowzip_inflate_raw(st);
	} else {
		goto fail;
	}

	/* Delayed error check. */
	if (st->have_error) {
		return;
	}

	/* Minimal validation: output length and CRC32. */
	if ((ptrdiff_t) (st->output_next - st->output_start) != (ptrdiff_t) header_uncompressed_size) {
		goto fail;
	}

	if (st->write_sync_callback) {
		st->write_sync_callback(st->udata_write);
	}

	if (header_crc32 != 0) {
		computed_crc32 = lowzip_zip_crc32(st->output_start, st->output_next);
		if (computed_crc32 != header_crc32) {
			goto fail;
		}
	}

	/* All checks out. */
	return;

	fail:
	st->have_error = 1;
}
