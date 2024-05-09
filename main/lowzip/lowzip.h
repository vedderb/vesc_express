#if !defined(LOWZIP_H_INCLUDED)
#define LOWZIP_H_INCLUDED

/* Read callback, limited to single byte reads at present for simplicity.
 * Return value is a byte in range [0x00,0xff] or 0x100 if out of bounds
 * or any other error.
 */
typedef unsigned int (*lowzip_read_callback)(void *udata, unsigned int offset);
typedef void (*lowzip_write_callback)(void *udata, int byte);
typedef void (*lowzip_write_sync_callback)(void *udata);

/* Lowzip state structure, allocated and initialized (partially) by caller.
 * Also contains the inflate state.
 */
typedef struct {
	/* Userdata for read callback. */
	void *udata;

	/* User-provided read callback to access the ZIP file. */
	lowzip_read_callback read_callback;

	void *udata_write;
	lowzip_write_callback write_callback;
	lowzip_write_sync_callback write_sync_callback;

	/* ZIP file length. */
	unsigned int zip_length;

	/* Output buffer for ZIP file data output. */
	unsigned char *output_start;
	unsigned char *output_end;
	unsigned char *output_next;  /* Initialize to 'output_start'. */

	/* Offset to start of central header. */
	unsigned int central_dir_offset;

	/* Error flag, for delayed error detection. */
	int have_error;

	/* Read offset (used by inflate code). */
	unsigned int read_offset;

	/* State for bitstream decoding (used by inflate code). */
	unsigned int curr;
	unsigned int have;

	/* Temporary scratch area used by both ZIP parsing and inflate.
	 * Huffman decoding needs the largest state; for size calculation
	 * see comments in prepare_huffman().  Declared as an array of
	 * 16-bit values rather than bytes to ensure alignment.
	 *
	 * Size breakdown for Huffman decoding:
	 *   32 + 572 bytes = 604 bytes for literal/length Huffman table
	 *   32 + 64 bytes  = 96 bytes for distance Huffman table
	 *   288 + 32 bytes = 320 bytes for nlit+ndist temporary code lengths
	 *   = 1020 bytes --> 510 16-bit ints.
	 */
	unsigned short scratch[510];
} lowzip_state;

/* Metadata about the most recent file header looked up from the ZIP file. */
typedef struct {
	/* Compression method: 0=Store, 8=Deflate. */
	unsigned int compression_method;

	/* CRC-32. */
	unsigned int crc32;

	/* Compressed size. */
	unsigned int compressed_size;

	/* Uncompressed size. */
	unsigned int uncompressed_size;

	/* Offset to start of compressed data. */
	unsigned int data_offset;

	/* Filename, truncated to 255 characters.  ZIP filenames can be
	 * 65535 bytes long, but 255 is enough in practice.
	 */
	char filename[255+1];
} lowzip_file;

/* ZIP API */
extern void lowzip_init_archive(lowzip_state *st);
extern lowzip_file *lowzip_locate_file(lowzip_state *st, int idx, const char *name);
extern void lowzip_get_data(lowzip_state *st);

/* Raw inflate call; not intended to be used directly. */
extern void lowzip_inflate_raw(lowzip_state *st);

#endif  /* LOWZIP_H_INCLUDED */
