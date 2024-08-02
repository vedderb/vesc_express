# lowzip

Lowzip is a footprint optimized ZIP decompressor with Deflate support.

Lowzip is intended to be used in portability challenged, low memory embedded
environments.  ZIP is a useful format in even embedded environments:

* It allows multiple files to be distributed as a single archive file.

* It allows file compression with the ability to extract individual files
  (unlike, for example, tar.gz).

* It has good multi-platform tool support.

* ZIP files can be appended to other files (such as executables) and the
  leading data is ignored.

As a concrete use case, ZIP can be used as an application package format for
Javascript applications.  A single ZIP file can contain a JSON metadata file,
a main application Javascript file, loadable Javascript and native modules,
and asset files.  Script and JSON/text files compress reasonably well using
Deflate.

Current x64 code footprint (for lowzip.c, excluding the test program) is about
3.2kB and RAM footprint is about 1.1kB.

**Status: alpha**

## API summary

See `test_lowzip.c`.

Initialize access to archive:

```c
lowzip_state st;  /* Allocated by caller, e.g. from stack frame, around 1.1kB. */

memset((void *) &st, 0, sizeof(st));
st.udata = (void *) my_udata;  /* May be used by read_callback. */
st.read_callback = my_read_callback;
st.zip_length = zip_file_length;

lowzip_init_archive(&st);

/* Success is indicated via st.have_error. */
if (st.have_error) {
    printf("Failed to init archive\n");
} else {
    printf("Archive init OK\n");
}
```

There are no dynamic allocations related to the state, and there's no method
to close a state.  Simply stop using it when you're done.

To scan filenames:

```c
int i;
lowzip_file *fi;

for (i = 0; ; i++) {
    fi = lowzip_locate_file(&st, i, NULL);
    if (!fi) {
        break;
    }
    printf("File %d: %s, %ld -> %ld bytes\n", i, fi->filename,
           (long) fi->compressed_size, (long) fi->uncompressed_size);
}
```

Files can be located based on exact filename match or by index (like above).
To read a file, first locate it using `lowzip_locate_file()` and then call
`lowzip_get_data()`; here using an exact filename:

```c
lowzip_file *fi;

fi = lowzip_locate_file(&st, 0, "net/http.js");

if (fi) {
    unsigned int output_length = fi->uncompressed_size;

    /* Calling lowzip_get_data() invalidates the 'fi' file info struct
     * because they share the same internal scratch storage, so read any
     * required fields (like filename and length) before calling
     * lowzip_get_data().
     */

    /* Set up output area.  It should be at least fi->uncompressed_size long
     * (if not, the output is truncated and st.have_error will be set).
     */
    st.output_start = output_buffer;
    st.output_end = output_buffer + output_length;
    st.output_next = output_buffer;

    /* Request data to be unpacked. */
    lowzip_get_data(&st);

    /* Success is indicated via st.have_error.  Output data is in the range
     * [st.output_start,st.output_next[ (end point exclusive) if success.
     */
    if (st.have_error) {
        printf("Failed to read file\n");
    } else {
        printf("Successfully unpacked, data follows:\n");
        fwrite((void *) st.output_start, 1, (size_t) (st.output_next - st.output_start), stdout);
    }
} else {
    printf("File not found\n");
}
```

## Designed for embedded environments

* Unzip only because ZIP files are rarely created by low memory embedded
  targets.

* Narrow ZIP feature support aimed at common/default ZIP options.

* Minimal code and RAM footprint, at the expense of speed.

* Supports Store (no compression, algorithm 0) and Deflate (algorithm 8).
  Having support for Deflate is useful because the inflate algorithm code
  footprint is very often offset by gains in compressing script/data files.

* No dynamic allocations (caller provided buffers) to avoid the platform
  dependency and memory churn related to dynamic allocation.  This also
  allows the caller to potentially reuse (otherwise unrelated) existing
  buffers which are unused at the time.  The caller can also easily
  allocate buffers from custom memory pools which are often used in script
  environments like Duktape or Lua.

* No file I/O calls, ZIP file is read using a caller provided read callback.

* Inflate output is written to a caller allocated buffer; the output
  buffer is also used for inflate backwards references so that the 32kB
  inflate window has no additional memory footprint.  Unzip/inflate output
  data cannot be streamed however.

## Limitations

* Unzip only.

* Only Store (algorithm 0) and Deflate (algorithm 8) compression methods
  supported.

* Validation is also minimal (except to guarantee memory safety).  However,
  file CRC-32 and length is validated after decompression.

* No support for generic flag bit 3 (crc32, compressed size, uncompressed
  size are zero and actual values follow compressed data).

* No support for encryption.

* No support for ZIP64.

* No support for multiple disk ZIP files (disk numbers are ignored).

## Resources

* https://en.wikipedia.org/wiki/Zip_%28file_format%29

* https://support.pkware.com/display/PKZIP/APPNOTE

* https://en.wikipedia.org/wiki/DEFLATE

* https://www.ietf.org/rfc/rfc1951.txt

## Security considerations

Main security goal is memory safety and eventual termination against arbitrary
inputs.

## Design notes

### End of central directory scanning

There's no 100% reliable method for scanning the ZIP file end of central
directory: it must be scanned backwards from the end of the file, but there's
a variable size comment field which may contain data matching the end of
central directory structure.  See https://github.com/thejoshwolfe/yauzl/issues/48#issuecomment-266587526.

Good behavior is to scan backwards until the end of central directory magic
value is found, then validate the header (including comment length being
consistent with end of file).  If the validation fails, continue scanning.

## Future work

* RAM index for N first files: index can be 8 bytes per entry (4 + 4) by
  using a trivial string hash to identify filename so that each entry is
  (hash(filename), hdroffset).

* The 'codes' array contains 9-bit values which are now stored as 16-bit
  values.  Figure out a way to store them more efficiently, e.g. by storing
  the high bit as a separate bitmask, but without increasing code footprint
  too much.
