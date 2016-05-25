FAQ
===

SE(3)
====

* Q: Why dual quaternions?

  - A: Dual quaternions are more compact and easier, computationally, to
    normalize and filter than matrices.

Scene Graphs
============

* Q: How can I load a URDF file?

  - A: Use the scene graph compiler, `aarxc`, to generate code from the
    URDF file.  Note that URDF support has additional dependencies;
    see `./INSTALL` for details.

* Q: How can I reload a modified scene graph file without restarting my
  program?

  - A: Compile the scene graph to a shared library and load the library
    with dlopen().  To reload the scene graph, recompile the scene
    graph and dlopen() the library again.

Common Errors
=============

* Q: `./configure` fails with when checking for cffi-grovel.

    - A: Older versions of SBCL (around 1.2.4) have issues with current
      versions of CFFI.  Please try installing a recent SBCL (>1.3.4).

* Q: I get error messages about missing .obj files or Blender being
  unable to convert a .dae to Wavefront OBJ.

  - A: We use Blender to convert various mesh formats to Wavefront OBJ,
    then import the OBJ file.  The Blender binaries in the Debian and
    Ubuntu repositories (as of Jessie and Trusty) are not built with
    COLLADA (.dae) support.  You can download the prebuilt binaries
    from http://www.blender.org/ which do support COLLADA.

* Q: When I try to compile a URDF file, I receive the error "aarx.core:
  not found".

  - A: URDF support in amino is only built if the necessary dependencies
    are installed.  Please ensure that you have SBCL, Quicklisp, and
    Sycamore installed and rebuild amino if necessary.  See
    `./INSTALL` for details.

* Q: When building aarx.core, I get an enormous stack trace, starting
  with:
  `Unable to load any of the alternatives:
     ("libamino_planning.so" (:DEFAULT "libamino_planning"))`.

  - A: This means that SBCL is unable to load the planning library or one
    of its dependecies, such as OMPL.  Typically, this means your
    linker is not configured properly.

    Sometimes, you just need to run `ldconfig` or `sudo ldconfig` to
    update the linker cache.

    If this doesn't work, you can set the LD_LIBRARY_PATH variable.
    First, find the location of libompl.so, e.g., by calling `locate
    libompl.so`.  Then, add the directory to your LD_LIBRARY_PATH
    variable.  Most commonly, this will mean adding one of the
    following lines to your shell startup files (e.g., .bashrc):

    `export LD_LIBRARY_PATH="/usr/local/lib/:$LD_LIBRARY_PATH"`

    `export LD_LIBRARY_PATH="/usr/local/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH"`

Performance
===========

* Q: How do I make it faster?

  - A: Numerical code often benefits from newer CPU instructions. Try
    compiling amino with -march=native either via CFLAGS="-O2
    -march=native ./configure or adding the equivalent to your
    Autoconf site defaults file (config.site).

  - A: An optimized BLAS library will also help some
    operations. OpenBLAS is among the fastest
    (http://www.openblas.net/). If installed, configure amino to use
    it with ./configure --with-blas=openblas.

* Q: Ray Tracing is SLOOOWWW!

  - A: Ray tracing is computationally expensive. Here are a few notes
    to help performance.

    Distribute: Ray tracing is embarassingly parallel, both across
    multiple frames and across pixels within a frame. The easiest way
    to render faster is to throw more cores at the problem.

    Parsing: While POV-Ray can use multiple threads to render a frame,
    parsing the input file uses only a single core. If large meshes
    are used, this can take a noticeable amount of time, and must be
    repeated for each frame. Consequently, rather than using multiple
    threads per frame, it may often be better to render multiple
    frames in parallel so that parsing is parallelized across the
    different frames.

    Optimize POV-Ray: Ray tracing is floating-point heavy. A few
    compilation flags make a small but noticable (~10%) improvement in
    performance.

      * `-march=native`: optimize for the current CPU. This may take
        advantage of FPU instructions available on newer CPUs, e.g.,
        AVX.
      * `-ffast-math`: optimize floating-point computation in ways that
        my violate IEEE semantics. Note that this could flag may
        slightly change the resulting image.
      * `-flto`: enable link-time optimization. This optimizes across
        multiple translation units.
