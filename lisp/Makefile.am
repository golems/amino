if HAVE_COMMON_LISP

bin_PROGRAMS = aarx.core

# Keep this list in sync with amino.asd
aarx_core_SOURCES = \
	package.lisp \
	basic-type.lisp \
	grovel.lisp \
	ffi.lisp \
	basic-ops.lisp \
	foreign.lisp \
	blas.lisp \
	libc.lisp \
	amino-la.lisp \
	op.lisp \
	generic.lisp \
	blas-generic.lisp \
	mem.lisp \
	io.lisp \
	tf-type.lisp \
	tf.lisp \
	tf-op.lisp \
	rx/robray-package.lisp \
	rx/rx-grovel.lisp \
	rx/rx-lib.lisp \
	rx/rx-type.lisp \
	rx/geom-bind.lisp \
	rx/cl-init.lisp \
	rx/sg-cfun.lisp \
	rx/rx-sg.lisp \
	rx/rx-gl.lisp \
	rx/rx-win.lisp \
	rx/rx-cl.lisp \
	rx/rx-mp.lisp \
	rx/rx-ik.lisp \
	rx/util.lisp \
	rx/mesh.lisp \
	rx/wavefront.lisp \
	rx/parameters.lisp \
	rx/povray.lisp \
	rx/scenegraph.lisp \
	rx/scenefile/urdf.lisp \
	rx/lexer.lisp \
	rx/inex.lisp \
	rx/scenefile/curly.lisp \
	rx/scenefile/moveit.lisp \
	rx/scenefile/scenefile.lisp \
	rx/animate.lisp \
	rx/draw.lisp \
	rx/trajectory.lisp \
	rx/config.lisp \
	rx/sg-gen.lisp

aarx.core: $(top_srcdir)/share/make-aarx.lisp $(aarx_core_SOURCES)
	$(top_srcdir)/share/make-aarx.sh $(COMMON_LISP) $(top_srcdir)

endif # HAVE_COMMON_LISP
