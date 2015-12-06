if HAVE_COMMON_LISP

bin_PROGRAMS = aarx.core

aarx_core_SOURCES = \
	amino-la.lisp \
	basic-ops.lisp \
	basic-type.lisp \
	blas-generic.lisp \
	blas.lisp \
	deprecated.lisp \
	ffi.lisp \
	foreign.lisp \
	generic.lisp \
	grovel.lisp \
	io.lisp \
	libc.lisp \
	mem.lisp \
	op.lisp \
	package.lisp \
	test.lisp \
	tf.lisp \
	tf-op.lisp \
	tf-type.lisp \
	rx/povray.lisp \
	rx/animate.lisp \
	rx/config.lisp \
	rx/animation-plan.lisp \
	rx/draw.lisp \
	rx/lexer.lisp \
	rx/assimp.lisp \
	rx/scenefile/curly.lisp \
	rx/scenefile/scenefile.lisp \
	rx/scenefile/urdf.lisp \
	rx/scenefile/moveit.lisp \
	rx/scenegraph.lisp \
	rx/wavefront.lisp \
	rx/util.lisp \
	rx/robray-package.lisp \
	rx/collada.lisp \
	rx/mesh.lisp \
	rx/trajectory.lisp \
	rx/parameters.lisp \
	rx/inex.lisp \
	rx/sg-gen.lisp

aarx.core: $(top_srcdir)/share/make-aarx.lisp $(aarx_core_SOURCES)
	$(top_srcdir)/share/make-aarx.sh $(COMMON_LISP) $(top_srcdir)

endif # HAVE_COMMON_LISP
