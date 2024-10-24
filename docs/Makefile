# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
apidoc:
	sphinx-apidoc -f -o source/modules ../src/r0b0 --separate

md_rst:
	pandoc -s *.md -o source/*.rst
	pandoc -s README.md -o source/README.rst
	pandoc -s blsm.md -o source/blsm.rst


rebuild:
	make apidoc
	make md_rst
	make html

serve:
	python3 -m http.server --directory build/html 7777

docpush:
	make rebuild
	./deploy.sh