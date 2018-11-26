# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = Homework5
SOURCEDIR     = .
BUILDDIR      = _build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)


clean:
	-rm -f *.aux
	-rm -f *.lof
	-rm -f *.log
	-rm -f *.lot
	-rm -f *.toc
	-rm -f *.out

HW5: hw5.tex
	pdflatex -interaction=nonstopmode --shell-escape hw5.tex;pdflatex -interaction=nonstopmode --shell-escape hw5.tex
