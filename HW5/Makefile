
.PHONY: all clean

all: HW5

clean:
	-rm -f *.aux
	-rm -f *.lof
	-rm -f *.log
	-rm -f *.lot
	-rm -f *.toc
	-rm -f *.out

HW5: hw5.tex
	pdflatex -interaction=nonstopmode --shell-escape hw5.tex;pdflatex -interaction=nonstopmode --shell-escape hw5.tex
