TEX := hw6

all: HW6

clean:
	-rm -f hw6.pdf

HW6: hw6.tex
	latexmk -pdf -pdflatex="pdflatex -interaction=nonstopmode" --shell-escape hw6.tex
