all:
	./makelibref *.h > libref.tex
	pdflatex libref
	open libref.pdf

clean:
	rm -f *.tex
	rm -f libref.log
	rm -f libref.aux
	rm -f libref.pdf

