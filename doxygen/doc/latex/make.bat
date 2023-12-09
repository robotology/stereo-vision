set Dir_Old=%cd%
cd /D %~dp0

del /s /f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl refman.pdf

set LATEX_CMD=latex
%LATEX_CMD% refman.tex
echo ----
makeindex refman.idx
setlocal enabledelayedexpansion
set count=8
:repeat
set content=X
for /F "tokens=*" %%T in ( 'findstr /C:"Rerun LaTeX" refman.log' ) do set content="%%~T"
if !content! == X for /F "tokens=*" %%T in ( 'findstr /C:"Rerun to get cross-references right" refman.log' ) do set content="%%~T"
if !content! == X goto :skip
set /a count-=1
if !count! EQU 0 goto :skip

echo ----
%LATEX_CMD% refman.tex
goto :repeat
:skip
endlocal
makeindex refman.idx
%LATEX_CMD% refman.tex
dvips -o refman.ps refman.dvi
gswin32c -q -dNOPAUSE -dBATCH -sDEVICE=pdfwrite -sOutputFile=refman.pdf -c save pop -f refman.ps
