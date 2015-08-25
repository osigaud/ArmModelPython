# Options for the makefile


# Name of the project (also the name of the output)
PROJECT = codyco

# Other folder to compile
DIRS =

# Output of the full compilation
#   dvi : compile with LaTeX and stop after its dvi output
#   ps : compile with LaTeX and apply the dvips command on the dvi
#   pdf : compile with pdflatex
LATEXFORMAT = pdf

# Use BiBTeX to generate the bilbiography
#   true: use bibtex
#   false : doesn't use bibtex for the bibliography
BIBLIOGRAPHY = true

# Use makeindex to generate the index
#   true: use makeindex
#   false : doesn't use makeindex
INDEX = false

# Multiple files
#  true  - each tex file is compiled
#  false - only $(PROJECT).tex is compiled
MULTIPLEFILES = false

# Dictionary to use to correct the document
DICTIONARY = english

# Verbose compiling
#   yes: command line used are displayed
#   no: command line used are not displayed
VERBOSE = no

DOT_GRAPH = $(wildcard *.dot)

DOT_TEX = $(patsubst %.dot,%.dot_t,$(DOT_GRAPH))

%.dot_t: %.dot makefile
	dot -Txdot -Nshape=none $< | dot2tex -s -f pgf --graphstyle="scale=0.35" --template=dot2tex.template -t math -o $@

#*****************************************************************#
#*                                                               *#
#* You shouldn't have to modify this part. You have to know      *#
#* exactly what you are doing. If you really want to modify it   *#
#* then maybe it is better to modify directly the template       *#
#* associated                                                    *#
#*                                                               *#
#*****************************************************************#


#
# Define default options for the project. They are used only
# if they are not already defined
#

# Default name of the output
ifeq ($(strip $(PROJECT)),)
PROJECT = project
endif

# Default for generating the bibliography using BiBTeX
ifeq ($(strip $(BIBLIOGRAPHY)),)
BIBLIOGRAPHY = false
endif

# Output of the full compilation
#   dvi : compile with LaTeX and stop after its dvi output
#   ps : compile with LaTeX and apply the dvips command on the dvi
#   pdf : compile with pdflatex
ifeq ($(strip $(LATEXFORMAT)),)
LATEXFORMAT = ps
endif

# Multiple files
#  true  - each tex file is compiled
#  false - only $(PROJECT).tex is compiled
ifeq ($(strip $(MULTIPLEFILES)),)
MULTIPLEFILES = false
endif


#
# Define the default program to use. Same as options: they
# are used only if they are not already defined
#

# Default LaTeX compiler
ifeq ($(strip $(LATEX_COMPILER)),)
LATEX_COMPILER = latex
endif

# Default LaTeX compiler
ifeq ($(strip $(PDFLATEX_COMPILER)),)
PDFLATEX_COMPILER = pdflatex
endif

# Default BIBTeX compiler
ifeq ($(strip $(BIBTEX_COMPILER)),)
BIBTEX_COMPILER = bibtex
endif

# Default pdf compiler
ifeq ($(strip $(PS2PDF)),)
PS2PDF = ps2pdf
endif

# Default ps converter
ifeq ($(strip $(DVI2PS)),)
DVI2PS = dvips
endif

# Default dvi viewer
ifeq ($(strip $(DVI_VIEWER)),)
DVI_VIEWER = xdvi
endif

# Default ps viewer
ifeq ($(strip $(PS_VIEWER)),)
PS_VIEWER = gv
endif

# Default pdf viewer
ifeq ($(strip $(PDF_VIEWER)),)
PDF_VIEWER = acroread
endif

#
# Define command line options from the makefile options
#

# Set the debug mark if the verbose option is no. It will be used
# when calling the command line
ifeq ($(VERBOSE),no)
 DEBUGMARK = @
endif

# Define the different command used
LATEX= $(DEBUGMARK)$(LATEX_COMPILER)
PDFLATEX= $(DEBUGMARK)$(PDFLATEX_COMPILER)
DVIPS = $(DEBUGMARK)$(DVI2PS)
PSPDF= $(DEBUGMARK)$(PS2PDF)
BIBTEX= $(DEBUGMARK)$(BIBTEX_COMPILER)


#
# The next step is to make lists of the files we want to compile
#

# File reference to build
PROJECTFILE = $(PROJECT)

# List of the source files
SRCS := $(wildcard *.tex)

# Define the list of files to compile

# Default LaTeX file output
ifeq ($(LATEXFORMAT),dvi)
FILESTARGETSUFFIXE = dvi
VIEWER = $(DVI_VIEWER)
endif
ifeq ($(LATEXFORMAT),ps)
FILESTARGETSUFFIXE = ps
VIEWER = $(PS_VIEWER)
endif
ifeq ($(LATEXFORMAT),pdf)
FILESTARGETSUFFIXE = pdf
VIEWER = $(PDF_VIEWER)
endif


ifeq ($(MULTIPLEFILES),true)
FILESTARGET = $(patsubst %.tex,%.$(FILESTARGETSUFFIXE),$(wildcard *.tex))
VIEWEDFILES = $(patsubst %.tex,%.$(FILESTARGETSUFFIXE),$(wildcard *.tex))
DEPENDENCIES =
else
FILESTARGET = $(PROJECTFILE).$(FILESTARGETSUFFIXE)
VIEWEDFILES = $(PROJECTFILE).$(FILESTARGETSUFFIXE)
DEPENDENCIES = $(wildcard *.tex)
endif

#
# All the options and commands are now define. So, we can start
# to defining the target to build the program
#


# This is the default target. It points to the other folder to build
# and the program to build
all: subdirs $(INKSCAPE_PDF) $(DOT_TEX) $(FILESTARGET) makefile

# Start by compiling the other folders
GOALS := $(subst tests,$(EMPTY),$(MAKECMDGOALS))
subdirs:
  ifneq ($(strip $(DIRS)),)
	@for dir in $(DIRS); do $(MAKE) -C $$dir $(GOALS); done
  endif

$(PROJECTFILE) : all


%.dvi : %.tex $(DEPENDENCIES) makefile
	@echo "Compiling $@..."
  ifeq ($(BIBLIOGRAPHY),true)
	-$(LATEX) -interaction=nonstopmode $<
	-$(BIBTEX) $(basename $<)
  endif
	$(LATEX) -interaction=nonstopmode $<
  ifeq ($(INDEX),true)
	makeindex $(addsuffix .idx, $(basename $<))
  endif
	$(LATEX) -interaction=nonstopmode $<
	$(LATEX) -interaction=nonstopmode $<
  ifeq ($(LATEXFORMAT),dvi)
	@mv $(addsuffix .dvi, $(basename $<)) tmp.dvi
	@mv tmp.dvi $(PROJECTFILE).dvi
  endif

%.ps : %.dvi
	@echo "Converting $@ into postcript format..."
	$(DVIPS) $< -o $(addsuffix .ps, $(basename $<))

stamp:
	touch stamp

LATEX_FILTER="paper.tex|Underfull|Overfull|warning|rror:"

%.pdf : %.tex makefile $(INKSCAPE_PDF) stamp
	@echo "Compiling $@..."
	-$(PDFLATEX) -file-line-error -interaction=nonstopmode $< | grep -i -E $(LATEX_FILTER)
  ifeq ($(BIBLIOGRAPHY),true)
	-$(BIBTEX) $(basename $<) -terse
  endif
	@sleep 1
	@touch stamp
# 	$(PDFLATEX) -interaction=nonstopmode $<
# 	$(PDFLATEX) -interaction=nonstopmode $<
# 	$(PDFLATEX) -interaction=nonstopmode $<


#
# We define a target to be able to clean all the build files
#
clean: subdirs
ifneq ($(strip $(wildcard *~)),)
	@echo "Cleaning backup file(s)..."
	-@rm $(wildcard *~)
endif

ifneq ($(strip $(wildcard $(PROJECTFILE))),)
	@echo "Cleaning project file(s)..."
	-@rm $ $(PROJECTFILE)
endif

ifneq ($(strip $(wildcard *.dvi)),)
	@echo "Cleaning dvi file(s)..."
	-@rm $(wildcard *.dvi)
endif
ifneq ($(strip $(wildcard paper.pdf)),)
	@echo "Cleaning pdf file(s)..."
	-@rm $(wildcard paper.pdf)
endif
ifneq ($(strip $(wildcard *.ps)),)
	@echo "Cleaning postscript file(s)..."
	-@rm $(wildcard *.ps)
endif
#Clean all latex files
	@echo "Cleaning LaTeX file(s)..."
ifneq ($(strip $(wildcard *.aux)),)
	-@rm $(wildcard *.aux)
endif
ifneq ($(strip $(wildcard *.log)),)
	-@rm $(wildcard *.log)
endif
ifneq ($(strip $(wildcard *.out)),)
	-@rm $(wildcard *.out)
endif
ifneq ($(strip $(wildcard *.bbl)),)
	-@rm $(wildcard *.bbl)
endif
ifneq ($(strip $(wildcard *.blg)),)
	-@rm $(wildcard *.blg)
endif
ifneq ($(strip $(wildcard *.glo)),)
	-@rm $(wildcard *.glo)
endif
ifneq ($(strip $(wildcard *.toc)),)
	-@rm $(wildcard *.toc)
endif
ifneq ($(strip $(wildcard *.ilg)),)
	-@rm $(wildcard *.ilg)
endif
ifneq ($(strip $(wildcard *.gls)),)
	-@rm $(wildcard *.gls)
endif
ifneq ($(strip $(wildcard *.idx)),)
	-@rm $(wildcard *.idx)
endif
ifneq ($(strip $(wildcard *.ind)),)
	-@rm $(wildcard *.ind)
endif
ifneq ($(strip $(wildcard *.lof)),)
	-@rm $(wildcard *.lof)
endif



#
# We define a target to be able to run the program
# This will disappear when stufe will be finished
#

# We first fix this variable in case a script
# file has been found in the current folder
SCRIPT := $(wildcard script)

tests: subdirs $(PROJECTFILE)
# ifeq ($(strip $(SCRIPT)),)
# # No script file has been found
# 	@for file in $(VIEWEDFILES); do $(VIEWER) $$file; done
# else
# 	./$(SCRIPT)
# endif


.PHONY : all tests clean subdirs


