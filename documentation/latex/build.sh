#!/bin/bash

# RUS Documentation Build Script
# Builds the comprehensive LaTeX documentation with proper dependencies

echo "=== Building RUS Technical Documentation ==="

# Check if required tools are installed
command -v pdflatex >/dev/null 2>&1 || { echo "pdflatex is required but not installed. Aborting." >&2; exit 1; }
command -v biber >/dev/null 2>&1 || { echo "biber is required but not installed. Aborting." >&2; exit 1; }

# Navigate to the LaTeX directory
cd "$(dirname "$0")"

# Clean previous build artifacts
echo "Cleaning previous build artifacts..."
rm -f *.aux *.bbl *.bcf *.blg *.fdb_latexmk *.fls *.log *.out *.run.xml *.synctex.gz *.toc *.lof *.lot *.lol
rm -f sections/*.aux
rm -f appendices/*.aux

echo "Starting LaTeX compilation..."

# First pass - generate aux files
echo "Pass 1: Initial compilation..."
pdflatex -interaction=nonstopmode -halt-on-error main.tex

if [ $? -ne 0 ]; then
    echo "Error: First LaTeX pass failed"
    exit 1
fi

# Run biber for bibliography
echo "Processing bibliography..."
biber main

if [ $? -ne 0 ]; then
    echo "Error: Bibliography processing failed"
    exit 1
fi

# Second pass - process bibliography references
echo "Pass 2: Processing references..."
pdflatex -interaction=nonstopmode -halt-on-error main.tex

if [ $? -ne 0 ]; then
    echo "Error: Second LaTeX pass failed"
    exit 1
fi

# Third pass - finalize cross-references
echo "Pass 3: Finalizing cross-references..."
pdflatex -interaction=nonstopmode -halt-on-error main.tex

if [ $? -ne 0 ]; then
    echo "Error: Third LaTeX pass failed"
    exit 1
fi

# Check if PDF was generated successfully
if [ -f "main.pdf" ]; then
    echo "=== Build Successful ==="
    echo "Generated: main.pdf"
    
    # Optional: Open PDF viewer (uncomment if desired)
    # if command -v open >/dev/null 2>&1; then
    #     open main.pdf
    # elif command -v xdg-open >/dev/null 2>&1; then
    #     xdg-open main.pdf
    # fi
    
    # Display file size and page count
    if command -v pdfinfo >/dev/null 2>&1; then
        PAGES=$(pdfinfo main.pdf | grep "Pages:" | awk '{print $2}')
        SIZE=$(ls -lh main.pdf | awk '{print $5}')
        echo "Document statistics: $PAGES pages, $SIZE"
    fi
    
else
    echo "Error: PDF generation failed"
    exit 1
fi

# Clean intermediate files (optional)
echo "Cleaning intermediate files..."
rm -f *.aux *.bbl *.bcf *.blg *.fdb_latexmk *.fls *.log *.out *.run.xml *.synctex.gz
rm -f sections/*.aux
rm -f appendices/*.aux

echo "=== Build Complete ==="
