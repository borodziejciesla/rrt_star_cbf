#!/bin/bash

# Define the main file and the output file name
main_file="paper.tex"
output_name="paper"

# Define log and temporary files to be cleaned up
temp_files=(".aux" ".bbl" ".blg" ".log" ".nav" ".out" ".snm" ".toc" ".fls" ".fdb_latexmk")

# ---
## Build the PDF and the Bibliography

# Run pdflatex to generate .aux and other temporary files.
pdflatex $main_file

# Run bibtex to process the bibliography. This requires the .aux file.
bibtex $output_name

# Rerun pdflatex twice. The first run resolves citations, and the second
# updates the table of contents, cross-references, etc.
pdflatex $main_file
pdflatex $main_file


# ---
## Clean Up

# Build short version
short_file="paper_short.tex"
short_name="paper_short"

pdflatex $short_file
bibtex $short_name
pdflatex $short_file
pdflatex $short_file


echo "Cleaning up temporary files..."
for ext in "${temp_files[@]}"; do
    if [ -f "$output_name$ext" ]; then
        rm "$output_name$ext"
        echo "Removed $output_name$ext"
    fi

    if [ -f "$short_name$ext" ]; then
        rm "$short_name$ext"
        echo "Removed $short_name$ext"
    fi
done

echo "PDF build and cleanup complete."