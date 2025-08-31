#!/bin/bash

# move to doc directory
cd ../doc

DIR=".mkdocs_venv"
if [ -d "$DIR" ]; then
    echo "venv for mkdocs exists. Activating."
    source .mkdocs_venv/bin/activate
else
    echo "mkdocs_venv does not exist. Creating venv."
    # create venv
    python3 -m venv .mkdocs_venv
    source .mkdocs_venv/bin/activate
    # install pip
    python -m ensurepip --default-pip
    pip install --upgrade pip
    # install mkdocs
    pip install mkdocs mkdocs-material mkdocs-jupyter
fi

echo "Publishing documentation"
mkdocs serve