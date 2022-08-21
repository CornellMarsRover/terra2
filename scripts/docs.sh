#!/bin/bash

# Generates doxygen documentation 

# Install doxygen theme
if [ ! -f install/doxygen-awesome-css-main/doxygen-awesome.css ]; then
  wget -O install/docs_theme.zip https://github.com/jothepro/doxygen-awesome-css/archive/refs/heads/main.zip
  unzip -d install install/docs_theme.zip
fi

doxygen Doxyfile