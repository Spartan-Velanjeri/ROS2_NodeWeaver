#!/bin/bash
cd $(dirname "$(readlink -f "$0")")
streamlit run HOME.py
